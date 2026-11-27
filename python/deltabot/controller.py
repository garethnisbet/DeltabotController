"""Host side controller: talks the DeltaBot line protocol over a serial port
and exposes the machine as an XY stage in millimetres.

The three screws are over-constrained - two degrees of freedom driven by three
actuators - so the useful interface is :meth:`DeltaBot.move_xy`, which splits a
position between all three screws with nothing left in the direction that only
strains the flexures.  See :mod:`deltabot.kinematics`.
"""

import time
from typing import Dict, List, Optional, Sequence, Tuple

from .config import Connection, Geometry
from .kinematics import (Pose, common_mode, estimate_z, pose_to_screws,
                         reachable, screws_to_pose)


class DeltaBotError(RuntimeError):
    """The firmware answered with "err ...", or stopped answering."""


def list_ports() -> List[Tuple[str, str]]:
    """Every serial port the machine might be on, as (device, description)."""
    from serial.tools import list_ports as _lp

    return [(p.device, p.description) for p in _lp.comports()]


def find_port() -> str:
    """Best guess at the Nano's port.  Raises if there is nothing obvious."""
    ports = list_ports()
    if not ports:
        raise DeltaBotError("no serial ports found - is the Nano plugged in?")
    for device, description in ports:
        text = (device + " " + description).lower()
        if any(k in text for k in ("nano", "ch340", "ch34", "usb-serial", "ftdi",
                                   "ft232", "usb uart", "arduino")):
            return device
    # Built in ttyS ports are always listed and never the Nano.
    usb = [d for d, _ in ports if any(k in d for k in ("USB", "ACM", "usbserial"))]
    return usb[0] if usb else ports[0][0]


class DeltaBot:
    """Control the DeltaBot from Python.

        with DeltaBot() as bot:
            bot.zero()
            bot.move_xy(0.5, 0.0)
            print(bot.pose())

    Pass ``port="sim"`` to drive the built in simulator instead of hardware.
    """

    def __init__(self, port: str = "auto", geometry: Optional[Geometry] = None,
                 baud: int = 115200, timeout: float = 1.0, connect: bool = True):
        self.geometry = geometry or Geometry()
        self.conn = Connection(port=port, baud=baud, timeout=timeout)
        self.serial = None
        self.banner = ""
        self.limit_hits: List[int] = []     # switches tripped since the last move began
        if connect:
            self.open()

    # ------------------------------------------------------------- session
    def open(self) -> None:
        if self.serial is not None:
            return
        if self.conn.port == "sim":
            from .sim import SimSerial

            self.serial = SimSerial(self.geometry.default_speed, self.geometry.default_accel)
        else:
            import serial

            port = find_port() if self.conn.port == "auto" else self.conn.port
            self.serial = serial.Serial(port, self.conn.baud, timeout=self.conn.timeout)
            self.conn.port = port
            time.sleep(self.conn.reset_delay)   # the Nano reboots on open; its
                                                # banner waits in the buffer

        self.banner = self._drain_banner()
        self.set_speed(self.geometry.default_speed)
        self.set_accel(self.geometry.default_accel)
        self.apply_soft_limits()

    def close(self) -> None:
        if self.serial is not None:
            try:
                self.disable()
            except Exception:
                pass
            self.serial.close()
            self.serial = None

    def __enter__(self) -> "DeltaBot":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ------------------------------------------------------------ protocol
    def _drain_banner(self) -> str:
        deadline = time.time() + 3.0
        lines = []
        while time.time() < deadline:
            raw = self.serial.readline()
            if not raw:
                break
            lines.append(raw.decode(errors="replace").strip())
            if "ready" in lines[-1]:
                return lines[-1]             # skip any bootloader noise before it
        return " / ".join(l for l in lines if l)

    def command(self, text: str, timeout: float = 5.0) -> List[str]:
        """Send one command and return the data lines before its "ok"."""
        if self.serial is None:
            raise DeltaBotError("not connected")
        self.serial.write((text.strip() + "\n").encode())
        data: List[str] = []
        deadline = time.time() + timeout
        while time.time() < deadline:
            raw = self.serial.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if not line or line == "done":      # completion of an earlier move
                continue
            if line.startswith("limit"):         # a limit switch stopped a move
                self.limit_hits.append(int(line.split()[1]))
                continue
            if line == "ok":
                return data
            if line.startswith("err"):
                raise DeltaBotError(f"{text.strip()!r}: {line[4:].strip()}")
            data.append(line)
        raise DeltaBotError(f"timed out waiting for a reply to {text.strip()!r}")

    # -------------------------------------------------------------- status
    def status(self) -> dict:
        """Parse the firmware's status line into a dict."""
        lines = self.command("?")
        if not lines or not lines[0].startswith("status"):
            raise DeltaBotError("no status returned")
        tokens = lines[0].split()
        out = {"moving": tokens[1] == "moving"}
        i = 2
        while i < len(tokens):
            key = tokens[i]
            if key in ("pos", "tgt", "limits", "sw"):
                width = 2 if key == "limits" else 3
                out[key] = [int(t) for t in tokens[i + 1:i + 1 + width]]
                i += width + 1
            else:
                out[key] = float(tokens[i + 1])
                i += 2
        out["en"] = bool(out.get("en", 1))
        if "sw" in out:
            out["sw"] = [bool(v) for v in out["sw"]]
        return out

    def limit_switches(self) -> List[bool]:
        """Whether each screw's limit switch is tripped (pressed, or its wire
        broken)."""
        s = self.status()
        if "sw" not in s:
            raise DeltaBotError(f"the firmware ({self.banner or 'unknown'}) does not report "
                                "limit switches - flash DeltaBot 1.1 or later")
        return s["sw"]

    def pin_levels(self) -> Dict[str, int]:
        """Raw level, 0 or 1, of D2..D13 and A0..A5 (firmware 1.2 or later).
        Every input has the internal pull-up, so an unused pin reads 1 and a
        pin that something ties to GND reads 0."""
        try:
            lines = self.command("I")
        except DeltaBotError as exc:
            raise DeltaBotError(f"the firmware ({self.banner or 'unknown'}) cannot report "
                                "pin levels - flash DeltaBot 1.2 or later") from exc
        for line in lines:
            if line.startswith("pins"):
                digital, analog = line.split()[1:3]
                names = [f"D{p}" for p in range(2, 14)] + [f"A{p}" for p in range(6)]
                return dict(zip(names, (int(c) for c in digital + analog)))
        raise DeltaBotError("no pin levels returned")

    def positions(self) -> List[int]:
        """Current position of each screw, in steps."""
        lines = self.command("P")
        for line in lines:
            if line.startswith("pos"):
                return [int(t) for t in line.split()[1:4]]
        raise DeltaBotError("no position returned")

    def extensions(self) -> List[float]:
        """Current extension of each screw, in mm from the zero position."""
        g = self.geometry
        return [g.steps_to_mm(i, s) for i, s in enumerate(self.positions())]

    def pose(self) -> Pose:
        """Current platform position in the plane."""
        return screws_to_pose(self.extensions(), self.geometry)

    def z(self) -> float:
        """Estimated height of the platform, mm.  Not a controlled axis: it
        falls out of how far off centre the platform has been driven."""
        extensions = self.extensions()
        return estimate_z(screws_to_pose(extensions, self.geometry),
                          self.geometry, extensions)

    def strain(self) -> float:
        """Common mode screw extension, mm.  This is the over-constrained
        direction: anything but zero here is the three legs fighting each
        other.  Commanded moves always leave it at zero; single screw jogs and
        lost steps do not."""
        return common_mode(self.extensions())

    def is_moving(self) -> bool:
        return self.status()["moving"]

    def wait(self, timeout: float = 300.0, poll: float = 0.05) -> None:
        """Block until the platform has stopped.  Raises if a limit switch
        stopped it short."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self.is_moving():
                if self.limit_hits:
                    hits = ", ".join(str(i) for i in sorted(set(self.limit_hits)))
                    raise DeltaBotError(f"limit switch {hits} tripped; motion stopped")
                return
            time.sleep(poll)
        raise DeltaBotError("timed out waiting for the move to finish")

    # ------------------------------------------------------------- motion
    def move_steps(self, a: int, b: int, c: int, relative: bool = False,
                   wait: bool = True) -> None:
        """Move the three screws to (or by) a number of steps."""
        self.limit_hits = []
        self.command(f"{'R' if relative else 'M'} {int(a)} {int(b)} {int(c)}")
        if wait:
            self.wait()

    def move_extensions(self, extensions: Sequence[float], relative: bool = False,
                        wait: bool = True) -> None:
        """Move the three screws to (or by) an extension in mm."""
        if len(extensions) != 3:
            raise ValueError("need exactly three screw extensions")
        g = self.geometry
        steps = [g.mm_to_steps(i, h) for i, h in enumerate(extensions)]
        self.move_steps(*steps, relative=relative, wait=wait)

    def move_xy(self, x: Optional[float] = None, y: Optional[float] = None,
                wait: bool = True) -> Pose:
        """Move the platform to a position in the plane, in mm from zero.

        Omitted axes keep their current value.  The move is split across all
        three screws so that nothing goes into the over-constrained direction.
        """
        current = self.pose()
        target = Pose(current.x if x is None else x, current.y if y is None else y)
        if not reachable(target, self.geometry):
            raise DeltaBotError(
                f"{target.r:.3f} mm is outside the {self.geometry.max_radius_mm:.3f} mm "
                "working radius")
        self.move_extensions(pose_to_screws(target, self.geometry), wait=wait)
        return target

    def nudge_xy(self, dx: float = 0.0, dy: float = 0.0, wait: bool = True) -> Pose:
        """Move by a relative distance in the plane."""
        p = self.pose()
        return self.move_xy(p.x + dx, p.y + dy, wait=wait)

    def move_polar(self, r: float, theta_deg: float, wait: bool = True) -> Pose:
        """Move to a radius and direction from centre."""
        from math import cos, radians, sin

        return self.move_xy(r * cos(radians(theta_deg)), r * sin(radians(theta_deg)),
                            wait=wait)

    def preload(self, mm: float, wait: bool = True) -> None:
        """Add a common mode extension to all three screws at once.

        This is the over-constrained direction: it does not move the platform
        in the plane, it pushes the three legs against each other, which is
        occasionally useful for taking up slack against the springs.  Use it
        sparingly and keep it small.
        """
        current = self.extensions()
        self.move_extensions([h + mm for h in current], wait=wait)

    def jog(self, axis: int, steps: int, wait: bool = True) -> None:
        """Move a single screw, in steps.

        Because the stage is over-constrained, one screw moving alone is a
        third preload and two thirds motion; it is the right tool for checking
        wiring and directions, not for positioning.
        """
        if not 0 <= axis <= 2:
            raise ValueError("axis must be 0, 1 or 2")
        self.limit_hits = []
        self.command(f"J {axis} {int(steps)}")
        if wait:
            self.wait()

    def jog_mm(self, axis: int, mm: float, wait: bool = True) -> None:
        """Move a single screw, in mm."""
        self.jog(axis, self.geometry.mm_to_steps(axis, mm), wait=wait)

    def home(self, wait: bool = True) -> None:
        """Return every screw to zero, and so the platform to centre (zero
        is wherever :meth:`zero` was last called; the limit switches only stop
        motion, they do not set it)."""
        self.move_steps(0, 0, 0, wait=wait)

    def stop(self) -> None:
        """Decelerate to a stop."""
        self.command("S")

    def halt(self) -> None:
        """Stop dead.  Steps may be lost, so re-zero afterwards."""
        self.command("X")

    # -------------------------------------------------------------- homing
    def _balanced(self, axis: int, mm: float) -> List[float]:
        """Relative screw moves that drive screw ``axis`` by ``mm`` and the
        other two by half as much the other way.  They sum to zero, so the
        platform moves along that leg and nothing goes into preload.

        There is no switch on the far side, so the other two are never taken
        more than ``home_away_max_mm`` away from their own switches from
        where this screw's homing started; past that the
        homing screw carries on alone."""
        g = self.geometry
        current = self.extensions()
        moves = [0.0, 0.0, 0.0]
        moves[axis] = mm
        for other in range(3):
            if other == axis:
                continue
            away = -g.home_direction[other]
            limit = self._home_start[other] + away * g.home_away_max_mm
            target = current[other] - mm / 2.0
            if away * (target - limit) > 0:
                target = limit
            moves[other] = target - current[other]
        return moves

    def _seek_switch(self, axis: int, mm: float, speed: float) -> int:
        """Drive screw ``axis`` up to ``mm`` towards its switch, balanced, and
        return its step position when the switch tripped."""
        g = self.geometry
        self.set_speed(speed)
        try:
            self.move_extensions(self._balanced(axis, g.home_direction[axis] * mm),
                                 relative=True)
        except DeltaBotError:
            if set(self.limit_hits) == {axis}:
                return self.positions()[axis]
            raise
        print(f"before raise switch {axis} did not trip within {mm:.2f} mm:", self.status()["sw"], self.limit_hits)
        raise DeltaBotError(f"switch {axis} did not trip within {mm:.2f} mm")

    def _back_off(self, axis: int, max_mm: float) -> float:
        """Move screw ``axis`` off its switch, balanced, in ``home_backoff_mm``
        steps until it releases (a switch releases later than it trips), then
        one step more for a run up.  Returns the distance backed off."""
        g = self.geometry
        self.set_speed(g.home_speed)
        step = g.home_backoff_mm
        backed = 0.0
        while True:
            self.move_extensions(self._balanced(axis, -g.home_direction[axis] * step),
                                 relative=True)
            backed += step
            time.sleep(0.05)                 # the firmware debounces the release
            if not self.limit_switches()[axis]:
                break
            if backed >= max_mm - 1e-9:
                raise DeltaBotError(
                    f"switch {axis} still reads tripped after backing off {backed:.2f} mm. "
                    f"With normally closed wiring that means its pin (D{9 + axis}) is not "
                    "reaching GND: check the switch is connected, COM goes to GND and "
                    "NC (not NO) to the pin, and the firmware is 1.1 or later. 'status' "
                    "shows all three switches")
        self.move_extensions(self._balanced(axis, -g.home_direction[axis] * step),
                             relative=True)
        return backed + step

    def _approach(self, axis: int, backed: float) -> int:
        """Having backed ``backed`` mm off a switch, return quickly to one
        ``home_backoff_mm`` short of where it tripped, then find it slowly.
        Returns the step position at which it tripped."""
        g = self.geometry
        step = g.home_backoff_mm
        self.set_speed(g.home_speed)
        try:
            self.move_extensions(self._balanced(axis, g.home_direction[axis] * (backed - step)),
                                 relative=True)
        except DeltaBotError:
            if set(self.limit_hits) != {axis}:
                raise
            # It tripped early; back off again and make the whole approach slowly.
            backed = self._back_off(axis, g.home_release_max_mm)
            return self._seek_switch(axis, backed + step, g.home_slow_speed)
        return self._seek_switch(axis, 10 * step, g.home_slow_speed)

    def _home_axis(self, axis: int) -> None:
        """Home one screw from roughly neutral: a balanced move down onto its
        switch, a slow second approach, then a balanced move back up by
        ``home_offset_mm``, which becomes that screw's zero."""
        g = self.geometry
        if self.limit_switches()[axis]:
            # Already on the switch: drive off it first, a step at a time,
            # stopping as soon as it releases.  If it has not released after
            # home_release_max_mm it is wiring, and _back_off says so.
            start = self.positions()
            self._home_start = self.extensions()
            try:
                self._back_off(axis, g.home_release_max_mm)
            except DeltaBotError:
                try:
                    self.set_speed(g.home_speed)
                    self.move_steps(*start)
                except DeltaBotError:
                    pass
                raise
        start = self.positions()
        self._home_start = self.extensions()
        try:
            self._seek_switch(axis, g.home_offset_mm + g.home_search_margin_mm,
                              g.home_speed)
            backed = self._back_off(axis, g.home_release_max_mm)
            self._approach(axis, backed)
        except DeltaBotError:
            try:                             # leave the platform where it started
                self.set_speed(g.home_speed)
                self.move_steps(*start)
            except DeltaBotError:
                pass
            raise
        print("before set speed:", self.status()["sw"], self.limit_hits)
        self.set_speed(g.home_speed)
        print("before final move:", self.status()["sw"], self.limit_hits)
        self.move_extensions(self._balanced(axis, -g.home_direction[axis] * g.home_offset_mm),
                             relative=True)
        pos = self.positions()
        pos[axis] = 0
        self.zero(pos)

    def find_home(self, axes: Sequence[int] = (0, 1, 2)) -> None:
        """Set zero from the limit switches, one screw at a time.

        Start with the stage roughly at neutral.  Each screw in ``axes`` is
        taken in turn: a balanced move (that screw by ``d``, the other two by
        ``-d/2``, so the platform slides along that leg and nothing goes into
        preload) drives its nut down onto its switch; it backs off and finds
        the switch again slowly; then a balanced move takes it back up by
        ``home_offset_mm`` and that point becomes the screw's zero.  At the
        end every homed screw is put on its zero; screws not in ``axes`` go
        back where they were and keep their current zero.  This also clears any preload
        and lost steps on the screws homed.

        Soft limits are off while searching, since the position is not yet
        trusted; speed and limits are put back afterwards, even on error.
        """
        axes = list(dict.fromkeys(int(a) for a in axes))
        if not axes or any(not 0 <= a <= 2 for a in axes):
            raise ValueError("axes must be drawn from 0, 1 and 2")
        before = self.status()
        self.command("B 0 0")
        try:
            for axis in axes:
                self._home_axis(axis)
            # Each screw's balanced moves nudge the others by half the
            # difference between its switch distance and home_offset_mm; put
            # every homed screw on its zero and the rest back where they were.
            target = list(before["pos"])
            for axis in axes:
                target[axis] = 0
            self.move_steps(*target)
        finally:
            self.set_speed(before["speed"])
            self.command("B %d %d" % tuple(before["limits"]))

    calibrate_home = find_home

    # ---------------------------------------------------------- parameters
    def zero(self, positions: Optional[Sequence[int]] = None) -> None:
        """Declare the current mechanical position to be the origin."""
        if positions is None:
            self.command("Z")
        else:
            self.command("Z %d %d %d" % tuple(int(p) for p in positions))

    def set_speed(self, steps_per_s: float) -> None:
        self.command(f"V {steps_per_s:.0f}")

    def set_accel(self, steps_per_s2: float) -> None:
        self.command(f"A {steps_per_s2:.0f}")

    def set_speed_mm_s(self, mm_per_s: float) -> None:
        """Set the speed as screw travel, mm/s.  The platform itself moves
        ``lateral_gain`` times as fast along the leg being driven."""
        self.set_speed(mm_per_s * self.geometry.steps_per_mm)

    def enable(self) -> None:
        self.command("E 1")

    def disable(self) -> None:
        """Cut current to the motors.  The screws hold position mechanically,
        but the platform can then be moved by hand."""
        self.command("E 0")

    def apply_soft_limits(self, travel_mm: Optional[float] = None) -> None:
        """Push a per screw travel limit, in mm either side of zero, into the
        firmware.  This is a backstop below the host's working radius: the
        firmware knows only about steps."""
        travel = self.geometry.travel_mm if travel_mm is None else travel_mm
        span = int(round(travel * self.geometry.steps_per_mm))
        self.command(f"B {-span} {span}")

    # ------------------------------------------------------------ patterns
    def circle(self, radius: float = 0.5, points: int = 36, turns: int = 1,
               wait: bool = True) -> None:
        """Walk the platform round a circle - a quick check that all three
        screws work and drive the right way."""
        from math import cos, pi, sin

        for i in range(points * turns + 1):
            angle = 2 * pi * i / points
            self.move_xy(radius * cos(angle), radius * sin(angle), wait=wait)
        self.move_xy(0.0, 0.0, wait=wait)

    def calibrate_gain(self, measured_mm: float, screw_mm: float = 1.0) -> float:
        """Work out ``lateral_gain`` from one measurement and store it.

        Extend one screw by ``screw_mm``, measure how far the platform actually
        moved in the plane (signed along that leg's direction), and pass it in.
        A single screw contributes two thirds of its travel to the motion, so
        the gain is ``measured / (2/3 * screw_mm)``.
        """
        if screw_mm == 0:
            raise ValueError("screw_mm must not be zero")
        gain = measured_mm / ((2.0 / 3.0) * screw_mm)
        self.geometry.lateral_gain = gain
        return gain
