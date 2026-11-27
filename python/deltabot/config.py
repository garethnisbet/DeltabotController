"""Machine configuration for the 3d printed DeltaBot.

Everything the host needs to convert between steps, millimetres and a platform
position lives here.  The numbers below match the kit as described in
Instructions.pdf, except for ``lateral_gain``, which depends on how the flexure
legs are built and has to be measured once on your own machine.
"""

from dataclasses import dataclass
from typing import Tuple


@dataclass
class Geometry:
    """Physical description of the three screw jacks."""

    # 28BYJ-48 rewired as a bipolar motor: 64 steps/rev on the rotor through
    # a 1/64 gearbox gives 2048 full steps per output revolution.
    steps_per_rev: int = 2048

    # Microstepping set by the jumpers under the GRBL shield's drivers.
    # No jumpers fitted = full step = 1.
    microsteps: int = 1

    # An M3 coarse thread advances 0.5 mm per revolution.
    screw_pitch_mm: float = 0.5

    # Where each screw sits, and so the in-plane direction it drives the
    # platform along, measured counter-clockwise from the +X axis.
    # Axis 0 is the motor on the shield's X driver.
    screw_angles_deg: Tuple[float, float, float] = (90.0, 210.0, 330.0)

    # +1 if a positive step count extends that screw (drives its leg), -1 if
    # the motor or the driver wiring runs the other way.
    direction: Tuple[int, int, int] = (1, 1, 1)

    # CALIBRATE THIS.  Millimetres the platform moves in the plane per
    # millimetre a screw is extended, and which way: positive means extending
    # a screw pushes the platform along that leg's direction (away from that
    # screw's corner), negative means it pulls the platform towards it.
    # See "Calibrating the gain" in the README - it is a two minute job with a
    # ruler or a dial gauge, and everything in mm depends on it.
    lateral_gain: float = 0.8

    # Effective leg length used for the second order Z estimate.  Only affects
    # the reported Z, which is an estimate: Z is not a controlled axis.
    leg_length_mm: float = 40.0

    # How much the platform lifts per mm of common mode screw extension.  The
    # common mode is the over-constrained direction, so this is small and
    # mostly preload; leave it at zero unless you have measured it.
    common_mode_gain: float = 0.0

    # Working radius in the plane, mm.  Commands outside it are refused.
    max_radius_mm: float = 3.0

    # Travel of each screw, in millimetres, either side of the zero you set.
    # Used to build the firmware's soft limits.
    travel_mm: float = 4.0

    # Homing on the limit switches (see DeltaBot.find_home).  +1 if a screw
    # reaches its switch by extending, -1 by retracting.  On this machine
    # the firmware's DIR_INVERT makes positive steps drive the nut down onto
    # the switch, so +1 (with direction = +1).
    home_direction: Tuple[int, int, int] = (1, 1, 1)

    # The switches trip about 5 mm below neutral.  Homing finds a switch,
    # then comes back up this far, and that point is the screw's zero.
    home_offset_mm: float = 5.0

    # Homing starts from roughly neutral, so each screw searches down
    # home_offset_mm plus this much before giving up.
    home_search_margin_mm: float = 1.5

    # While one screw homes, the balanced move turns the other two away from
    # their switches, where nothing would stop them.  They go at most this
    # far from where homing started, enough for a fully balanced search;
    # past that the homing screw would carry on alone.
    home_away_max_mm: float = 3.5

    # Back off the switch in steps of this much until it releases, then one
    # step more; go back quickly to this far short of where it tripped, and
    # cover the rest slowly.  Give up if it has not released after
    # home_release_max_mm.
    home_backoff_mm: float = 0.2
    home_release_max_mm: float = 2.0
    home_speed: float = 300.0        # steps/s, search, back off and return
    home_slow_speed: float = 120.0   # steps/s, the final approach

    # Motion defaults.  The 28BYJ-48 loses steps well before 1000 steps/s.
    default_speed: float = 400.0
    default_accel: float = 2000.0

    @property
    def steps_per_mm(self) -> float:
        return self.steps_per_rev * self.microsteps / self.screw_pitch_mm

    def steps_to_mm(self, axis: int, steps: float) -> float:
        return self.direction[axis] * steps / self.steps_per_mm

    def mm_to_steps(self, axis: int, mm: float) -> int:
        return int(round(self.direction[axis] * mm * self.steps_per_mm))


@dataclass
class Connection:
    port: str = "auto"
    baud: int = 115200
    timeout: float = 1.0
    # The Nano resets when the port is opened; give the bootloader time.
    reset_delay: float = 2.0
