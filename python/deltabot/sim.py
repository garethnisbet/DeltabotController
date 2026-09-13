"""A tiny stand-in for the firmware, so the host side can be exercised with
no hardware attached.  It speaks the same line protocol over an object that
looks enough like a ``serial.Serial`` for :class:`~deltabot.controller.DeltaBot`.

Motion is instantaneous: a move completes as soon as it is issued.
"""

import time


class SimSerial:
    def __init__(self, speed=600.0, accel=2000.0):
        self.is_open = True
        self.pos = [0, 0, 0]
        self.tgt = [0, 0, 0]
        self.enabled = True
        self.speed = speed
        self.accel = accel
        self.limits = [-400000, 400000]
        self._out = ["DeltaBot 1.0 (simulated) ready"]
        self._in = ""

    # -- serial.Serial surface -------------------------------------------
    def write(self, data):
        self._in += data.decode()
        while "\n" in self._in:
            line, self._in = self._in.split("\n", 1)
            self._handle(line.strip())
        return len(data)

    def readline(self):
        if not self._out:
            return b""
        return (self._out.pop(0) + "\n").encode()

    @property
    def in_waiting(self):
        return sum(len(s) + 1 for s in self._out)

    def reset_input_buffer(self):
        self._out.clear()

    def close(self):
        self.is_open = False

    # -- protocol ---------------------------------------------------------
    def _handle(self, line):
        if not line:
            return
        cmd, args = line[0].upper(), line[1:].replace(",", " ").split()
        try:
            nums = [int(float(a)) for a in args]
        except ValueError:
            self._out.append("err bad number")
            return

        if cmd in "MR":
            if len(nums) != 3:
                return self._out.append("err need 3 values")
            tgt = nums if cmd == "M" else [p + d for p, d in zip(self.pos, nums)]
            if any(not self._ok(t) for t in tgt):
                return self._out.append("err soft limit")
            self._move(tgt)
        elif cmd == "J":
            if len(nums) != 2 or not 0 <= nums[0] <= 2:
                return self._out.append("err need axis and steps")
            tgt = list(self.pos)
            tgt[nums[0]] += nums[1]
            if not self._ok(tgt[nums[0]]):
                return self._out.append("err soft limit")
            self._move(tgt)
        elif cmd == "P":
            self._out.append("pos %d %d %d" % tuple(self.pos))
            self._out.append("ok")
        elif cmd == "?":
            self._out.append(
                "status idle pos %d %d %d tgt %d %d %d en %d speed %.2f accel %.2f limits %d %d"
                % (*self.pos, *self.tgt, int(self.enabled), self.speed, self.accel, *self.limits)
            )
            self._out.append("ok")
        elif cmd == "V":
            self.speed = float(args[0]); self._out.append("ok")
        elif cmd == "A":
            self.accel = float(args[0]); self._out.append("ok")
        elif cmd == "E":
            self.enabled = bool(nums[0]); self._out.append("ok")
        elif cmd == "Z":
            self.pos = list(nums) if len(nums) == 3 else [0, 0, 0]
            self.tgt = list(self.pos); self._out.append("ok")
        elif cmd == "B":
            self.limits = nums[:2]; self._out.append("ok")
        elif cmd in "SX":
            self.tgt = list(self.pos); self._out.append("ok")
        elif cmd == "$":
            self._out.append("simulated firmware"); self._out.append("ok")
        else:
            self._out.append("err unknown command")

    def _ok(self, target):
        lo, hi = self.limits
        return lo == hi or lo <= target <= hi

    def _move(self, tgt):
        moved = tgt != self.pos
        self.pos = list(tgt)
        self.tgt = list(tgt)
        self._out.append("ok")
        if moved:
            self._out.append("done")
