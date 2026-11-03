"""Plant-side fault injectors for O5.

Faults, not attacks: a joint gets more friction, locks up, or the command path gets slower.
Each fault changes ONLY the simulated plant. The analysis side's Pinocchio model never sees
it — that gap is the signal the detector reads.

The fault types are the ones schema/telemetry_v0.yaml already allows (`friction`,
`stuck_joint`, `delay`); `backlash` is not implemented.

    FaultSpec("friction", joint=3, param=4.0, onset_s=3.0)   # Coulomb friction x4 on joint 3
    FaultSpec("stuck_joint", joint=3, param=200.0)           # friction (N*m) far above the drive
    FaultSpec("delay", joint=-1, param=25.0)                 # position targets arrive 25 ms late

Usage inside a stepping loop (Isaac Sim not imported here, so this stays unit-testable):

    inj = FaultInjector(art, base_friction, hz)
    ...each step:
    inj.update(t)                       # applies the fault once t >= onset
    target = inj.delay_target(target)   # pass the commanded target through the delay line
    ...at the end of the run:
    inj.clear()                         # restore the healthy plant
"""
from collections import deque
from dataclasses import dataclass

import numpy as np

ARM = 7
TYPES = ("none", "friction", "stuck_joint", "delay")


@dataclass
class FaultSpec:
    type: str = "none"
    joint: int = -1            # 0-6, or -1 for faults that are not per-joint (delay)
    param: float = np.nan      # friction: multiplier | stuck_joint: friction N*m | delay: ms
    onset_s: float = 3.0       # simulated seconds into the run

    def __post_init__(self):
        if self.type not in TYPES:
            raise ValueError(f"fault type {self.type!r} not in {TYPES}")
        if self.type in ("friction", "stuck_joint") and not 0 <= self.joint < ARM:
            raise ValueError(f"{self.type} needs joint 0-6, got {self.joint}")

    @property
    def tag(self):
        if self.type == "none":
            return "healthy"
        j = f"j{self.joint}" if self.joint >= 0 else "all"
        return f"{self.type}_{j}_{self.param:g}"


class FaultInjector:
    def __init__(self, art, base_friction, hz, spec=None):
        self.art = art
        self.base = np.asarray(base_friction, dtype=np.float64).copy()
        self.hz = hz
        self.idx = np.arange(ARM)
        self.set_spec(spec or FaultSpec())

    def set_spec(self, spec):
        self.clear()
        self.spec = spec
        self.active = False
        n = int(round(spec.param * 1e-3 * self.hz)) if spec.type == "delay" else 0
        self._line = deque()
        self._delay_steps = n

    def update(self, t):
        """Apply the fault the first time t reaches its onset. Returns whether it is active."""
        s = self.spec
        if s.type != "none" and not self.active and t >= s.onset_s:
            self.active = True
            if s.type == "friction":
                f = self.base.copy(); f[s.joint] = self.base[s.joint] * s.param
                self._write(f)
            elif s.type == "stuck_joint":
                f = self.base.copy(); f[s.joint] = s.param
                self._write(f)
            # delay acts in delay_target(); nothing to write to the plant
        return self.active

    def delay_target(self, target):
        """Pass a commanded target through the delay line (a no-op unless the fault is a
        delay and has started). The line is primed with the target in force at onset."""
        if self.spec.type != "delay" or not self.active or self._delay_steps == 0:
            self._last = target
            return target
        if not self._line:
            self._line.extend([self._last] * self._delay_steps)
        self._line.append(target)
        return self._line.popleft()

    def clear(self):
        """Restore the healthy plant."""
        if getattr(self, "active", False) or getattr(self, "spec", None) is not None:
            self._write(self.base)
        self.active = False
        self._line = deque()

    def _write(self, f):
        self.art.set_friction_coefficients(
            np.asarray(f, np.float32).reshape(1, ARM), joint_indices=self.idx)
