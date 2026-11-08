"""Plant-side fault injectors for O5.

Faults, not attacks: a joint gets more friction, locks up, develops backlash, or the command
path gets slower. Each fault changes ONLY the simulated plant. The analysis side's Pinocchio
model never sees it: that gap is the signal the detector reads.

The fault types are the ones schema/telemetry_v1.yaml allows. All are applied through the
torque command path (effort mode), because friction, backlash and a locked joint only show up
in the APPLIED torque (docs/REVIEW.md F2).

    FaultSpec("friction", joint=3, param=4.0, onset_s=3.0)    # friction effort x4 on joint 3
    FaultSpec("stuck_joint", joint=3, param=200.0)            # friction effort 200 N*m, far above the drive
    FaultSpec("delay", joint=0, param=0.025)                  # torque commands arrive 25 ms late
    FaultSpec("backlash", joint=3, param=0.02)                # 0.02 rad of play between motor and link

Usage in a stepping loop (Isaac not imported here, so this is unit-testable):

    inj = FaultInjector(plant, hz)
    inj.set_spec(spec)
    ...each step:
    inj.update(t)                          # applies the fault once t >= onset
    tau = inj.command(tau, qdd_last)       # delay line, then backlash
    ...end of run:  inj.clear()
"""
from collections import deque
from dataclasses import dataclass

import numpy as np

import pal_config as cfg

ARM = cfg.DOF
TYPES = ("none", "friction", "backlash", "stuck_joint", "delay")


@dataclass
class FaultSpec:
    type: str = "none"
    joint: int = -1            # 0-6; -1 only for type none. The schema requires a joint for every
                               # fault, so delay carries a nominal 0 although it hits every joint.
    param: float = np.nan      # friction: multiplier | stuck_joint: friction N*m | delay: seconds
                               # | backlash: total play in rad
    onset_s: float = 3.0       # simulated seconds into the run

    def __post_init__(self):
        if self.type not in TYPES:
            raise ValueError(f"fault type {self.type!r} not in {TYPES}")
        if (self.type == "none") != (self.joint == -1):
            raise ValueError("joint must be -1 exactly when type is none (schema rule)")
        if self.type != "none" and not 0 <= self.joint < ARM:
            raise ValueError(f"{self.type} needs joint 0-{ARM - 1}, got {self.joint}")

    @property
    def tag(self):
        if self.type == "none":
            return "healthy"
        j = "all" if self.type == "delay" else f"j{self.joint}"
        p = f"{self.param * 1e3:g}ms" if self.type == "delay" else f"{self.param:g}"
        return f"{self.type}_{j}_{p}"


class FaultInjector:
    """`plant` needs .base_friction (7,3) and .set_friction((7,3)); see pal_plant.Plant."""

    def __init__(self, plant, hz, motor_inertia=None):
        self.plant, self.hz = plant, hz
        self.dt = 1.0 / hz
        # reflected rotor inertia per joint, = the plant's armature. It used to be a single 0.0061 (the Panda's),
        # which made backlash on the UR10e's 0.02-0.1 kg*m^2 rotors move several times too freely.
        self.jm = np.broadcast_to(np.asarray(cfg.ARMATURE if motor_inertia is None else motor_inertia, float), (ARM,)).copy()
        self.spec = FaultSpec()
        self.active = False
        self._line = deque()
        self._delay_steps = 0
        self._last = np.zeros(ARM)

    def set_spec(self, spec):
        self.clear()
        self.spec = spec
        self._delay_steps = int(round(spec.param * self.hz)) if spec.type == "delay" else 0
        # backlash state: gap position x = theta_motor - q (rad), relative velocity v (rad/s)
        self._bx, self._bv, self._contact = 0.0, 0.0, 0

    def update(self, t):
        """Apply the fault the first time t reaches its onset. Returns whether it is active."""
        s = self.spec
        if s.type != "none" and not self.active and t >= s.onset_s:
            self.active = True
            f = self.plant.base_friction.copy()
            if s.type == "friction":
                f[s.joint, :] *= s.param
                self.plant.set_friction(f)
            elif s.type == "stuck_joint":
                f[s.joint, 0] = f[s.joint, 1] = s.param
                self.plant.set_friction(f)
            # delay and backlash act in command()
        return self.active

    def command(self, tau, qdd_last):
        """Pass the controller's torque through the faulty actuation path."""
        tau = np.asarray(tau, dtype=float).copy()
        s = self.spec
        if not self.active or s.type not in ("delay", "backlash"):
            self._last = tau
            return tau
        if s.type == "delay" and self._delay_steps > 0:
            if not self._line:
                self._line.extend([self._last.copy()] * self._delay_steps)
            self._line.append(tau)
            return self._line.popleft()
        if s.type == "backlash":
            j, half = s.joint, 0.5 * s.param
            if self._contact == 0:
                # in the gap: the motor side moves freely, nothing reaches the link
                self._bv += (tau[j] / self.jm[j] - qdd_last[j]) * self.dt
                self._bx += self._bv * self.dt
                if abs(self._bx) >= half:
                    self._bx = np.sign(self._bx) * half
                    self._bv = 0.0
                    self._contact = int(np.sign(self._bx))
                tau[j] = 0.0 if self._contact == 0 else tau[j]
            else:
                # engaged: torque reaches the link until it reverses and opens the gap
                if np.sign(tau[j]) == -self._contact and tau[j] != 0.0:
                    self._contact = 0
                    self._bx, self._bv = self._contact * half, 0.0
                    tau[j] = 0.0
        return tau

    def clear(self):
        """Restore the healthy plant."""
        if self.spec.type in ("friction", "stuck_joint") and self.active:
            self.plant.set_friction(self.plant.base_friction)
        self.active = False
        self._line = deque()
