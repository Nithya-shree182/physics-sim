"""Independent Franka Panda dynamics model, built from the URDF with Pinocchio.

This is the "model torque" side of the residual:

    residual r = tau_model - tau_measured
    tau_model  = M(q) qdd + C(q, qd) qd + g(q)

It must stay independent of the simulator: nothing here reads Isaac Sim. The only inputs are
the URDF and the handful of settings below, each with its source.

Settings that change M(q) or g(q), and where they come from:
  - Fingers are locked at `finger_open` (0.04 m each, fully open), so the model is the 7-DOF
    arm with the hand and both fingers as rigid tool mass (0.93 kg total).
  - Armature (reflected rotor inertia between motor and joint) 0.0061 kg*m^2 per arm joint,
    the value dev's pal_config.py applies to the simulated plant. A URDF has none, so it is
    set on the model as model.armature, and Pinocchio adds it to the diagonal of M and as
    armature * qdd in rnea. gravity_check.py verifies that the installed Pinocchio does this.
  - Gravity is (0, 0, -9.81), set explicitly.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pinocchio as pin

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = REPO_ROOT / "panda_arm_hand.urdf"

ARM_JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
FINGER_JOINTS = ["panda_finger_joint1", "panda_finger_joint2"]

ARMATURE_DEFAULT = 0.0061      # kg*m^2 per arm joint, matches dev's pal_config.ARMATURE_PHYSICAL
FINGER_OPEN = 0.04             # m per finger, fully open
GRAVITY = (0.0, 0.0, -9.81)


@dataclass
class PandaModel:
    model: pin.Model
    data: pin.Data
    ee_frame: str

    @property
    def armature(self) -> np.ndarray:
        return np.array(self.model.armature)

    @property
    def nq(self) -> int:
        return self.model.nq

    @property
    def joint_names(self) -> list[str]:
        # joint 0 in a Pinocchio model is the universe
        return list(self.model.names)[1:]

    @property
    def effort_limit(self) -> np.ndarray:
        return np.array(self.model.effortLimit)

    @property
    def total_mass(self) -> float:
        return float(pin.computeTotalMass(self.model))

    def _q(self, q) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        if q.shape != (self.nq,):
            raise ValueError(f"q must have shape ({self.nq},), got {q.shape}")
        return q

    def g(self, q) -> np.ndarray:
        """Gravity torque: what the joints must supply to hold the arm still at q (N*m)."""
        return pin.computeGeneralizedGravity(self.model, self.data, self._q(q)).copy()

    def M(self, q) -> np.ndarray:
        """Mass matrix including armature, symmetric."""
        pin.crba(self.model, self.data, self._q(q))
        upper = np.triu(self.data.M)
        return upper + np.triu(upper, 1).T

    def coriolis(self, q, qd) -> np.ndarray:
        """C(q, qd) qd, the velocity-coupling torque, gravity excluded."""
        q, qd = self._q(q), self._q(qd)
        full = pin.rnea(self.model, self.data, q, qd, np.zeros(self.nq))
        return full - self.g(q)

    def tau_model(self, q, qd, qdd) -> np.ndarray:
        """M(q) qdd + C(q, qd) qd + g(q), with armature."""
        q, qd, qdd = self._q(q), self._q(qd), self._q(qdd)
        return pin.rnea(self.model, self.data, q, qd, qdd).copy()

    def jacobian(self, q, frame: str | None = None) -> np.ndarray:
        """6x7 geometric Jacobian in the world-aligned frame. Rows 0-2 linear (m/s per rad/s),
        rows 3-5 angular (rad/s per rad/s). The two blocks have different units, which matters
        for any conditioning score built from singular values."""
        fid = self.model.getFrameId(frame or self.ee_frame)
        J = pin.computeFrameJacobian(
            self.model, self.data, self._q(q), fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        return np.array(J)

    def ee_position(self, q, frame: str | None = None) -> np.ndarray:
        fid = self.model.getFrameId(frame or self.ee_frame)
        pin.forwardKinematics(self.model, self.data, self._q(q))
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[fid].translation.copy()

    def potential_energy(self, q) -> float:
        return float(pin.computePotentialEnergy(self.model, self.data, self._q(q)))


def load_panda(
    urdf: str | Path = DEFAULT_URDF,
    armature: float = ARMATURE_DEFAULT,
    finger_open: float = FINGER_OPEN,
    gravity: tuple[float, float, float] = GRAVITY,
    ee_frame: str = "panda_hand",
) -> PandaModel:
    """Load the URDF as a 7-DOF arm with the fingers locked at `finger_open`."""
    urdf = Path(urdf)
    if not urdf.is_file():
        raise FileNotFoundError(f"URDF not found: {urdf}")

    full = pin.buildModelFromUrdf(str(urdf))

    # Lock the fingers at a reference configuration so their mass stays on the hand.
    q_ref = pin.neutral(full)
    lock_ids = []
    for name in FINGER_JOINTS:
        if full.existJointName(name):
            jid = full.getJointId(name)
            lock_ids.append(jid)
            q_ref[full.joints[jid].idx_q] = finger_open
    model = pin.buildReducedModel(full, lock_ids, q_ref) if lock_ids else full

    model.gravity.linear = np.array(gravity, dtype=float)
    model.armature = np.full(model.nv, float(armature))

    if model.nq != 7:
        raise RuntimeError(f"expected a 7-DOF arm after locking fingers, got nq={model.nq}")
    names = list(model.names)[1:]
    if names != ARM_JOINTS:
        raise RuntimeError(f"unexpected joint order: {names}")
    if not model.existFrame(ee_frame):
        raise RuntimeError(f"end-effector frame '{ee_frame}' not in the model")

    return PandaModel(model=model, data=model.createData(), ee_frame=ee_frame)
