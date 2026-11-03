"""Independent Franka Panda dynamics model, built from the URDF with Pinocchio.

This is the "model torque" side of the residual:

    residual r = tau_measured - tau_model
    tau_model  = M(q) qdd + C(q, qd) qd + g(q) + tau_f(qd)

where tau_f is joint friction (see below), which Pinocchio's rnea does not include.

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
  - Friction is the Gaz et al. 2019 sigmoid (supplementary material, Table I), the same model
    and values as dev's pal_config.friction_sigmoid() and docs/SOURCES.md. It is smooth
    through zero velocity and exactly zero at rest, so it does not affect the gravity check.
    Each joint's friction can be scaled (friction_scale), which the synthetic plant uses for
    the deliberate 1.15x mismatch and for friction faults.
"""

from __future__ import annotations

from dataclasses import dataclass, field
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

# Gaz et al. 2019, supplementary material Table I: tau_f(qd) = phi1/(1+exp(-phi2(qd+phi3)))
# - phi1/(1+exp(-phi2 phi3)). Same values as dev's pal_config.FRICTION_SIGMOID_PHI1..3.
FRICTION_PHI1 = np.array([0.54615, 0.87224, 0.64068, 1.27940, 0.83904, 0.30301, 0.56489])        # N*m
FRICTION_PHI2 = np.array([5.1181, 9.0657, 10.136, 5.5903, 8.3469, 17.133, 10.336])               # s/rad
FRICTION_PHI3 = np.array([0.039533, 0.025882, -0.046070, 0.036194, 0.026226, -0.021047, 0.0035526])  # rad/s


def friction_sigmoid(qd) -> np.ndarray:
    """Joint friction torque (N*m) for a 7-vector of joint velocities. Zero at rest."""
    qd = np.asarray(qd, dtype=float)
    a, b, c = FRICTION_PHI1, FRICTION_PHI2, FRICTION_PHI3
    return a / (1.0 + np.exp(-b * (qd + c))) - a / (1.0 + np.exp(-b * c))


@dataclass
class PandaModel:
    model: pin.Model
    data: pin.Data
    ee_frame: str
    friction_scale: np.ndarray = field(default_factory=lambda: np.ones(7))

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

    def friction(self, qd) -> np.ndarray:
        """Joint friction torque tau_f(qd), scaled per joint by friction_scale."""
        return self.friction_scale * friction_sigmoid(self._q(qd))

    def tau_model(self, q, qd, qdd, include_friction: bool = True) -> np.ndarray:
        """M(q) qdd + C(q, qd) qd + g(q) (+ tau_f(qd)), with armature."""
        q, qd, qdd = self._q(q), self._q(qd), self._q(qdd)
        tau = pin.rnea(self.model, self.data, q, qd, qdd).copy()
        return tau + self.friction(qd) if include_friction else tau

    def forward_dynamics(self, q, qd, tau) -> np.ndarray:
        """Joint acceleration produced by joint torque tau, friction included."""
        q, qd, tau = self._q(q), self._q(qd), self._q(tau)
        return pin.aba(self.model, self.data, q, qd, tau - self.friction(qd)).copy()

    def jacobian(self, q, frame: str | None = None) -> np.ndarray:
        """6x7 geometric Jacobian in the world-aligned frame. Rows 0-2 linear (m/s per rad/s),
        rows 3-5 angular (rad/s per rad/s). The two blocks have different units, which matters
        for any conditioning score built from singular values."""
        fid = self.model.getFrameId(frame or self.ee_frame)
        J = pin.computeFrameJacobian(
            self.model, self.data, self._q(q), fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        return np.array(J)

    def ee_pose(self, q, frame: str | None = None) -> tuple[np.ndarray, np.ndarray]:
        """World position (m) and orientation as a unit quaternion (w, x, y, z)."""
        fid = self.model.getFrameId(frame or self.ee_frame)
        pin.forwardKinematics(self.model, self.data, self._q(q))
        pin.updateFramePlacements(self.model, self.data)
        oMf = self.data.oMf[fid]
        quat = pin.Quaternion(oMf.rotation)
        return oMf.translation.copy(), np.array([quat.w, quat.x, quat.y, quat.z])

    def ee_position(self, q, frame: str | None = None) -> np.ndarray:
        return self.ee_pose(q, frame)[0]

    def point_acceleration(self, q, qd, qdd, offset_local=(0.0, 0.0, 0.0),
                           frame: str | None = None) -> np.ndarray:
        """World linear acceleration (m/s^2) of a point fixed in `frame` at `offset_local` (m),
        e.g. a payload held at the TCP. Gravity is not included."""
        fid = self.model.getFrameId(frame or self.ee_frame)
        pin.forwardKinematics(self.model, self.data, self._q(q), self._q(qd), self._q(qdd))
        pin.updateFramePlacements(self.model, self.data)
        R = self.data.oMf[fid].rotation
        p = R @ np.asarray(offset_local, dtype=float)
        lwa = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        v = pin.getFrameVelocity(self.model, self.data, fid, lwa)
        a = pin.getFrameClassicalAcceleration(self.model, self.data, fid, lwa)
        w = v.angular
        return a.linear + np.cross(a.angular, p) + np.cross(w, np.cross(w, p))

    def potential_energy(self, q) -> float:
        return float(pin.computePotentialEnergy(self.model, self.data, self._q(q)))


def load_panda(
    urdf: str | Path = DEFAULT_URDF,
    armature: float = ARMATURE_DEFAULT,
    finger_open: float = FINGER_OPEN,
    gravity: tuple[float, float, float] = GRAVITY,
    ee_frame: str = "panda_hand",
    friction: bool = True,
) -> PandaModel:
    """Load the URDF as a 7-DOF arm with the fingers locked at `finger_open`.
    friction=False gives a rigid-body-only model (friction_scale all zero)."""
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

    return PandaModel(
        model=model,
        data=model.createData(),
        ee_frame=ee_frame,
        friction_scale=np.ones(model.nv) if friction else np.zeros(model.nv),
    )
