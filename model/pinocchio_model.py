"""The independent dynamics model: the arm's URDF read by Pinocchio, and nothing else.

This is the "model torque" side of the residual:

    residual r = tau_measured - tau_model
    tau_model  = M(q) qdd + C(q, qd) qd + g(q) + tau_f(qd)

where tau_f is joint friction, which Pinocchio's rnea does not include.

It must stay independent of the simulator: nothing here reads Isaac Sim. The only inputs are
the URDF and the robot spec (`workspace/pal_robot.py`). The plant derives from the SAME URDF
with a deliberate, documented mismatch (5% on driven-link mass, 15% on friction, and a
different friction law), so a residual is a measurement of that mismatch and of any injected
fault, never of two unrelated models disagreeing.

What enters M(q) and g(q), and where it comes from:

  - Every joint that is not one of the arm's own is locked at a reference configuration
    rather than deleted, so the mass it carries stays on its parent link. The UR10e's URDF
    has none; the hook stays for an arm that does.
  - Armature (reflected rotor inertia between motor and joint) comes from the robot spec. A
    URDF cannot express it, so it is set as model.armature, and Pinocchio adds it to the
    diagonal of M and as armature*qdd in rnea. model/gravity_check.py verifies that the
    installed Pinocchio actually does this, because a version that ignored it would make the
    model quietly lighter than the plant.
  - Gravity is set explicitly to (0, 0, -9.81) rather than left to a default.
  - Friction is a sigmoid whose parameters live on the robot spec. It is smooth through zero
    velocity and exactly zero at rest, so it does not disturb a static gravity check. Each
    joint can be scaled (friction_scale) for a friction fault or for the plant-vs-model
    mismatch.

A HELD BOX IS PART OF THE MODEL, NOT A CORRECTION TO IT. `with_payload()` returns a model
with the carried object's real rigid-body inertia appended to the tool frame: a box's
m/12*diag(y^2+z^2, ...) tensor at its own centre of mass, half a box beyond the suction cups.
That is not the same as adding m*g to the wrist, and the difference is the whole reason the
payload shows up in C(q,qd)qd and M(q)qdd as well as in g(q). The point-mass form is still
available (`point_mass=True`) because the payload ESTIMATOR models an unknown load that way —
it cannot know a shape it has not measured.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "workspace"))
import pal_robot  # noqa: E402

_SPEC = pal_robot.active()
DEFAULT_URDF = Path(_SPEC.urdf_file())
ARM_JOINTS = list(_SPEC.joint_names)

ARMATURE_DEFAULT = _SPEC.armature    # kg*m^2 per joint, reflected rotor inertia (pal_robot)
LOCK_AT = 0.0                  # reference value any non-arm joint is locked at
GRAVITY = (0.0, 0.0, -9.81)

# tau_f(qd) = phi1/(1+exp(-phi2(qd+phi3))) - phi1/(1+exp(-phi2 phi3)). The parameters live on the
# robot spec (pal_robot), because they are a property of the arm; these names are the active one's.
FRICTION_PHI1 = _SPEC.friction_phi1       # N*m
FRICTION_PHI2 = _SPEC.friction_phi2       # s/rad
FRICTION_PHI3 = _SPEC.friction_phi3       # rad/s


def friction_sigmoid(qd, spec=None) -> np.ndarray:
    """Joint friction torque (N*m) for a vector of joint velocities. Zero at rest.

    The parameters live on the robot spec, because they are a property of the arm. With no
    spec the active one is used; a `qd` whose length does not match it is an error rather
    than something to guess at."""
    qd = np.asarray(qd, dtype=float)
    spec = spec or pal_robot.active()
    if qd.shape[-1] != spec.dof:
        raise ValueError(f"qd has {qd.shape[-1]} joints, {spec.name} has {spec.dof}")
    a, b, c = spec.friction_phi1, spec.friction_phi2, spec.friction_phi3
    return a / (1.0 + np.exp(-b * (qd + c))) - a / (1.0 + np.exp(-b * c))


@dataclass
class RobotModel:
    model: pin.Model
    data: pin.Data
    ee_frame: str
    friction_scale: np.ndarray = field(default_factory=lambda: np.ones(pal_robot.active().dof))
    spec: object = None            # the pal_robot.RobotSpec this was built from
    payload_kg: float = 0.0        # a carried object folded into the model by with_payload()
    payload_offset: tuple = (0.0, 0.0, 0.0)
    payload_size: tuple | None = None

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
        return self.friction_scale * friction_sigmoid(self._q(qd), self.spec)

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
        """The 6xn geometric Jacobian in the world-aligned frame. Rows 0-2 linear (m/s per rad/s),
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

    # ── energy and power ────────────────────────────────────────────────────────────────
    def potential_energy(self, q) -> float:
        """Gravitational potential energy (J), absolute: measured from the world origin with
        g = -9.81 z. Only differences between poses mean anything physically, but having the
        absolute value makes the two sides comparable, which the in-sim log cannot do (it only
        has dV/dt = g'qd to integrate)."""
        return float(pin.computePotentialEnergy(self.model, self.data, self._q(q)))

    def kinetic_energy(self, q, qd) -> float:
        """0.5 qd' M(q) qd (J), armature included.

        Computed through M rather than pin.computeKineticEnergy. The two agree exactly on the
        installed Pinocchio (4.1: both include model.armature, checked to 1e-15 in
        tests/test_energy.py), but going through M is the version that cannot silently drop the
        rotors' share — about 1.8% of the energy at 1 rad/s on this arm — if a future Pinocchio
        changes what its energy helper accounts for. The test pins both facts, so a change in
        that behaviour fails loudly instead of making the twin quietly lighter than the plant."""
        q, qd = self._q(q), self._q(qd)
        return 0.5 * float(qd @ self.M(q) @ qd)

    def total_energy(self, q, qd) -> float:
        """Kinetic plus potential (J). Conserved on a frictionless, torque-free swing, which
        makes it the cheapest check that an integrator or a mass matrix is right."""
        return self.kinetic_energy(q, qd) + self.potential_energy(q)

    def joint_power(self, qd, tau) -> np.ndarray:
        """Mechanical power per joint, tau * qd (W). Positive = the motor is doing work on the
        arm; negative = the arm is driving the motor and the joint is braking."""
        return self._q(qd) * self._q(tau)

    def gravity_power(self, q, qd) -> float:
        """g(q)' qd (W) = dV/dt: how fast potential energy is being gained. The identity is
        what lets the in-sim log carry a relative potential without link centres of mass."""
        return float(self.g(q) @ self._q(qd))

    def decompose(self, q, qd, qdd) -> dict:
        """The model torque split into the four terms that make it up, each in N*m.

        gravity + inertial + coriolis + friction = tau_model. This is what turns the residual
        from one number into an explanation: a friction fault moves the friction term, a
        payload moves gravity and inertial together, a stuck joint moves none of them and
        shows up entirely in what is left over.
        """
        q, qd, qdd = self._q(q), self._q(qd), self._q(qdd)
        g = self.g(q)
        inertial = self.M(q) @ qdd
        # rnea(q, qd, qdd) - rnea(q, 0, qdd) isolates the velocity-dependent part exactly,
        # including the armature's contribution, instead of forming C and multiplying.
        full = pin.rnea(self.model, self.data, q, qd, qdd).copy()
        coriolis = full - g - inertial
        friction = self.friction(qd)
        return {"gravity": g, "inertial": inertial, "coriolis": coriolis,
                "friction": friction, "total": full + friction}

    # ── pose quality ────────────────────────────────────────────────────────────────────
    def operational_space_inertia(self, q, frame: str | None = None) -> np.ndarray:
        """The 3x3 apparent mass (kg) at the tool, (J_v M^-1 J_v')^-1.

        How heavy the tool feels to a push from each direction. A payload estimator pushed
        near a singularity is reading a direction the arm is nearly infinitely stiff in, which
        is why this and sigma_min belong in the same conversation as the residual."""
        Jv = self.jacobian(q, frame)[:3]
        Minv = np.linalg.inv(self.M(self._q(q)))
        return np.linalg.inv(Jv @ Minv @ Jv.T)

    def manipulability(self, q, frame: str | None = None) -> float:
        """Yoshikawa's measure sqrt(det(J_v J_v')) (m^3/rad^3), on the LINEAR block only.

        The full 6xn Jacobian mixes m/s rows with rad/s rows, so its determinant has no single
        unit and its condition number is not a physical quantity. Taking the SVD of the whole
        thing is the obvious wrong move; this does not."""
        Jv = self.jacobian(q, frame)[:3]
        return float(np.sqrt(max(np.linalg.det(Jv @ Jv.T), 0.0)))

    def singular_values(self, q, frame: str | None = None) -> np.ndarray:
        """Singular values of the Jacobian's linear block (m/rad), largest first."""
        return np.linalg.svd(self.jacobian(q, frame)[:3], compute_uv=False)

    def gravity_headroom(self, q) -> np.ndarray:
        """|g(q)| as a fraction of each joint's effort limit: how much of the arm's strength
        is spent just holding the pose."""
        return np.abs(self.g(q)) / self.spec.tau_max

    # ── a carried object ────────────────────────────────────────────────────────────────
    def with_payload(self, mass_kg: float, size=None, offset=None, *, point_mass=False,
                     frame: str | None = None) -> "RobotModel":
        """A copy of this model carrying `mass_kg` rigidly attached to the tool frame.

        `size` is the box's (x, y, z) extent in metres; with none given the payload is a point
        mass, which is also what `point_mass=True` forces. `offset` is where its centre of mass
        sits in the tool frame, defaulting to the spec's tool_offset — for the UR10e that is
        0.195 m down the flange's z, which is the cup plane plus half a box.

        The returned model is independent: this one is not modified, so a caller can hold a
        loaded and an unloaded model side by side, which is exactly what a payload test needs.
        """
        if mass_kg < 0:
            raise ValueError(f"payload mass must not be negative, got {mass_kg}")
        model = self.model.copy()
        fid = model.getFrameId(frame or self.ee_frame)
        f = model.frames[fid]
        off = np.asarray(self.spec.tool_offset if offset is None else offset, dtype=float)
        if mass_kg > 0:
            if point_mass or size is None:
                I = np.zeros((3, 3))
            else:
                x, y, z = (float(v) for v in size)
                I = mass_kg / 12.0 * np.diag([y * y + z * z, x * x + z * z, x * x + y * y])
            # Inertia(mass, lever, tensor): the tensor is about the body's own centre of mass,
            # the lever is zero because the placement already puts the frame there.
            body = pin.Inertia(float(mass_kg), np.zeros(3), I)
            placement = f.placement * pin.SE3(np.eye(3), off)
            model.appendBodyToJoint(f.parentJoint, body, placement)
        out = RobotModel(model=model, data=model.createData(), ee_frame=self.ee_frame,
                         friction_scale=self.friction_scale.copy(), spec=self.spec)
        out.payload_kg = float(mass_kg)
        out.payload_offset = tuple(off)
        out.payload_size = None if (point_mass or size is None) else tuple(float(v) for v in size)
        return out



def load_robot(
    spec=None,
    urdf: str | Path | None = None,
    armature=None,
    lock_at: float = LOCK_AT,
    gravity: tuple[float, float, float] = GRAVITY,
    ee_frame: str | None = None,
    friction: bool = True,
) -> RobotModel:
    """Load an arm's URDF as a fixed-base rigid-body model.

    `spec` is a pal_robot.RobotSpec and supplies the URDF, the expected joint names and order, the
    armature and the tool frame; leaving it out takes the active robot.

    Any extra joints the URDF carries that are not the arm's own are locked at `lock_at` rather
    than deleted, so the mass they carry stays on their parent link. The UR10e's URDF has none.
    friction=False gives a rigid-body-only model (friction_scale all zero)."""
    spec = spec or pal_robot.active()
    urdf = Path(urdf) if urdf is not None else Path(spec.urdf_file())
    ee_frame = ee_frame or spec.tool_frame
    armature = spec.armature if armature is None else armature
    if not urdf.is_file():
        raise FileNotFoundError(f"URDF not found: {urdf}")

    full = pin.buildModelFromUrdf(str(urdf))

    # Lock every joint that is not one of the arm's, at a reference configuration, so the mass it
    # carries stays on its parent link instead of leaving the model.
    arm = list(spec.joint_names)
    q_ref = pin.neutral(full)
    lock_ids = []
    for jid in range(1, full.njoints):
        name = full.names[jid]
        if name in arm:
            continue
        lock_ids.append(jid)
        q_ref[full.joints[jid].idx_q] = lock_at
    model = pin.buildReducedModel(full, lock_ids, q_ref) if lock_ids else full

    model.gravity.linear = np.array(gravity, dtype=float)
    arm_armature = np.asarray(armature, dtype=float)
    model.armature = (np.full(model.nv, float(armature)) if arm_armature.ndim == 0
                      else arm_armature.astype(float))

    if model.nq != spec.dof:
        raise RuntimeError(f"expected a {spec.dof}-DOF arm after locking, got nq={model.nq}")
    names = list(model.names)[1:]
    if names != arm:
        raise RuntimeError(f"unexpected joint order: {names}, expected {arm}")
    if not model.existFrame(ee_frame):
        raise RuntimeError(f"end-effector frame {ee_frame!r} not in the model")

    return RobotModel(
        model=model,
        data=model.createData(),
        ee_frame=ee_frame,
        friction_scale=np.ones(model.nv) if friction else np.zeros(model.nv),
        spec=spec,
    )
