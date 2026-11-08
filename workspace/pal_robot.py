"""Which arm this project is running, and every number that changes when the arm changes.

Until 2026-11-08 the arm was a Franka Panda and its constants were spread over `pal_config.py`,
`model/pinocchio_model.py` and `model/poses.py`. Porting to a UR10e meant finding all of them, so
they now live here, once, as a `RobotSpec`. Pure numpy: no Isaac and no Pinocchio import, so the
simulator container and the host-side analysis can both read it.

    import pal_robot
    R = pal_robot.active()            # honours $PAL_ROBOT, default "ur10e"
    R.dof, R.tau_max, R.home          # ... etc

The arm is the UR10e. The Franka Panda this project started on was removed on 2026-11-08; the
structure stays because it is what makes the arm-specific numbers findable in one place, and
because adding a second arm should not mean another hunt through ten files. `$PAL_ROBOT` selects
among whatever is defined here.

A note on provenance. The UR10e's masses, centres of mass, full 3x3 inertia tensors, link offsets
and limits are Universal Robots' own published values. But **nobody has published an identified
friction or rotor-inertia set for a UR10e**, so those two are authored here and tagged [CHOICE]. That is deliberate and it is sound
for what this project is: a digital twin for studying how physics parameters move the residual, not
a model claiming to match a particular physical robot (see docs/HANDOFF.md, the 2026-11-08 entry).
The plant and the model both derive from the same URDF with a controlled mismatch, so the method
stays self-consistent whatever the absolute accuracy. If these ever need to be real, identify them
from healthy data rather than looking for a citation that does not exist.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
DEG = np.pi / 180.0


@dataclass(frozen=True)
class SensorSpec:
    """The joint sensors. Idealised on purpose: these are study parameters, not a hardware claim.

    Since 2026-11-08 the project does not try to reproduce what a real controller can measure (a
    real UR reports current-derived torque, which could not carry this method at all). Instead the
    twin is given clean joint torque sensing and the noise is a knob, so the question "how good must
    the sensing be before detection fails?" can be answered by sweeping it."""
    q: float                 # rad, white noise on the position reading
    qd: float                # rad/s
    tau: float               # N*m
    bias_tau: float = 0.0    # N*m, constant per-joint offset (0 = perfectly calibrated)
    bandwidth_hz: float = 0.0  # 0 = ideal; >0 applies a first-order lag to the torque reading


@dataclass(frozen=True)
class RobotSpec:
    name: str
    dof: int
    urdf: str                       # path inside the simulator container
    urdf_host: str                  # path on the host, for Pinocchio and the checks
    joint_names: tuple
    home: np.ndarray                # rad
    q_min: np.ndarray
    q_max: np.ndarray
    qd_max: np.ndarray              # rad/s, datasheet velocity limits
    tau_max: np.ndarray             # N*m, datasheet effort limits
    armature: np.ndarray            # kg*m^2, reflected rotor inertia added to the mass matrix diagonal
    friction_static: np.ndarray     # N*m, Coulomb
    friction_dynamic: np.ndarray    # N*m
    friction_viscous: np.ndarray    # N*m*s/rad
    # The MODEL side's friction law: tau_f(qd) = phi1/(1+exp(-phi2(qd+phi3))) - phi1/(1+exp(-phi2 phi3)).
    # A smooth stand-in for Coulomb friction that is exactly zero at rest. The plant applies
    # static/dynamic/viscous through PhysX instead, and the gap between the two representations is
    # part of what the residual measures.
    friction_phi1: np.ndarray       # N*m, the saturation torque
    friction_phi2: np.ndarray       # s/rad, how sharply it rises off zero
    friction_phi3: np.ndarray       # rad/s, offset
    sensors: SensorSpec
    tool_frame: str                 # the URDF frame the tool offset is measured from
    tool_offset: tuple              # m, TCP along the tool frame's z
    usd_subpath: str                # Isaac asset-server path for the visual/articulation asset
    usd_candidates: tuple           # fallbacks, because asset paths move between Isaac releases
    prim: str                       # where it is placed on the stage
    base_z: float = 0.0             # m, where the robot's base sits in world z
    reach: float = 0.0              # m, nominal datasheet reach, for sanity checks and scene layout
    notes: dict = field(default_factory=dict)

    @property
    def idx(self) -> np.ndarray:
        return np.arange(self.dof)

    def urdf_file(self) -> str:
        """The URDF to actually open.

        There are two paths because there are two filesystems: `urdf` is where the simulator
        container sees it (the repo's assets are bind-mounted under /workspace), `urdf_host` is
        where it sits in the checkout. Inside the container `urdf_host` resolves against `/` and
        does not exist, so always ask through here rather than picking one."""
        for cand in (self.urdf, self.urdf_host):
            if cand and os.path.exists(cand):
                return cand
        raise FileNotFoundError(
            f"{self.name}: no URDF at {self.urdf!r} (container) or {self.urdf_host!r} (host). "
            f"If this is the UR10e, generate it with tools/make_ur10e_urdf.py.")

    def friction_3col(self, scale: float = 1.0) -> np.ndarray:
        """(dof, 3) static, dynamic, viscous — the shape pal_plant.set_friction expects."""
        return np.stack([self.friction_static * scale, self.friction_dynamic * scale,
                         self.friction_viscous * scale], axis=1)

    def check(self) -> None:
        n = self.dof
        for attr in ("home", "q_min", "q_max", "qd_max", "tau_max", "armature",
                     "friction_static", "friction_dynamic", "friction_viscous"):
            v = getattr(self, attr)
            assert v.shape == (n,), f"{self.name}.{attr} has shape {v.shape}, expected ({n},)"
        assert len(self.joint_names) == n, f"{self.name}: {len(self.joint_names)} joint names for {n} joints"
        assert np.all(self.q_min < self.q_max), f"{self.name}: q_min must be below q_max"
        assert np.all(self.q_min <= self.home) and np.all(self.home <= self.q_max), \
            f"{self.name}: the home pose is outside the joint limits"
        assert np.all(self.tau_max > 0) and np.all(self.qd_max > 0), f"{self.name}: limits must be positive"


# ── Universal Robots UR10e ─────────────────────────────────────────────────────────────────────
# The arm from 2026-11-08 on. Chosen for three reasons: 1300 mm of reach instead of 855 mm, which
# makes a real conveyor cell fit; 6 joints instead of 7, so IK is square and has no null space; and
# it is the arm an industrial audience recognises.
#
# Masses, centres of mass, full 3x3 inertia tensors, link offsets and all three limit sets come
# from Universal Robots' own config (see tools/make_ur10e_urdf.py, which generates the URDF from
# them). Pinocchio reads 29.104 kg of moving mass from that URDF; adding the 4.0 kg fixed base gives
# 33.104 kg against a 33.5 kg datasheet figure, which is the cross-check that the chain is right.
_UR_TAU = np.array([330.0, 330.0, 150.0, 54.0, 54.0, 54.0])

UR10E = RobotSpec(
    name="ur10e",
    dof=6,
    urdf="/workspace/assets_all/ur10e/ur10e.urdf",
    urdf_host=str(REPO / "assets" / "ur10e" / "ur10e.urdf"),
    joint_names=("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                 "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"),
    # The standard UR "ready" pose. Measured from the generated URDF: it puts tool0 at
    # (0.691, 0.174, 0.677) m with the tool z axis pointing exactly straight down, which is what a
    # suction pick needs, and the worst gravity torque is 42.6 N*m of a 330 N*m limit.
    home=np.array([0.0, -90.0, 90.0, -90.0, -90.0, 0.0]) * DEG,
    q_min=np.array([-360.0, -360.0, -180.0, -360.0, -360.0, -360.0]) * DEG,
    q_max=np.array([360.0, 360.0, 180.0, 360.0, 360.0, 360.0]) * DEG,
    qd_max=np.array([120.0, 120.0, 180.0, 180.0, 180.0, 180.0]) * DEG,
    tau_max=_UR_TAU,
    # [CHOICE] Reflected rotor inertia. UR publishes no figure. These are sized as J_motor * N^2 for
    # a ~100:1 harmonic drive, which lands the big joints near 10% of their link inertia — the right
    # order for a geared arm, and small enough not to dominate the mass matrix. Identify if it matters.
    armature=np.array([0.10, 0.10, 0.05, 0.02, 0.02, 0.02]),
    # [CHOICE] Friction. No identified set exists for a UR10e. Coulomb is set at 1% of each joint's
    # own effort limit (the Panda's Gaz values sit near 0.3% of its limits; harmonic drives rub more
    # than the Panda's, so 1% is the deliberate, documented step up), dynamic at 0.85 of static as
    # for the Panda, and viscous scaled the same way. These are the numbers a friction fault is
    # multiplied against, so they set the detection scale: treat them as a knob, not a measurement.
    friction_static=0.010 * _UR_TAU,
    friction_dynamic=0.0085 * _UR_TAU,
    friction_viscous=np.array([1.00, 1.00, 0.50, 0.20, 0.20, 0.20]),
    # [CHOICE] No identified sigmoid exists for a UR10e. The saturation torque is the Coulomb
    # figure above; the steepness is one value for every joint, inside the 5-17 s/rad range Gaz
    # measured on the Panda, and there is no offset. Like the Coulomb values these set the scale a
    # friction fault is measured against, so treat them as a knob.
    friction_phi1=0.010 * _UR_TAU,
    friction_phi2=np.full(6, 12.0),
    friction_phi3=np.zeros(6),
    # Idealised sensing (see SensorSpec). tau sigma 0.05 N*m holds the Panda's noise-to-limit ratio
    # (0.02 of 87) at this arm's much larger torques, so detection results stay comparable.
    sensors=SensorSpec(q=2e-5, qd=1e-3, tau=0.05),
    tool_frame="tool0",
    tool_offset=(0.0, 0.0, 0.0),        # tool0 is the bare flange; the suction cups add their own offset
    usd_subpath="/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd",
    usd_candidates=("/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd",
                    "/Isaac/Robots/UniversalRobots/ur10e/ur10e_instanceable.usd",
                    "/Isaac/Robots/UR10e/ur10e.usd",
                    "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"),
    prim="/World/UR10e",
    reach=1.300,
    notes={"gripper": "suction cup array (pal_suction.py)",
           "friction_source": "[CHOICE] authored, see pal_robot.py",
           "kinematics_hash": "calib_5119701370761913513"},
)

ROBOTS = {r.name: r for r in (UR10E,)}
DEFAULT_ROBOT = "ur10e"


def get(name: str) -> RobotSpec:
    try:
        spec = ROBOTS[name]
    except KeyError:
        raise ValueError(f"unknown robot {name!r}; have {sorted(ROBOTS)}") from None
    spec.check()
    return spec


def active() -> RobotSpec:
    """The arm this process is running, from $PAL_ROBOT (default ur10e)."""
    return get(os.environ.get("PAL_ROBOT", DEFAULT_ROBOT).strip().lower())


if __name__ == "__main__":                                   # a quick look: `python workspace/pal_robot.py`
    for spec in ROBOTS.values():
        spec.check()
        print(f"\n{spec.name}  ({spec.dof} joints, reach {spec.reach} m)")
        print(f"  home     {np.round(spec.home, 3)}")
        print(f"  tau_max  {spec.tau_max}")
        print(f"  qd_max   {np.round(spec.qd_max, 3)}")
        print(f"  friction {np.round(spec.friction_static, 3)} N*m static")
        print(f"  armature {spec.armature}")
        print(f"  urdf     {spec.urdf_host}  (exists: {Path(spec.urdf_host).exists()})")
