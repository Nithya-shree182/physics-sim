"""The one place the plant is built.

"Plant" is control-engineering for the thing being controlled: here, the simulated Franka arm
with its masses, friction and motors. It stands in for the real robot; the analysis side's
Pinocchio model is the separate prediction it is compared against.

Every capture / sweep script used to carry its own ~60-line copy of "build the arm, author
the URDF inertials, set friction", and the copies drifted: that is how an invalid rate sweep
(a World rebuilt in one process, physics_dt silently ignored) got into the docs. Scripts now
call build_plant() once per process.

Import this only AFTER SimulationApp has started (it imports Isaac modules).

Order matters and is deliberate: every tensor-API write happens AFTER the final world.reset(),
because reset discards them. Every write is read back and asserted.

FRICTION (fixed 2026-11-03, docs/REVIEW.md F1). The old call, set_friction_coefficients, takes a
DIMENSIONLESS PhysX coefficient; passing Gaz's N*m values produced ~38 N*m of friction on joint 2.
Friction here goes through the physics view's set_dof_friction_properties, which takes a static
effort and a dynamic effort in N*m and a viscous coefficient in N*m*s/rad. Verified by
pal_selftest.py: 1.4 N*m injected shows up as 1.388 N*m.

MASS MISMATCH. The proposal requires the twin's model to differ from the plant ("otherwise
detection would succeed trivially"). `link_mass_scale` scales the mass and inertia of arm links
1-7 only (not the gripper, which is part of the robot, not the payload). The analysis side's
model keeps the URDF values.
"""
import xml.etree.ElementTree as ET

import numpy as np
from pxr import PhysxSchema
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation

import pal_config as cfg

ARM = 7
PAYLOAD_TCP_Z = 0.1034        # m along panda_hand z; where the analysis side's model puts the payload
DEFAULT_MASS_SCALE = 1.05     # [CHOICE] deliberate plant-vs-model mass mismatch on arm links 1-7
ARM_LINKS = tuple(f"panda_link{i}" for i in range(1, 8))


def _urdf_inertials(path):
    out = {}
    for l in ET.parse(path).getroot().findall("link"):
        it = l.find("inertial")
        if it is None:
            continue
        m, o, I = it.find("mass"), it.find("origin"), it.find("inertia")
        out[l.get("name")] = {
            "mass": float(m.get("value")),
            "com": np.array([float(x) for x in o.get("xyz").split()])
                   if o is not None and o.get("xyz") else np.zeros(3),
            "tensor": np.array([[float(I.get("ixx")), float(I.get("ixy")), float(I.get("ixz"))],
                                [float(I.get("ixy")), float(I.get("iyy")), float(I.get("iyz"))],
                                [float(I.get("ixz")), float(I.get("iyz")), float(I.get("izz"))]])}
    return out


def base_friction():
    """(7, 3) static N*m, dynamic N*m, viscous N*m*s/rad: Gaz et al. x PLANT_FRICTION_SCALE."""
    s = cfg.PLANT_FRICTION_SCALE
    return np.stack([cfg.FRICTION_COULOMB_STATIC * s, cfg.FRICTION_COULOMB_DYNAMIC * s,
                     cfg.FRICTION_VISCOUS * s], axis=1)


class Plant:
    def __init__(self, hz, kp=400.0, kd=80.0):
        self.hz, self.dt = hz, 1.0 / hz
        self.world = World(stage_units_in_meters=1.0, physics_dt=self.dt, rendering_dt=1 / 30.0)
        pc = self.world.get_physics_context()
        pc.set_solver_type(cfg.SOLVER_TYPE); pc.enable_ccd(cfg.ENABLE_CCD)
        pc.enable_stablization(cfg.ENABLE_STABILIZATION); pc.set_gravity(cfg.GRAVITY)
        sp = PhysxSchema.PhysxSceneAPI.Apply(pc.get_current_physics_scene_prim())
        sp.CreateMinPositionIterationCountAttr(cfg.POSITION_ITERATIONS)
        sp.CreateMinVelocityIterationCountAttr(cfg.VELOCITY_ITERATIONS)
        add_reference_to_stage(usd_path=get_assets_root_path() + cfg.ROBOT_USD_SUBPATH,
                               prim_path=cfg.ROBOT_PRIM)
        self.art = Articulation(cfg.ROBOT_PRIM)
        self.world.reset(); self.art.initialize()
        self.idx = np.arange(ARM)
        self.pv = self.art._physics_view
        self.bodies = list(self.art.body_names)
        self.urdf = _urdf_inertials(cfg.URDF_PATH)
        self.stock_masses = np.asarray(self.art.get_body_masses()).reshape(-1)
        self.art.set_gains(kps=np.full((1, ARM), kp * cfg.DEG_PER_RAD, np.float32),
                           kds=np.full((1, ARM), kd * cfg.DEG_PER_RAD, np.float32),
                           joint_indices=self.idx)
        self.art.set_armatures(cfg.ARMATURE.reshape(1, ARM).astype(np.float32), joint_indices=self.idx)
        self.base_friction = base_friction()
        self.payload_kg, self.mass_scale = 0.0, 1.0
        self.set_inertials(0.0, DEFAULT_MASS_SCALE)
        self.set_friction(self.base_friction)

    # -- inertials ------------------------------------------------------------------------
    def set_inertials(self, payload_kg, mass_scale):
        """Author masses/COMs/inertias from the URDF, scaled arm links, payload on the hand.

        Callable between runs (no reset needed); read back and asserted every time."""
        nb = len(self.bodies)
        masses, coms, tensors = [], [], []
        for i, b in enumerate(self.bodies):
            if b not in self.urdf:
                masses.append(float(self.stock_masses[i])); coms.append(np.zeros(3))
                tensors.append(np.eye(3) * 1e-3); continue
            u = self.urdf[b]
            m0, c0, T0 = u["mass"], u["com"], u["tensor"]
            if b in ARM_LINKS:
                m0, T0 = m0 * mass_scale, T0 * mass_scale
            if b == "panda_hand" and payload_kg > 0:
                # hand + point mass at the TCP: combined mass and COM; full tensor by parallel axes
                cp = np.array([0.0, 0.0, PAYLOAD_TCP_Z])
                m = m0 + payload_kg
                c = (m0 * c0 + payload_kg * cp) / m
                dh, dp = c0 - c, cp - c
                T = (T0 + m0 * (np.dot(dh, dh) * np.eye(3) - np.outer(dh, dh))
                     + payload_kg * (np.dot(dp, dp) * np.eye(3) - np.outer(dp, dp)))
                m0, c0, T0 = m, c, T
            masses.append(m0); coms.append(c0); tensors.append(T0)
        masses = np.array(masses, np.float32); coms = np.array(coms, np.float32)
        quats = np.tile(np.array([1.0, 0, 0, 0], np.float32), (nb, 1))
        self.art.set_body_masses(masses.reshape(1, nb))
        self.art.set_body_coms(coms.reshape(1, nb, 3), quats.reshape(1, nb, 4))
        # set_body_inertias takes the FULL 3x3 tensor (9 numbers) in the link frame; it rejects the
        # 3-number form. Before 2026-11-03 only the diagonal was written (off-diagonal terms dropped,
        # up to 0.008 kg*m^2 on M). Verified end to end by pal_selftest.py T5 against an independent
        # numpy mass matrix (pal_inertia_convention.py tried the alternatives: a principal-axes
        # quaternion is NOT how this API works).
        self.art.set_body_inertias(np.array(tensors, np.float32).reshape(1, nb, 9))
        back = np.asarray(self.art.get_body_masses()).reshape(nb)
        assert np.allclose(back, masses, atol=1e-4), "mass write did not take"
        self.payload_kg, self.mass_scale = payload_kg, mass_scale

    # -- friction -------------------------------------------------------------------------
    def set_friction(self, props7x3):
        """Write (7, 3) [static N*m, dynamic N*m, viscous N*m*s/rad]; fingers keep zero."""
        props7x3 = np.asarray(props7x3, np.float64)
        full = np.asarray(self.pv.get_dof_friction_properties()).copy()      # (1, 9, 3)
        full[0, :ARM, :] = props7x3
        self.pv.set_dof_friction_properties(full.astype(np.float32), np.array([0], dtype=np.int32))
        back = np.asarray(self.pv.get_dof_friction_properties())[0, :ARM, :]
        assert np.allclose(back, props7x3, atol=1e-5), "friction write did not take"
        self.friction = props7x3.copy()

    def to_effort_mode(self):
        """Drive off (kp = kd = 0) and the arm joints switched to explicit torque commands."""
        self.art.set_gains(kps=np.zeros((1, ARM), np.float32), kds=np.zeros((1, ARM), np.float32),
                           joint_indices=self.idx)
        self.art.switch_control_mode("effort", joint_indices=self.idx)

    # -- external force (validation push) ---------------------------------------------------
    def push(self, force_xyz, link_name="panda_hand"):
        """Apply a world-frame force (N) at the origin of `link_name`; zero clears it."""
        nb = len(self.bodies)
        f = np.zeros((1, nb, 3), np.float32); f[0, self.bodies.index(link_name)] = force_xyz
        t = np.zeros((1, nb, 3), np.float32)
        self.pv.apply_forces_and_torques_at_position(f, t, None, np.array([0], dtype=np.int32), True)


def build_plant(hz, payload_kg=0.0, mass_scale=DEFAULT_MASS_SCALE, kp=400.0, kd=80.0):
    plant = Plant(hz, kp=kp, kd=kd)
    plant.set_inertials(payload_kg, mass_scale)
    print(f"[plant] {hz:.0f} Hz | payload {payload_kg:g} kg | arm link mass scale {mass_scale:g} | "
          f"friction (N*m) static {plant.friction[:, 0].round(3).tolist()} | readbacks ok", flush=True)
    return plant
