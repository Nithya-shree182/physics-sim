"""The one place the plant is built.

Every capture / sweep script used to carry its own ~60-line copy of "build the arm, author
the URDF inertials, set friction", and the copies drifted: that is how an invalid rate sweep
(a World rebuilt in one process, physics_dt silently ignored) got into the docs. Scripts now
call build_plant() once per process.

Import this only AFTER SimulationApp has started (it imports Isaac modules).

Order matters and is deliberate: every tensor-API write (masses, COMs, inertias, gains,
armature, friction) happens AFTER the final world.reset(), because reset discards them.
"""
import xml.etree.ElementTree as ET
from types import SimpleNamespace

import numpy as np
from pxr import PhysxSchema
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation

import pal_config as cfg

ARM = 7
PAYLOAD_TCP_Z = 0.1034   # m along panda_hand z; where the analysis side's model puts the payload


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
            "diag": np.array([float(I.get("ixx")), float(I.get("iyy")), float(I.get("izz"))])}
    return out


def build_plant(hz, payload_kg=0.0, kp=400.0, kd=80.0):
    """Build the Franka plant at `hz` and return a namespace with world, art, idx, fric.

    kp, kd are N*m/rad and N*m*s/rad (position_stiff drive). Friction is Gaz x
    PLANT_FRICTION_SCALE from pal_config; armature is the physical 0.0061.
    """
    dt = 1.0 / hz
    world = World(stage_units_in_meters=1.0, physics_dt=dt, rendering_dt=1 / 30.0)
    pc = world.get_physics_context()
    pc.set_solver_type(cfg.SOLVER_TYPE); pc.enable_ccd(cfg.ENABLE_CCD)
    pc.enable_stablization(cfg.ENABLE_STABILIZATION); pc.set_gravity(cfg.GRAVITY)
    sp = PhysxSchema.PhysxSceneAPI.Apply(pc.get_current_physics_scene_prim())
    sp.CreateMinPositionIterationCountAttr(cfg.POSITION_ITERATIONS)
    sp.CreateMinVelocityIterationCountAttr(cfg.VELOCITY_ITERATIONS)

    add_reference_to_stage(usd_path=get_assets_root_path() + cfg.ROBOT_USD_SUBPATH,
                           prim_path=cfg.ROBOT_PRIM)
    art = Articulation(cfg.ROBOT_PRIM)
    world.reset(); art.initialize()
    idx = np.arange(ARM)

    U = _urdf_inertials(cfg.URDF_PATH)
    bodies = list(art.body_names); nb = len(bodies)
    cur = np.asarray(art.get_body_masses()).reshape(nb)
    masses, coms, diags = [], [], []
    for i, b in enumerate(bodies):
        if b not in U:
            masses.append(float(cur[i])); coms.append(np.zeros(3)); diags.append(np.full(3, 1e-3))
            continue
        m0, c0, d0 = U[b]["mass"], U[b]["com"], U[b]["diag"]
        if b == "panda_hand" and payload_kg > 0:
            # hand + point mass at the TCP: combined mass, combined COM, parallel-axis inertia
            cp = np.array([0.0, 0.0, PAYLOAD_TCP_Z])
            m = m0 + payload_kg
            c = (m0 * c0 + payload_kg * cp) / m
            dh, dp = c0 - c, cp - c
            d = d0 + m0 * (np.dot(dh, dh) - dh ** 2) + payload_kg * (np.dot(dp, dp) - dp ** 2)
            masses.append(m); coms.append(c); diags.append(d)
        else:
            masses.append(m0); coms.append(c0); diags.append(d0)
    masses = np.array(masses, np.float32)
    coms = np.array(coms, np.float32)
    diags = np.array(diags, np.float32)
    quats = np.tile(np.array([1., 0, 0, 0], np.float32), (nb, 1))

    art.set_gains(kps=np.full((1, ARM), kp * cfg.DEG_PER_RAD, np.float32),
                  kds=np.full((1, ARM), kd * cfg.DEG_PER_RAD, np.float32), joint_indices=idx)
    art.set_body_masses(masses.reshape(1, nb))
    art.set_body_coms(coms.reshape(1, nb, 3), quats.reshape(1, nb, 4))
    try:
        art.set_body_inertias(diags.reshape(1, nb, 3))
    except Exception:
        f = np.zeros((nb, 9), np.float32); f[:, 0], f[:, 4], f[:, 8] = diags.T
        art.set_body_inertias(f.reshape(1, nb, 9))
    art.set_armatures(cfg.ARMATURE.reshape(1, ARM).astype(np.float32), joint_indices=idx)
    fric = cfg.FRICTION_COULOMB_STATIC * cfg.PLANT_FRICTION_SCALE
    art.set_friction_coefficients(fric.reshape(1, ARM).astype(np.float32), joint_indices=idx)

    # Read back AFTER the writes: a readback before a reset proves nothing (see CONTEXT.md).
    m_back = np.asarray(art.get_body_masses()).reshape(nb)
    assert np.allclose(m_back, masses, atol=1e-4), "mass write did not take"
    f_back = np.asarray(art.get_friction_coefficients()).reshape(-1)[:ARM]
    assert np.allclose(f_back, fric, atol=1e-5), "friction write did not take"
    print(f"[plant] {hz:.0f} Hz | kp {kp:g} kd {kd:g} | payload {payload_kg:g} kg | "
          f"total mass {m_back.sum():.3f} kg | friction readback ok", flush=True)

    return SimpleNamespace(world=world, art=art, idx=idx, fric=fric, hz=hz, dt=dt,
                           payload_kg=payload_kg)
