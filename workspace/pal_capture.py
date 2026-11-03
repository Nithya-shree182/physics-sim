#!/usr/bin/env python3
"""Capture a telemetry run in schema/telemetry_v0.yaml format.

    PAL_TRAJ=slow_reference PAL_PAYLOAD_KG=0 PAL_STEPS=4000 ./run_capture.sh

Payload is attached as a point mass at the TCP, 0.1034 m along panda_hand's z axis, which
is where the analysis side's synthetic plant puts it. The earlier approach of simply
raising panda_hand's mass put the load at the hand origin instead, and also produced a
misleading lever arm: the asset's hand is 0.558 kg, not the URDF's 0.730, so writing
`0.73 + 3.0` actually added 3.172 kg and made 13.89 N*m look like 4.896 N*m/kg when the
true figure is 4.631.
"""
import json
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_capture.py running", flush=True)

from pxr import PhysxSchema
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation
import pal_config as cfg
from pal_telemetry import TelemetryWriter

ARM = 7
HZ = float(os.environ.get("PAL_PHYSICS_HZ", cfg.PHYSICS_HZ))
DT = 1.0 / HZ
STEPS = int(os.environ.get("PAL_STEPS", 4000))
TRAJ = os.environ.get("PAL_TRAJ", "slow_reference")
PAYLOAD = float(os.environ.get("PAL_PAYLOAD_KG", 0.0))
MJ_EVERY = int(os.environ.get("PAL_MJ_EVERY", 1))
RUN_ID = os.environ.get("PAL_RUN_ID", f"{TRAJ}_{PAYLOAD:g}kg_{int(HZ)}hz")
TCP_Z = 0.1034                      # m along panda_hand z; matches the analysis side

world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=1/30.0)
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

# ── URDF inertials, plus the payload folded into panda_hand at the TCP ───────────────
root = ET.parse(cfg.URDF_PATH).getroot()
U = {}
for l in root.findall("link"):
    it = l.find("inertial")
    if it is None:
        continue
    m, o, I = it.find("mass"), it.find("origin"), it.find("inertia")
    U[l.get("name")] = {"mass": float(m.get("value")),
        "com": np.array([float(x) for x in o.get("xyz").split()])
               if o is not None and o.get("xyz") else np.zeros(3),
        "diag": np.array([float(I.get("ixx")), float(I.get("iyy")), float(I.get("izz"))])}

bodies = list(art.body_names); nb = len(bodies)
cur = np.asarray(art.get_body_masses()).reshape(nb)
masses, coms, diags = [], [], []
for i, b in enumerate(bodies):
    if b not in U:
        masses.append(float(cur[i])); coms.append(np.zeros(3)); diags.append(np.full(3, 1e-3))
        continue
    m0, c0, d0 = U[b]["mass"], U[b]["com"], U[b]["diag"]
    if b == "panda_hand" and PAYLOAD > 0:
        # Composite of hand + point mass at the TCP: combined mass, combined COM, and the
        # payload's parallel-axis contribution about that combined COM.
        cp = np.array([0.0, 0.0, TCP_Z])
        m = m0 + PAYLOAD
        c = (m0 * c0 + PAYLOAD * cp) / m
        dh, dp = c0 - c, cp - c
        d = d0 + m0 * (np.dot(dh, dh) - dh**2) + PAYLOAD * (np.dot(dp, dp) - dp**2)
        masses.append(m); coms.append(c); diags.append(d)
    else:
        masses.append(m0); coms.append(c0); diags.append(d0)

masses = np.array(masses, np.float32)
coms = np.array(coms, np.float32)
diags = np.array(diags, np.float32)
quats = np.tile(np.array([1., 0, 0, 0], np.float32), (nb, 1))

art.set_gains(kps=np.full((1, ARM), 400.0 * cfg.DEG_PER_RAD, np.float32),
              kds=np.full((1, ARM), 80.0 * cfg.DEG_PER_RAD, np.float32), joint_indices=idx)
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

# ── Trajectory ────────────────────────────────────────────────────────────────────────
F0 = np.array([.55, .45, .60, .50, .75, .70, .80])
K = sum(1. / (h + 1) for h in range(3))
FRAC = {"hold": 0.0, "slow_reference": 0.20, "excitation": 0.60}.get(TRAJ, 0.20)
A = FRAC * cfg.QD_MAX / (2 * np.pi * F0 * K)


def traj(t):
    q = cfg.HOME_JOINTS.copy(); qd = np.zeros(ARM); qdd = np.zeros(ARM)
    for h in range(3):
        w = 2 * np.pi * F0 * (h + 1); a = A / (h + 1) ** 2
        q += a * np.sin(w * t); qd += a * w * np.cos(w * t); qdd -= a * w * w * np.sin(w * t)
    return q, qd, qdd


q0, _, _ = traj(0.0)
art.set_joint_positions(np.concatenate([q0, [.04, .04]]))
for _ in range(400):
    world.step(render=False)

writer = TelemetryWriter(
    art, RUN_ID, out_dir="/workspace/data",
    ee_body_index=cfg.EE_JACOBIAN_ROW, ee_prim=f"{cfg.ROBOT_PRIM}/panda_hand",
    mj_every=MJ_EVERY,
    labels={"traj_type": TRAJ, "drive_mode": "position_stiff",
            "payload_kg": PAYLOAD, "payload_attached": PAYLOAD > 0,
            "fault_type": "none", "fault_joint": -1, "fault_param": np.nan,
            "attack_type": "none"},
    manifest={"git_commit": os.environ.get("PAL_GIT_COMMIT", "uncommitted"),
              "machine": os.environ.get("PAL_MACHINE", "omen-5090"),
              "isaac_sim_version": "6.0.1", "physics_hz": HZ,
              "solver": {"type": cfg.SOLVER_TYPE,
                         "position_iterations": cfg.POSITION_ITERATIONS,
                         "velocity_iterations": cfg.VELOCITY_ITERATIONS},
              "asset": {"usd": cfg.ROBOT_USD_SUBPATH, "urdf": cfg.URDF_PATH,
                        "inertials_authored_from_urdf": True},
              "armature": cfg.ARMATURE.tolist(),
              "friction_source": cfg.FRICTION_SOURCE,
              "plant_friction_scale": cfg.PLANT_FRICTION_SCALE,
              "friction_values": fric.tolist(),
              "torque_convention": cfg.TORQUE_CONVENTION,
              "gravity": [0.0, 0.0, cfg.GRAVITY], "seed": 0,
              "payload_attachment": {"body": "panda_hand", "offset_m": [0, 0, TCP_Z],
                                     "model": "point mass folded into the hand's composite"},
              "finger_q_ref": cfg.FINGER_OPEN})

for k in range(STEPS):
    t = k * DT
    qt, qdt, qddt = traj(t)
    art.set_joint_position_targets(np.concatenate([qt, [.04, .04]]).reshape(1, -1))
    world.step(render=False)
    writer.capture(k, t, DT, q_cmd=qt, qd_cmd=qdt, qdd_cmd=qddt)

writer.close()
print(f"[capture] traj={TRAJ} payload={PAYLOAD}kg steps={STEPS} mj_every={MJ_EVERY}", flush=True)
simulation_app.close()
