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

from pal_plant import build_plant, PAYLOAD_TCP_Z as TCP_Z
plant = build_plant(HZ, payload_kg=PAYLOAD)
world, art, idx, fric = plant.world, plant.art, plant.idx, plant.fric

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
    t_cmd = k * DT                      # time the command is issued
    qt, qdt, qddt = traj(t_cmd)
    art.set_joint_position_targets(np.concatenate([qt, [.04, .04]]).reshape(1, -1))
    world.step(render=False)
    # The row holds POST-step values, so its timestamp is the end of the step, not the
    # start. Caught by the analysis side (R2.2): t = k*DT lagged the data by one step.
    writer.capture(k, (k + 1) * DT, DT, q_cmd=qt, qd_cmd=qdt, qdd_cmd=qddt)

writer.close()
print(f"[capture] traj={TRAJ} payload={PAYLOAD}kg steps={STEPS} mj_every={MJ_EVERY}", flush=True)
simulation_app.close()
