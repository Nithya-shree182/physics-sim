#!/usr/bin/env python3
"""O4 data: known payload masses, held still at three poses and carried along the reference motion.

Known masses are the ground truth the estimator is scored against (proposal section 9: "error
against known test masses"). Every pose has a 0 kg run, which the estimator can use as a tare.
The plant carries the deliberate 5% arm-mass mismatch and sensor noise.

The payload is a rigid point mass at the tool centre point (0.1034 m along panda_hand z), not a
grasped object: there is no gripper actuation in this scene.

    tools/pal_pipeline.sh pal_o4_payload.py out/o4
"""
import json
import os

import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_o4_payload.py running", flush=True)

import pal_config as cfg
import pal_trajectories as T
from pal_plant import build_plant
from pal_runner import Runner, Scenario

HZ = float(os.environ.get("PAL_PHYSICS_HZ", cfg.PHYSICS_HZ))
OUT = "/workspace/out/o4"
plant = build_plant(HZ)
runner = Runner(plant, OUT, mj_every=10)

MASSES = (0.0, 0.5, 1.0, 2.0, 3.0)
POSES = ("home", "horizontal_reach", "folded")
runs = []
i = 0
for pose in POSES:
    for kg in MASSES:
        runs.append(Scenario(name=f"o4_hold_{pose}_{kg:g}kg".replace(".", "p"), traj=T.hold(cfg.PROBE_POSES[pose]),
                             traj_type="hold", duration_s=3.0, payload_kg=kg, noise_seed=2000 + i))
        i += 1
for kg in (0.0, 0.5, 1.0, 3.0):
    runs.append(Scenario(name=f"o4_move_{kg:g}kg".replace(".", "p"), traj=T.multisine(0.20),
                         traj_type="slow_reference", duration_s=6.0, payload_kg=kg, noise_seed=2000 + i))
    i += 1

meta = []
for sc in runs:
    runner.run(sc)
    meta.append({"run_id": f"{sc.name}_{int(HZ)}hz", "payload_kg": sc.payload_kg, "traj_type": sc.traj_type,
                 "pose": next((p for p in POSES if p in sc.name), "moving")})
    print(f"  {sc.name:<34} payload {sc.payload_kg:g} kg", flush=True)
json.dump({"runs": meta}, open(f"{OUT}/index.json", "w"), indent=2)
print(f"\n  written {len(runs)} runs to out/o4", flush=True)
simulation_app.close()
