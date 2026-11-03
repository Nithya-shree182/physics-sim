#!/usr/bin/env python3
"""Reference captures for calibrating the analysis side's model (proposal 7.2).

Slow reference trajectory (20% of the velocity limit), effort mode, no sensor noise, and NO
mass mismatch (mass scale 1.0) so a model built from the same URDF should close on it. Two runs:
0 kg and 1 kg. Written to data/ (tracked) for hand-over.

    tools/pal_pipeline.sh pal_capture.py data
"""
import os

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_capture.py running", flush=True)

import pal_config as cfg
import pal_trajectories as T
from pal_plant import build_plant
from pal_runner import Runner, Scenario

HZ = float(os.environ.get("PAL_PHYSICS_HZ", cfg.PHYSICS_HZ))
plant = build_plant(HZ, mass_scale=1.0)
runner = Runner(plant, "/workspace/data", mj_every=int(os.environ.get("PAL_MJ_EVERY", 10)))
for kg in (0.0, 1.0):
    sc = Scenario(name=f"ref_matched_{kg:g}kg".replace(".", "p"), traj=T.multisine(0.20),
                  traj_type="slow_reference", duration_s=float(os.environ.get("PAL_DURATION_S", 6.0)),
                  payload_kg=kg, mass_scale=1.0, noise_seed=None)
    runner.run(sc)
    print(f"[capture] {sc.name} payload {kg:g} kg", flush=True)
simulation_app.close()
