#!/usr/bin/env python3
"""Noise-free twins of the key O5 faults, to separate what the detector can do from what sensor noise takes away.

Same plant, trajectory, mass mismatch and fault timing as pal_o5_sweep.py, but NO sensor noise. Healthy, friction
x2 / x4 / x8 on joints 2 and 4, a locked joint 4, and a 50 ms delay.

    tools/pal_pipeline.sh pal_o5_clean.py out/clean
"""
import os

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_o5_clean.py running", flush=True)

import pal_config as cfg
import pal_trajectories as T
from pal_faults import FaultSpec
from pal_plant import build_plant
from pal_runner import Runner, Scenario

HZ = float(os.environ.get("PAL_PHYSICS_HZ", cfg.PHYSICS_HZ))
plant = build_plant(HZ)
runner = Runner(plant, "/workspace/out/clean", mj_every=10)
slow = T.multisine(0.20)
specs = [FaultSpec("none")]
specs += [FaultSpec("friction", j, m, 3.0) for j in (1, 3) for m in (2.0, 4.0, 8.0)]
specs += [FaultSpec("stuck_joint", 3, 200.0, 3.0), FaultSpec("delay", 0, 0.050, 3.0)]
for f in specs:
    runner.run(Scenario(name=f"clean_{f.tag}", traj=slow, duration_s=6.0, fault=f, noise_seed=None))
    print(f"  clean_{f.tag}", flush=True)
simulation_app.close()
