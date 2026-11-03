#!/usr/bin/env python3
"""Faults at 40% of the velocity limit (PAL_FAST_FRAC), below the 50% knee.

At 20% speed backlash and short command delays leave no signature in either channel (docs/REVIEW.md F10). Their effect grows
with speed, so this set asks whether those classes become findable when the arm moves faster. Same plant, noise and fault timing
as pal_o5_sweep.py, with the excitation trajectory at 40%; every faulty run shares noise seed 1000 with the healthy twin.

Why 40 and not 50: a first version at 50% had a healthy run peaking at 51.7% of the limit, and the healthy run raised the same joint-6
alarm, at the same moment (5.775 s), as every faulty run: apparent backlash and delay 'detections' were the healthy run's own false
alarm at the knee. A fault set has to sit below the knee for its detections to mean anything.

    tools/pal_pipeline.sh pal_o5_fast.py out/fast
"""
import os

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_o5_fast.py running", flush=True)

import pal_config as cfg
import pal_trajectories as T
from pal_faults import FaultSpec
from pal_plant import build_plant
from pal_runner import Runner, Scenario

HZ = float(os.environ.get("PAL_PHYSICS_HZ", cfg.PHYSICS_HZ))
plant = build_plant(HZ)
runner = Runner(plant, "/workspace/out/fast", mj_every=10)
traj = T.multisine(float(os.environ.get("PAL_FAST_FRAC", 0.40)))
specs = [FaultSpec("none"),
         FaultSpec("friction", 3, 2.0, 3.0), FaultSpec("friction", 3, 4.0, 3.0), FaultSpec("friction", 3, 8.0, 3.0),
         FaultSpec("backlash", 0, 0.02, 3.0), FaultSpec("backlash", 0, 0.12, 3.0), FaultSpec("backlash", 6, 0.12, 3.0),
         FaultSpec("delay", 0, 0.010, 3.0), FaultSpec("delay", 0, 0.030, 3.0), FaultSpec("stuck_joint", 3, 200.0, 3.0)]
for f in specs:
    log = runner.run(Scenario(name=f"fast_{f.tag}", traj=traj, traj_type="excitation", duration_s=6.0, fault=f, noise_seed=1000))
    print(f"  fast_{f.tag:<28} peak speed {log['speed_fraction']:.0%} of limit", flush=True)
simulation_app.close()
