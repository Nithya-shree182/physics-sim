#!/usr/bin/env python3
"""The analysis side's round 4 run list (docs/NOTES_FOR_DEV.md on nithya), everything at or below 50% speed.

  C2   excitation (multisine at 50% of the velocity limit), 0 kg and 1 kg, same trajectory and noise seed
  C4   the four requested faults from half way, 0 kg, slow reference: friction x3 on joint 3, friction x3 on
       joint 6, a locked joint 4, a 50 ms command delay. Each seed has its MATCHED fault-free twin (same
       trajectory, same noise realisation).
  C4b  the same four faults with a 1 kg payload held, and the 1 kg healthy twin
  C5   one fault-free run of 60 s
  C6   a payload picked up mid-run (0 -> 1 kg at 3 s)
  C9   seeds 2001 and 2002 of C4 (seed 1000 is the first set)

All in effort mode with the plant's 5% mass mismatch and seeded sensor noise. Joint numbers below are
1-based as the analysis side writes them; FaultSpec takes 0-based indices.

    tools/pal_pipeline.sh pal_extra_runs.py out/extra
"""
import os

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_extra_runs.py running", flush=True)

import pal_config as cfg
import pal_trajectories as T
from pal_faults import FaultSpec
from pal_plant import build_plant
from pal_runner import Runner, Scenario

HZ = float(os.environ.get("PAL_PHYSICS_HZ", cfg.PHYSICS_HZ))
ONSET = 3.0
plant = build_plant(HZ)
runner = Runner(plant, "/workspace/out/extra", mj_every=10)
slow = T.multisine(0.20)

runs = []
# C2: payload under acceleration, same trajectory and noise seed so the pair differs only by the load
for kg in (0.0, 1.0):
    runs.append(Scenario(name=f"c2_excitation_{kg:g}kg".replace(".", "p"), traj=T.multisine(0.50), traj_type="excitation",
                         duration_s=8.0, payload_kg=kg, noise_seed=3001))
# C4 / C4b / C9
faults = [FaultSpec("friction", 2, 3.0, ONSET), FaultSpec("friction", 5, 3.0, ONSET),
          FaultSpec("stuck_joint", 3, 200.0, ONSET), FaultSpec("delay", 0, 0.050, ONSET)]
for seed in (1000, 2001, 2002):
    runs.append(Scenario(name=f"c4_healthy_0kg_s{seed}", traj=slow, duration_s=6.0, noise_seed=seed))
    for f in faults:
        runs.append(Scenario(name=f"c4_{f.tag}_0kg_s{seed}", traj=slow, duration_s=6.0, fault=f, noise_seed=seed))
runs.append(Scenario(name="c4b_healthy_1kg_s1000", traj=slow, duration_s=6.0, payload_kg=1.0, noise_seed=1000))
for f in faults:
    runs.append(Scenario(name=f"c4b_{f.tag}_1kg_s1000", traj=slow, duration_s=6.0, payload_kg=1.0, fault=f, noise_seed=1000))
# C5: long fault-free run
runs.append(Scenario(name="c5_healthy_60s", traj=slow, duration_s=60.0, noise_seed=3000))
# C6: pickup mid-run
runs.append(Scenario(name="c6_pickup_0to1kg", traj=slow, duration_s=6.0, noise_seed=3002, payload_events=((3.0, 1.0),)))

print(f"\n  {len(runs)} runs", flush=True)
for sc in runs:
    log = runner.run(sc)
    print(f"  {sc.name:<40} peak speed {log['speed_fraction']:.0%} of limit", flush=True)
print("\n  done", flush=True)
simulation_app.close()
