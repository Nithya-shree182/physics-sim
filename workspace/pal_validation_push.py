#!/usr/bin/env python3
"""Independent validation (proposal 7.7): push the gripper with a KNOWN force in low-gain torque mode.

The arm is held by a soft computed-torque controller (Kp = 40, Kd = 12, ten times softer than
normal) so it yields to the push and settles; a constant world-frame force is applied at the hand
for 1.0 s and the estimate is read in the settled window; the force estimated from
the residual is then compared with the one applied (tools/check_push.py does the comparison from
the written Parquet files, not from anything computed here).

18 runs with realistic friction: 3 poses x 3 axes x 2 magnitudes (10 N, 20 N), then 6 control runs
with friction zeroed. No sensor noise and no mass mismatch, so a
disagreement can only come from the residual mathematics, which is what this test is for.

    tools/pal_pipeline.sh pal_validation_push.py out/push
"""
import os

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_validation_push.py running", flush=True)

import numpy as np
import pal_config as cfg
import pal_trajectories as T
from pal_plant import build_plant
from pal_runner import Runner, Scenario, PushSpec

HZ = float(os.environ.get("PAL_PHYSICS_HZ", cfg.PHYSICS_HZ))
plant = build_plant(HZ)
runner = Runner(plant, "/workspace/out/push", mj_every=1)     # fresh J every step for this test
n = 0
for pose in ("home", "horizontal_reach", "folded"):
    for axis, e in (("x", (1, 0, 0)), ("y", (0, 1, 0)), ("z", (0, 0, 1))):
        for mag in (10.0, 20.0):
            F = tuple(mag * np.array(e, float))
            sc = Scenario(name=f"push_{pose}_{axis}_{mag:g}N", traj=T.hold(cfg.PROBE_POSES[pose]),
                          traj_type="push", duration_s=1.7, gains="low", mass_scale=1.0,
                          push=PushSpec(F, 0.4, 1.5))
            runner.run(sc)
            n += 1
            print(f"  {sc.name}", flush=True)

# CONTROL: the same pushes with the plant's friction zeroed. The model has no friction, so static
# friction holding a joint back biases the estimate low; with friction off the residual mathematics
# should be exact. This separates "the maths is wrong" from "friction limits the accuracy".
plant.base_friction = np.zeros_like(plant.base_friction)
for pose in ("home", "folded"):
    for axis, e in (("x", (1, 0, 0)), ("y", (0, 1, 0)), ("z", (0, 0, 1))):
        F = tuple(10.0 * np.array(e, float))
        sc = Scenario(name=f"push_nofric_{pose}_{axis}_10N", traj=T.hold(cfg.PROBE_POSES[pose]),
                      traj_type="push", duration_s=1.7, gains="low", mass_scale=1.0,
                      push=PushSpec(F, 0.4, 1.5))
        runner.run(sc)
        n += 1
        print(f"  {sc.name}", flush=True)
print(f"\n  written {n} runs to out/push", flush=True)
simulation_app.close()
