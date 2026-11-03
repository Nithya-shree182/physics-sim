#!/usr/bin/env python3
"""O5 sweep: arm FAULTS (not attacks) injected into the plant, in effort mode, with sensor noise.

One plant, many runs. Every run is 6 s with the fault switched on at 3 s, so each carries its own
healthy stretch (the analysis side's detector calibrates on the first 3 s). Speed is 20% of the
velocity limit, inside the <= 50% regime where the residual floor is physical.

  * healthy x6, different noise seeds: the false-alarm statistics need runs that differ
  * friction x2 / x4 / x8 (of the Gaz values, in N*m) on joints 2, 3, 4, 6
  * a locked joint 4, backlash 0.02 / 0.06 / 0.12 rad on joint 1 and 0.12 rad on joint 7, command delay 10 / 30 / 50 ms

The plant has the deliberate 5% arm-mass mismatch against the analysis side's URDF model, and
every logged sensor value carries seeded noise (pal_sensors.py).

Output: out/o5/*.npz + .json per run, out/o5/summary.json. Convert with tools/pal_pipeline.sh.

    tools/pal_pipeline.sh pal_o5_sweep.py out/o5
"""
import json
import os

import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_o5_sweep.py running", flush=True)

import pal_config as cfg
import pal_trajectories as T
from pal_faults import FaultSpec
from pal_plant import build_plant
from pal_runner import Runner, Scenario

HZ = float(os.environ.get("PAL_PHYSICS_HZ", cfg.PHYSICS_HZ))
DURATION_S = float(os.environ.get("PAL_DURATION_S", 6.0))
ONSET_S = float(os.environ.get("PAL_ONSET_S", 3.0))
OUT = "/workspace/out/o5"

plant = build_plant(HZ)
runner = Runner(plant, OUT, mj_every=10)
traj = T.multisine(0.20)

specs = [FaultSpec("none")] * 6
specs += [FaultSpec("friction", j, m, ONSET_S) for m in (2.0, 4.0, 8.0) for j in (1, 2, 3, 5)]
specs += [FaultSpec("stuck_joint", 3, 200.0, ONSET_S)]
# backlash only acts where the joint torque REVERSES sign; gravity-loaded joints 2-6 never reverse
# (measured: backlash on joints 4 and 6 changed nothing), so it goes on joints 1 and 7.
# Measured: at 20% speed backlash up to 0.02 rad withholds <1% of the torque and is invisible; the
# larger widths are there so the set spans undetectable to detectable.
specs += [FaultSpec("backlash", 0, w, ONSET_S) for w in (0.02, 0.06, 0.12)] + [FaultSpec("backlash", 6, 0.12, ONSET_S)]
specs += [FaultSpec("delay", 0, s, ONSET_S) for s in (0.010, 0.030, 0.050)]

print(f"\n  {len(specs)} runs | {DURATION_S:g}s each, fault at {ONSET_S:g}s | {HZ:.0f} Hz", flush=True)
print(f"  {'scenario':<28}{'|tau_cmd-net| pre':>19}{'post':>9}{'track rms pre':>15}{'post':>9}  joint", flush=True)
summary, n_healthy = [], 0
for i, f in enumerate(specs):
    if f.type == "none":
        name, seed = f"o5_healthy_s{n_healthy}", 1000 + n_healthy      # s0 shares seed 1000 with every faulty run
        n_healthy += 1
    else:
        name, seed = f"o5_{f.tag}", 1000
    # Matched baselines (analysis side's request, round 4 A2): healthy_s0 is the exact fault-free twin of
    # every faulty run (same trajectory, same noise realisation); s1..s5 differ, for the false-alarm spread.
    sc = Scenario(name=name, traj=traj, traj_type="slow_reference", duration_s=DURATION_S,
                  fault=f, noise_seed=seed, diagnostics=True)
    log = runner.run(sc)
    hz = int(HZ)
    pre, post = slice(int(1.0 * hz), int(ONSET_S * hz)), slice(int((ONSET_S + 0.5) * hz), int(DURATION_S * hz))
    r = np.abs(log["cmd"] - log["net"]); e = np.abs(log["q"] - log["q_des"])
    if f.type == "none":
        j = 3
    elif f.type == "delay":
        j = int(np.argmax(e[post].max(axis=0)))
    else:
        j = f.joint
    row = {"run_id": f"{name}_{hz}hz", "type": f.type, "joint": f.joint, "param": f.param,
           "noise_seed": seed, "joint_shown": j,
           "applied_minus_net_pre": float(r[pre, j].mean()), "applied_minus_net_post": float(r[post, j].mean()),
           "track_err_pre_rad": float(e[pre, j].max()), "track_err_post_rad": float(e[post, j].max()),
           "track_rms_pre_rad": float(np.sqrt((e[pre, j] ** 2).mean())), "track_rms_post_rad": float(np.sqrt((e[post, j] ** 2).mean())),
           "q_travel_post_rad": np.ptp(log["q"][post], axis=0).tolist()}
    summary.append(row)
    print(f"  {name:<28}{row['applied_minus_net_pre']:>19.3f}{row['applied_minus_net_post']:>9.3f}"
          f"{row['track_rms_pre_rad']:>15.4f}{row['track_rms_post_rad']:>9.4f}  j{j + 1}", flush=True)

with open(f"{OUT}/summary.json", "w") as fh:
    json.dump({"hz": HZ, "duration_s": DURATION_S, "onset_s": ONSET_S, "runs": summary}, fh, indent=2)
print(f"\n  written: out/o5/summary.json and {len(specs)} runs", flush=True)
simulation_app.close()
