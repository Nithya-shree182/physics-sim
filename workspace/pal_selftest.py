#!/usr/bin/env python3
"""Self-test of the plant. Run after ANY change to pal_plant.py, pal_runner.py or pal_faults.py.

    tools/run_sim.sh pal_selftest.py      # prints SELFTEST RESULT: PASS or FAIL

 T1  zero friction: the commanded torque equals M qdd + C + g (the identity closes)
 T2  friction really is in N*m: injected static = dynamic = 1.0 N*m on joints 2, 4, 6 appears as
     ~1.0 N*m of (tau_cmd - net) in the direction of motion, within 10%
 T3  inertials rewritten mid-run: gravity with a 3 kg payload and the arm mass mismatch matches an
     independent numpy-from-URDF calculation (pal_kinematics), within 0.15 N*m
 T4  a pushed force is recovered from the applied torque: F_est = -(J^T)^+ (tau_cmd - net)
     for 10 N along x, within 15%

These are the checks that would have caught the 2026-11-03 friction-units bug (docs/REVIEW.md F1).
"""
import sys

import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_selftest.py running", flush=True)

import pal_config as cfg
import pal_trajectories as T
from pal_plant import build_plant, DEFAULT_MASS_SCALE
from pal_runner import Runner, Scenario, PushSpec
from pal_kinematics import Urdf

ARM = 7
HZ = 1000.0
plant = build_plant(HZ)
runner = Runner(plant, "/workspace/out", mj_every=10)
results = []


def check(name, ok, detail):
    results.append(ok)
    print(f"  {name:<4}{'PASS' if ok else 'FAIL'}   {detail}", flush=True)


def run_net(traj, duration, **kw):
    sc = Scenario(name="selftest", traj=traj, duration_s=duration, diagnostics=True,
                  mass_scale=DEFAULT_MASS_SCALE, **kw)
    return runner.run(sc, write=False)


# T1 -----------------------------------------------------------------------------------------
plant.set_friction(np.zeros((ARM, 3)))
orig = plant.base_friction.copy(); plant.base_friction = np.zeros((ARM, 3))
log = run_net(T.multisine(0.20), 2.0)
sel = slice(500, None)
err = np.abs(log["cmd"][sel] - log["net"][sel]).mean(axis=0)
check("T1", bool((err < 0.05).all()),
      f"zero friction: mean |tau_cmd - net| per joint [N*m] {np.round(err, 4).tolist()} (limit 0.05)")

# T2 -----------------------------------------------------------------------------------------
for j in (1, 3, 5):
    f = np.zeros((ARM, 3)); f[j, 0] = f[j, 1] = 1.0
    plant.base_friction = f
    log = run_net(T.multisine(0.30), 3.0)
    qd, d = log["qd"][500:, j], (log["cmd"] - log["net"])[500:, j]
    mv = np.abs(qd) > 0.05
    est = float(np.mean(np.sign(qd[mv]) * d[mv]))
    check("T2", abs(est - 1.0) < 0.10, f"joint {j + 1}: injected 1.000 N*m, measured {est:.3f} N*m "
          f"(over {int(mv.sum())} moving samples)")
plant.base_friction = orig

# T3 -----------------------------------------------------------------------------------------
home = T.hold(cfg.HOME_JOINTS)
for kg in (0.0, 3.0):
    sc = Scenario(name="selftest", traj=home, traj_type="hold", duration_s=1.5, payload_kg=kg,
                  mass_scale=DEFAULT_MASS_SCALE, diagnostics=True)
    runner.run(sc, write=False)
    q = np.asarray(plant.art.get_joint_positions()).flatten()[:ARM]
    g_sim = np.asarray(plant.art.get_generalized_gravity_forces()).flatten()[:ARM]
    g_ref = Urdf(cfg.URDF_PATH, mass_scale=DEFAULT_MASS_SCALE, payload_kg=kg).gravity(q, 0.04)
    e = np.abs(g_sim - g_ref).max()
    check("T3", e < 0.15, f"payload {kg:g} kg, arm mass scale {DEFAULT_MASS_SCALE}: worst |g_sim - g_urdf| = {e:.4f} N*m")

# T4 -----------------------------------------------------------------------------------------
F = np.array([10.0, 0.0, 0.0])
sc = Scenario(name="selftest", traj=home, traj_type="push", duration_s=1.7, gains="low",
              mass_scale=DEFAULT_MASS_SCALE, diagnostics=True,
              push=PushSpec(tuple(F), 0.4, 1.5))
log = runner.run(sc, write=False)
est = []
for k in range(1000, 1450):
    r = log["cmd"][k] - log["net"][k]
    est.append(-np.linalg.pinv(log["J"][k].T) @ r)
est = np.mean(est, axis=0)
check("T4", abs(est[0] - 10.0) < 1.5 and abs(est[1]) < 1.5 and abs(est[2]) < 1.5,
      f"pushed {F.tolist()} N, estimated force {np.round(est[:3], 2).tolist()} N (torques {np.round(est[3:], 2).tolist()})")

print(f"\nSELFTEST RESULT: {'PASS' if all(results) else 'FAIL'}  ({sum(results)}/{len(results)})", flush=True)
simulation_app.close()
