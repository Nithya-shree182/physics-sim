#!/usr/bin/env python3
"""O1 feasibility for real: can a full telemetry frame be read EVERY physics step at 1 kHz,
and is the frame internally coherent?

The objective's pass condition is "no dropped frames across a full run". That is a
throughput claim and it has never been tested — the audit reads each quantity once.
"""
import time
import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation
import pal_config as cfg

N, HZ = 7, 1000.0
DT = 1.0 / HZ
world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=1/30.0)
add_reference_to_stage(usd_path=get_assets_root_path() + cfg.ROBOT_USD_SUBPATH,
                       prim_path=cfg.ROBOT_PRIM)
art = Articulation(cfg.ROBOT_PRIM)
world.reset(); art.initialize()
home = np.concatenate([cfg.HOME_JOINTS, [0.04, 0.04]])
art.set_joint_positions(home); art.set_joint_position_targets(home)
for _ in range(500):
    world.step(render=False)

STEPS = 10000          # 10 s of simulated time at 1 kHz


def full_frame():
    return (np.asarray(art.get_joint_positions()).flatten(),
            np.asarray(art.get_joint_velocities()).flatten(),
            np.asarray(art.get_generalized_gravity_forces()).flatten(),
            np.asarray(art.get_coriolis_and_centrifugal_forces()).flatten(),
            np.asarray(art.get_mass_matrices()).flatten(),
            np.asarray(art.get_jacobians()).flatten(),
            np.asarray(art.get_measured_joint_efforts()).flatten(),
            np.asarray(art.get_measured_joint_forces()).flatten())


print("\n" + "=" * 76)
print("  1 — THROUGHPUT: physics step only, no telemetry")
print("=" * 76)
t0 = time.perf_counter()
for _ in range(STEPS):
    world.step(render=False)
bare = time.perf_counter() - t0
print(f"  {STEPS} steps in {bare:.2f}s  ->  {STEPS/bare:,.0f} steps/s  "
      f"({STEPS/bare/HZ:.2f}x real time)")

print("\n" + "=" * 76)
print("  2 — THROUGHPUT: physics step + FULL telemetry frame every step")
print("=" * 76)
AMP = np.array([0.0, 0.25, 0.0, 0.25, 0.0, 0.25, 0.0])
t0 = time.perf_counter()
per_step = np.empty(STEPS)
frames = []
for k in range(STEPS):
    s0 = time.perf_counter()
    target = cfg.HOME_JOINTS + AMP * np.sin(2*np.pi*0.25*k*DT)
    art.set_joint_position_targets(np.concatenate([target, [0.04, 0.04]]))
    world.step(render=False)
    f = full_frame()
    per_step[k] = time.perf_counter() - s0
    if k % 1000 == 0:
        frames.append(f)
full = time.perf_counter() - t0
print(f"  {STEPS} steps in {full:.2f}s  ->  {STEPS/full:,.0f} steps/s  "
      f"({STEPS/full/HZ:.2f}x real time)")
print(f"  telemetry overhead: {100*(full-bare)/bare:.1f}% on top of bare physics")
print(f"  per-step: mean {1e3*per_step.mean():.3f} ms, "
      f"p99 {1e3*np.percentile(per_step,99):.3f} ms, max {1e3*per_step.max():.3f} ms")
print(f"  budget at {HZ:.0f} Hz is {1e3*DT:.3f} ms/step")
over = int(np.sum(per_step > DT))
print(f"  steps over budget: {over}/{STEPS} ({100*over/STEPS:.1f}%)")
print(f"  VERDICT: {'real-time capable' if STEPS/full >= HZ else 'SLOWER THAN REAL TIME'} "
      f"— headless batch generation is unaffected either way, live streaming is not")

print("\n" + "=" * 76)
print("  3 — FRAME SIZE / DATA VOLUME")
print("=" * 76)
q, qd, g, cor, M, J, tm, mf = frames[0]
cols = {"q": 7, "qd": 7, "g": 7, "coriolis": 7, "M (7x7)": 49, "J (6x7)": 42,
        "tau_measured": 7, "tau_applied": 7, "q_cmd": 7, "ee_pose": 7, "labels/time": 12}
total = sum(cols.values())
print(f"  {'group':<16}{'floats':>8}")
for k, v in cols.items():
    print(f"  {k:<16}{v:>8}")
print(f"  {'TOTAL':<16}{total:>8} floats/frame")
for name, nbytes in (("float64", 8), ("float32", 4)):
    bps = total * nbytes * HZ
    print(f"  {name}: {bps/1e6:.2f} MB/s  |  {bps*60/1e9:.2f} GB/min  |  "
          f"{bps*3600/1e9:.1f} GB/hour (uncompressed)")

print("\n" + "=" * 76)
print("  4 — FRAME COHERENCE: are all quantities from the SAME step?")
print("=" * 76)
# Read the same quantity twice without stepping: if it changes, a readback is not a
# snapshot. Then check whether reads taken before and after a step differ as expected.
a1 = np.asarray(art.get_joint_positions()).flatten().copy()
a2 = np.asarray(art.get_joint_positions()).flatten().copy()
print(f"  same-step repeat read identical: {np.allclose(a1, a2)}")

g_before = np.asarray(art.get_generalized_gravity_forces()).flatten()[:N].copy()
q_before = np.asarray(art.get_joint_positions()).flatten()[:N].copy()
world.step(render=False)
g_after = np.asarray(art.get_generalized_gravity_forces()).flatten()[:N].copy()
q_after = np.asarray(art.get_joint_positions()).flatten()[:N].copy()
print(f"  q changed across the step:    {not np.allclose(q_before, q_after)}")
print(f"  g(q) changed across the step: {not np.allclose(g_before, g_after)}")
print("  (both must be True: if g(q) were stale by one step it would lag q during fast")
print("   motion and inject a false residual that grows with speed)")

simulation_app.close()
