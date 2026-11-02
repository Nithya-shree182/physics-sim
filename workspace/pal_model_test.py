#!/usr/bin/env python3
"""Validate the dynamics model itself: tau_model = M(q) qdd + C(q,qd) qd + g(q).

Three tests, each one able to fail independently:

  A  Payload sweep at rest. With qd = qdd = 0 the model reduces to g(q), so the measured
     torque must equal the gravity term, and the CHANGE caused by a payload must be
     linear in its mass.

  B  Cross-check g(q) against J(q). A point mass m at the tool centre contributes
     tau = J_v^T (0, 0, -m*g) to the gravity term. The simulator reports the gravity
     vector and the Jacobian through two separate code paths, so if they agree the
     model's two main ingredients are confirmed against each other rather than against
     an assumption.

  C  Moving test. Drive a real trajectory and evaluate the full three-term model against
     the measured torque, with qdd from a finite difference of velocity. This is the
     section 7.2 calibration test, and the only one of the three in which M and C
     participate at all.
"""
import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from pxr import UsdPhysics
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation
import pal_config as cfg

N = 7
HZ = 1000.0
DT = 1.0 / HZ

world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=1/30.0)
add_reference_to_stage(usd_path=get_assets_root_path() + cfg.ROBOT_USD_SUBPATH,
                       prim_path=cfg.ROBOT_PRIM)
art = Articulation(cfg.ROBOT_PRIM)
world.reset()
art.initialize()

home = np.concatenate([cfg.HOME_JOINTS, [0.04, 0.04]])
hand = world.stage.GetPrimAtPath(f"{cfg.ROBOT_PRIM}/panda_hand")
mass_api = UsdPhysics.MassAPI.Apply(hand)


def set_payload(kg):
    """A payload rigidly held by the gripper is, dynamically, extra mass on the hand."""
    mass_api.CreateMassAttr(cfg.HAND_MASS + float(kg))


def hold(steps=800):
    art.set_joint_positions(home)
    art.set_joint_position_targets(home)
    for _ in range(steps):
        world.step(render=False)


def read():
    q = np.asarray(art.get_joint_positions()).flatten()[:N]
    qd = np.asarray(art.get_joint_velocities()).flatten()[:N]
    g = np.asarray(art.get_generalized_gravity_forces()).flatten()[:N]
    cor = np.asarray(art.get_coriolis_and_centrifugal_forces()).flatten()[:N]
    M = np.asarray(art.get_mass_matrices()).reshape(9, 9)[:N, :N]
    tm = np.asarray(art.get_measured_joint_efforts()).flatten()[:N]
    J = np.asarray(art.get_jacobians())
    return q, qd, g, cor, M, tm, J


print("\n" + "=" * 78)
print("  A — PAYLOAD SWEEP AT REST   (qd = qdd = 0, so tau_model reduces to g(q))")
print("=" * 78)
LADDER = [0.0, 0.5, 1.0, 2.0, 3.0, 5.0]
rows, base_g = [], None
print(f"  {'payload':>8}{'g j2':>10}{'g j4':>10}{'g j6':>10}"
      f"{'tau_m j4':>11}{'|resid| max':>13}{'d_tau j4':>10}")
for m in LADDER:
    set_payload(m)
    world.reset()
    hold()
    q, qd, g, cor, M, tm, J = read()
    if base_g is None:
        base_g = g.copy()
    resid = tm - g                      # at rest the whole model IS g(q)
    rows.append((m, g.copy(), tm.copy(), resid.copy()))
    print(f"  {m:>7.1f}kg{g[1]:>10.3f}{g[3]:>10.3f}{g[5]:>10.3f}"
          f"{tm[3]:>11.3f}{np.max(np.abs(resid)):>13.5f}{g[3]-base_g[3]:>10.3f}")

print("\n  Linearity of the gravity term in payload mass (joint 4):")
masses = np.array([r[0] for r in rows])
dtau4 = np.array([r[1][3] - base_g[3] for r in rows])
slope = np.polyfit(masses, dtau4, 1)[0]
pred = slope * masses
print(f"    d_tau4 = {slope:.4f} N*m per kg")
print(f"    max departure from a straight line: "
      f"{np.max(np.abs(dtau4 - (pred + np.polyfit(masses, dtau4, 1)[1]))):.5f} N*m")
print(f"    => effective lever arm at the TCP = {slope/9.81:.4f} m")

print("\n" + "=" * 78)
print("  B — CROSS-CHECK g(q) AGAINST J(q)   (two independent simulator outputs)")
print("=" * 78)
set_payload(0.0)
world.reset(); hold()
_, _, g0, _, _, _, J0 = read()
set_payload(3.0)
world.reset(); hold()
_, _, g3, _, _, _, J3 = read()

Jm = J3.reshape(J3.shape[0], -1, J3.shape[-2], J3.shape[-1])[0, -1]   # last link
Jv = Jm[:3, :N]                                                       # linear rows
weight = np.array([0.0, 0.0, -3.0 * 9.81])                            # 3 kg, downward
tau_from_J = -Jv.T @ weight
d_g = g3 - g0

print(f"  {'joint':<9}{'d g(q)':>12}{'J_v^T W':>12}{'difference':>12}")
for i in range(N):
    print(f"  joint{i+1:<4}{d_g[i]:>12.4f}{tau_from_J[i]:>12.4f}{d_g[i]-tau_from_J[i]:>12.4f}")
err = np.max(np.abs(d_g - tau_from_J))
print(f"\n  max difference {err:.4f} N*m  "
      f"({100*err/np.max(np.abs(d_g)):.2f}% of the largest term)")
print(f"  {'AGREE — g(q) and J(q) are mutually consistent' if err < 0.5 else 'DISAGREE — investigate the Jacobian link index / TCP offset'}")

print("\n" + "=" * 78)
print("  C — MOVING TEST   (the full three-term model; section 7.2 calibration)")
print("=" * 78)
set_payload(0.0)
world.reset()
art.set_joint_positions(home)
for _ in range(500):
    world.step(render=False)

# A slow smooth sinusoid on joints 2, 4 and 6 — the joints that carry gravity load here.
AMP = np.array([0.0, 0.25, 0.0, 0.25, 0.0, 0.25, 0.0])
FREQ = 0.25                                         # Hz; slow, per the proposal's 7.2
STEPS = int(4.0 * HZ)                               # one full cycle
qd_prev = np.zeros(N)
res_hist, tm_hist, parts = [], [], []

for k in range(STEPS):
    t = k * DT
    target = cfg.HOME_JOINTS + AMP * np.sin(2 * np.pi * FREQ * t)
    art.set_joint_position_targets(np.concatenate([target, [0.04, 0.04]]))
    world.step(render=False)

    q, qd, g, cor, M, tm, J = read()
    qdd = (qd - qd_prev) / DT                       # raw finite difference
    qd_prev = qd.copy()
    if k < int(0.5 * HZ):                           # skip the start transient
        continue
    tau_model = M @ qdd + cor + g
    res_hist.append(tm - tau_model)
    tm_hist.append(tm)
    parts.append((np.abs(M @ qdd), np.abs(cor), np.abs(g)))

res = np.array(res_hist)
rms = np.sqrt(np.mean(res ** 2, axis=0))
pk = np.max(np.abs(res), axis=0)
acc_p, cor_p, g_p = (np.mean(np.array([p[i] for p in parts]), axis=0) for i in range(3))

print(f"  trajectory: {FREQ} Hz sinusoid, +/-{AMP[1]} rad on joints 2/4/6, "
      f"{len(res)} samples at {HZ:.0f} Hz\n")
print(f"  {'joint':<9}{'RMS resid':>11}{'peak':>9}{'% limit':>9}"
      f"{'|M qdd|':>10}{'|C qd|':>9}{'|g|':>9}")
for i in range(N):
    print(f"  joint{i+1:<4}{rms[i]:>11.4f}{pk[i]:>9.4f}"
          f"{100*rms[i]/cfg.TAU_MAX[i]:>8.2f}%{acc_p[i]:>10.4f}{cor_p[i]:>9.4f}{g_p[i]:>9.3f}")

worst = int(np.argmax(rms / cfg.TAU_MAX))
pct = 100 * rms[worst] / cfg.TAU_MAX[worst]
print(f"\n  worst joint {worst+1} at {pct:.2f}% of its {cfg.TAU_MAX[worst]:.0f} N*m limit")
print(f"  tolerance {100*cfg.CALIBRATION_TOLERANCE:.0f}% (section 7.2)  ->  "
      f"{'PASS' if pct <= 100*cfg.CALIBRATION_TOLERANCE else 'FAIL'}")
print(f"\n  Term sizes averaged over the run — this says which terms the test exercised:")
print(f"    gravity      {np.mean(g_p):.4f} N*m")
print(f"    acceleration {np.mean(acc_p):.4f} N*m")
print(f"    Coriolis     {np.mean(cor_p):.4f} N*m")

simulation_app.close()
