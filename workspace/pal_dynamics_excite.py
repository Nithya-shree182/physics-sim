#!/usr/bin/env python3
"""Exercise M(q) and C(q,qd) for real.

The earlier calibration run was 98% gravity, so it validated g(q) and little else. Here
the arm is driven by a multi-sine excitation trajectory on all seven joints — the same
family of signal used for dynamic identification — with amplitudes and frequencies chosen
to push joint velocities toward the datasheet limits. Then:

  1  frictionless plant: does the three-term identity close under real dynamics?
  2  raw vs filtered qdd: how much of the residual is finite-difference noise?
  3  friction added: how much of the residual is friction?
  4  random pose/velocity snapshots: term sizes across the workspace, not one trajectory
"""
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
rng = np.random.RandomState(7)

world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=1/30.0)
add_reference_to_stage(usd_path=get_assets_root_path() + cfg.ROBOT_USD_SUBPATH,
                       prim_path=cfg.ROBOT_PRIM)
art = Articulation(cfg.ROBOT_PRIM)
world.reset(); art.initialize()
idx = np.arange(N)

# Stiff enough to actually track a fast trajectory; gains are per radian here.
art.set_gains(kps=np.full((1, N), 400.0 * cfg.DEG_PER_RAD, dtype=np.float32),
              kds=np.full((1, N), 80.0 * cfg.DEG_PER_RAD, dtype=np.float32),
              joint_indices=idx)


def set_friction(vals):
    art.set_friction_coefficients(np.asarray(vals, dtype=np.float32).reshape(1, N),
                                  joint_indices=idx)


def read():
    q = np.asarray(art.get_joint_positions()).flatten()[:N]
    qd = np.asarray(art.get_joint_velocities()).flatten()[:N]
    g = np.asarray(art.get_generalized_gravity_forces()).flatten()[:N]
    cor = np.asarray(art.get_coriolis_and_centrifugal_forces()).flatten()[:N]
    M = np.asarray(art.get_mass_matrices()).reshape(9, 9)[:N, :N]
    tm = np.asarray(art.get_measured_joint_efforts()).flatten()[:N]
    return q, qd, g, cor, M, tm


# ── Excitation trajectory: 3-harmonic multi-sine per joint, random phases ─────────────
# Amplitudes sized against each joint's range, base frequency chosen so peak velocity
# lands near the datasheet limit rather than at a token fraction of it.
AMP = np.array([0.55, 0.40, 0.55, 0.45, 0.70, 0.60, 0.70])
F0 = np.array([0.55, 0.45, 0.60, 0.50, 0.75, 0.70, 0.80])
PH = rng.uniform(0, 2 * np.pi, (3, N))
CENTER = cfg.HOME_JOINTS.copy()


def traj(t):
    q = CENTER.copy()
    qd = np.zeros(N)
    for h in range(3):
        w = 2 * np.pi * F0 * (h + 1)
        a = AMP / (h + 1) ** 2
        q += a * np.sin(w * t + PH[h])
        qd += a * w * np.cos(w * t + PH[h])
    return q, qd


def run(friction, label, steps=6000, warm=800):
    set_friction(friction)
    world.reset()
    q0, _ = traj(0.0)
    art.set_joint_positions(np.concatenate([q0, [0.04, 0.04]]))
    for _ in range(300):
        world.step(render=False)

    qd_prev = np.zeros(N)
    qdd_filt = np.zeros(N)
    ALPHA = 0.05                       # first-order low pass on the acceleration estimate
    R_raw, R_filt, P, QD = [], [], [], []
    for k in range(steps):
        t = k * DT
        qt, _ = traj(t)
        art.set_joint_position_targets(np.concatenate([qt, [0.04, 0.04]]))
        world.step(render=False)
        q, qd, g, cor, M, tm = read()
        qdd = (qd - qd_prev) / DT
        qd_prev = qd.copy()
        qdd_filt = (1 - ALPHA) * qdd_filt + ALPHA * qdd
        if k < warm:
            continue
        R_raw.append(tm - (M @ qdd + cor + g))
        R_filt.append(tm - (M @ qdd_filt + cor + g))
        P.append((np.abs(M @ qdd), np.abs(cor), np.abs(g)))
        QD.append(qd.copy())
    return (np.array(R_raw), np.array(R_filt), np.array(P), np.array(QD), label)


def report(R_raw, R_filt, P, QD, label):
    acc = np.mean(np.array([p[0] for p in P]), axis=0)
    cor = np.mean(np.array([p[1] for p in P]), axis=0)
    gr = np.mean(np.array([p[2] for p in P]), axis=0)
    rms_r = np.sqrt(np.mean(R_raw ** 2, axis=0))
    rms_f = np.sqrt(np.mean(R_filt ** 2, axis=0))
    print(f"\n  {label}")
    print(f"  {'joint':<8}{'|qd|max':>9}{'|M qdd|':>10}{'|C qd|':>9}{'|g|':>9}"
          f"{'C as %':>8}{'RMS raw':>10}{'RMS filt':>10}{'% lim':>8}")
    for i in range(N):
        tot = acc[i] + cor[i] + gr[i]
        print(f"  j{i+1:<7}{np.max(np.abs(QD[:, i])):>9.2f}{acc[i]:>10.3f}{cor[i]:>9.3f}"
              f"{gr[i]:>9.3f}{100*cor[i]/max(tot,1e-9):>7.1f}%{rms_r[i]:>10.4f}"
              f"{rms_f[i]:>10.4f}{100*rms_f[i]/cfg.TAU_MAX[i]:>7.2f}%")
    print(f"  TOTALS   mean |M qdd| {np.mean(acc):.3f}   mean |C qd| {np.mean(cor):.3f}"
          f"   mean |g| {np.mean(gr):.3f}")
    worst = int(np.argmax(rms_f / cfg.TAU_MAX))
    pct = 100 * rms_f[worst] / cfg.TAU_MAX[worst]
    print(f"  worst joint {worst+1}: {pct:.2f}% of limit  ->  "
          f"{'PASS' if pct <= 5 else 'FAIL'} against the 5% tolerance")
    return rms_r, rms_f


print("\n" + "=" * 94)
print("  1 — EXCITATION TRAJECTORY, FRICTIONLESS PLANT   (pure M / C / g validation)")
print("=" * 94)
a = run(np.zeros(N), "multi-sine, 3 harmonics, all 7 joints, friction = 0")
rms_r0, rms_f0 = report(*a)
print(f"\n  raw vs filtered qdd: RMS residual {np.mean(rms_r0):.4f} -> {np.mean(rms_f0):.4f} N*m "
      f"({100*(1-np.mean(rms_f0)/np.mean(rms_r0)):.0f}% of it was finite-difference noise)")

print("\n" + "=" * 94)
print("  2 — SAME TRAJECTORY, GAZ FRICTION IN THE PLANT   (model still frictionless)")
print("=" * 94)
fr = cfg.FRICTION_COULOMB_STATIC * cfg.PLANT_FRICTION_SCALE
b = run(fr, f"friction = Gaz Coulomb x{cfg.PLANT_FRICTION_SCALE}")
rms_r1, rms_f1 = report(*b)
print(f"\n  friction applied (N*m): {np.round(fr, 3)}")
print(f"  residual growth from friction alone: {np.round(rms_f1 - rms_f0, 4)}")
print("  The model has no friction term, so this difference IS the friction signature —")
print("  the thing O5 must detect a CHANGE in, not the thing itself.")

print("\n" + "=" * 94)
print("  3 — RANDOM POSES AND VELOCITIES   (term sizes across the workspace)")
print("=" * 94)
set_friction(np.zeros(N))
print(f"  {'trial':>6}{'c(q)':>9}{'|qd| max':>10}{'|M qdd|':>10}{'|C qd|':>10}"
      f"{'|g|':>9}{'resid max':>11}")
mats = []
for trial in range(8):
    lo = np.maximum(cfg.Q_MIN + 0.3, cfg.HOME_JOINTS - 1.0)
    hi = np.minimum(cfg.Q_MAX - 0.3, cfg.HOME_JOINTS + 1.0)
    qr = rng.uniform(lo, hi)
    world.reset()
    art.set_joint_positions(np.concatenate([qr, [0.04, 0.04]]))
    for _ in range(200):
        world.step(render=False)
    # Drive hard toward a displaced target to build up real velocity, then sample.
    tgt = np.clip(qr + rng.uniform(-0.6, 0.6, N), cfg.Q_MIN + 0.2, cfg.Q_MAX - 0.2)
    art.set_joint_position_targets(np.concatenate([tgt, [0.04, 0.04]]))
    qd_prev = np.zeros(N)
    best = None
    for k in range(260):
        world.step(render=False)
        q, qd, g, cor, M, tm = read()
        qdd = (qd - qd_prev) / DT
        qd_prev = qd.copy()
        if k > 30 and (best is None or np.max(np.abs(qd)) > best[0]):
            best = (np.max(np.abs(qd)), M @ qdd, cor.copy(), g.copy(), tm - (M @ qdd + cor + g))
    J = np.asarray(art.get_jacobians())
    Jl = J.reshape(J.shape[0], -1, J.shape[-2], J.shape[-1])[0, cfg.EE_JACOBIAN_ROW][:, :N]
    sv = np.linalg.svd(Jl, compute_uv=False)
    mats.append(np.linalg.cond(np.asarray(art.get_mass_matrices()).reshape(9, 9)[:N, :N]))
    print(f"  {trial:>6}{sv.min()/sv.max():>9.4f}{best[0]:>10.2f}"
          f"{np.mean(np.abs(best[1])):>10.3f}{np.mean(np.abs(best[2])):>10.3f}"
          f"{np.mean(np.abs(best[3])):>9.3f}{np.max(np.abs(best[4])):>11.4f}")
print(f"\n  mass-matrix condition number across these poses: "
      f"min {min(mats):.1f}, max {max(mats):.1f}")
print("  (M is pose-dependent by a factor of several — a stretched arm really is harder")
print("   to swing than a folded one, which is the whole reason M(q) cannot be a constant)")

simulation_app.close()
