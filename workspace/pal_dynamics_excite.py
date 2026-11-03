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


import xml.etree.ElementTree as ET

BODIES = list(art.body_names)
_root = ET.parse("/workspace/assets/panda_arm_hand.urdf").getroot()
URDF = {}
for _l in _root.findall("link"):
    _it = _l.find("inertial")
    if _it is None:
        continue
    _m, _o, _I = _it.find("mass"), _it.find("origin"), _it.find("inertia")
    URDF[_l.get("name")] = {
        "mass": float(_m.get("value")),
        "com": [float(x) for x in _o.get("xyz").split()] if _o is not None and _o.get("xyz") else [0.0]*3,
        "diag": [float(_I.get("ixx")), float(_I.get("iyy")), float(_I.get("izz"))],
    }


def author_plant():
    """Overwrite the asset's inertial parameters with the URDF's.

    Must be called AFTER the final world.reset(): a reset restores the articulation from
    USD and silently discards every tensor-API write — masses, COMs, inertias, friction
    and armature alike. That is what made the earlier run of this test meaningless.
    """
    nb = len(BODIES)
    cur = np.asarray(art.get_body_masses()).reshape(nb)
    m = np.array([URDF[b]["mass"] if b in URDF else float(cur[i])
                  for i, b in enumerate(BODIES)], dtype=np.float32)
    c = np.array([URDF[b]["com"] if b in URDF else [0.0]*3 for b in BODIES], dtype=np.float32)
    d = np.array([URDF[b]["diag"] if b in URDF else [1e-3]*3 for b in BODIES], dtype=np.float32)
    q = np.tile(np.array([1.0, 0, 0, 0], dtype=np.float32), (nb, 1))
    art.set_body_masses(m.reshape(1, nb))
    art.set_body_coms(c.reshape(1, nb, 3), q.reshape(1, nb, 4))
    try:
        art.set_body_inertias(d.reshape(1, nb, 3))
    except Exception:
        f = np.zeros((nb, 9), dtype=np.float32)
        f[:, 0], f[:, 4], f[:, 8] = d[:, 0], d[:, 1], d[:, 2]
        art.set_body_inertias(f.reshape(1, nb, 9))
    art.set_armatures(cfg.ARMATURE.reshape(1, N).astype(np.float32), joint_indices=idx)
    return m


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
AMP_RAW = np.array([0.55, 0.40, 0.55, 0.45, 0.70, 0.60, 0.70])
F0 = np.array([0.55, 0.45, 0.60, 0.50, 0.75, 0.70, 0.80])

# Peak velocity of a 3-harmonic multi-sine with amplitudes A/(h+1)^2 and frequencies
# F0*(h+1) is A * 2*pi*F0 * sum_h 1/(h+1).  Solve for A given a target peak velocity.
_K = sum(1.0 / (h + 1) for h in range(3))


def amps_for(frac):
    """Amplitudes whose peak commanded velocity is `frac` of each joint's datasheet limit."""
    return frac * cfg.QD_MAX / (2 * np.pi * F0 * _K)


AMP = amps_for(0.60)
PH = rng.uniform(0, 2 * np.pi, (3, N))
CENTER = cfg.HOME_JOINTS.copy()


def traj(t, amp=None):
    a_base = AMP if amp is None else amp
    q = CENTER.copy()
    qd = np.zeros(N)
    for h in range(3):
        w = 2 * np.pi * F0 * (h + 1)
        a = a_base / (h + 1) ** 2
        q += a * np.sin(w * t + PH[h])
        qd += a * w * np.cos(w * t + PH[h])
    return q, qd


def run(friction, label, steps=6000, warm=800):
    # Order matters: reset first, THEN write through the tensor API, then verify.
    world.reset()
    art.set_gains(kps=np.full((1, N), 400.0 * cfg.DEG_PER_RAD, dtype=np.float32),
                  kds=np.full((1, N), 80.0 * cfg.DEG_PER_RAD, dtype=np.float32),
                  joint_indices=idx)
    m_written = author_plant()
    set_friction(friction)
    q0, _ = traj(0.0)
    art.set_joint_positions(np.concatenate([q0, [0.04, 0.04]]))
    for _ in range(300):
        world.step(render=False)
    m_live = np.asarray(art.get_body_masses()).flatten()[:len(m_written)]
    f_live = np.asarray(art.get_friction_coefficients()).flatten()[:N]
    print(f"  [survive] masses {bool(np.allclose(m_live, m_written, atol=1e-4))}  "
          f"friction {np.round(f_live, 3)}", flush=True)

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


print("\n" + "=" * 96)
print("  VELOCITY-FRACTION SWEEP — is the residual model error, or limit saturation?")
print("=" * 96)
print("  The previous run commanded peak velocities up to 2.47x the datasheet limit.")
print("  A joint pinned at its velocity limit is held there by a CONSTRAINT force, which")
print("  appears in tau_measured and is nowhere in M qdd + C qd + g. If that is what the")
print("  residual was, it collapses as the commanded velocity drops below the limit.\n")
print(f"  {'frac':>6}{'cmd qd pk':>11}{'actual pk':>11}{'saturated?':>12}"
      f"{'RMS resid':>11}{'worst %lim':>12}{'verdict':>10}")

results = []
for frac in (0.20, 0.40, 0.60, 0.80, 1.00, 1.50):
    AMP = amps_for(frac)
    globals()["AMP"] = AMP
    R_raw, R_filt, P, QD, lbl = run(np.zeros(N), f"frac={frac}", steps=4000, warm=600)
    cmd_pk = float(np.max(frac * cfg.QD_MAX))
    act_pk = float(np.max(np.abs(QD)))
    # "Saturated" = any joint spent time within 1% of its own limit.
    sat = bool(np.any(np.max(np.abs(QD), axis=0) > 0.99 * cfg.QD_MAX))
    rms = np.sqrt(np.mean(R_filt ** 2, axis=0))
    worst = float(np.max(100 * rms / cfg.TAU_MAX))
    results.append((frac, cmd_pk, act_pk, sat, float(np.mean(rms)), worst))
    print(f"  {frac:>6.2f}{cmd_pk:>11.2f}{act_pk:>11.2f}{('YES' if sat else 'no'):>12}"
          f"{np.mean(rms):>11.4f}{worst:>11.2f}%{('PASS' if worst <= 5 else 'FAIL'):>10}")

print("\n  Per-joint detail at the largest frac that stays under the limits:")
clean = [r for r in results if not r[3]]
if clean:
    frac = clean[-1][0]
    globals()["AMP"] = amps_for(frac)
    R_raw, R_filt, P, QD, lbl = run(np.zeros(N), f"frac={frac}", steps=6000, warm=800)
    report(R_raw, R_filt, P, QD, f"frictionless, peak velocity = {frac:.0%} of limit")

    print("\n  Now with Gaz friction in the plant, model still frictionless:")
    fr = cfg.FRICTION_COULOMB_STATIC * cfg.PLANT_FRICTION_SCALE
    b = run(fr, "with friction", steps=6000, warm=800)
    rf = report(*b)
else:
    print("  every tested fraction saturated — lower the sweep range")

simulation_app.close()
