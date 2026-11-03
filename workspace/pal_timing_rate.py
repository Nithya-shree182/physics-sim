#!/usr/bin/env python3
"""Two questions in one run.

P1  Is the fast-motion residual a one-step misalignment between tau_measured and the
    state used to evaluate the model? We read everything after world.step(), but PhysX
    may report the effort for the step just completed while reporting state after it.
    Test directly: evaluate the model with PRE-step state and with POST-step state and
    see which matches the measured torque.

P4  What physics rate can actually carry a full telemetry frame in real time? 1 kHz runs
    at 0.42x. Measure several rates and pick the fastest that keeps up.
"""
import time
import xml.etree.ElementTree as ET

import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_timing_rate.py running", flush=True)

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation
import pal_config as cfg

N = 7
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
        "diag": [float(_I.get("ixx")), float(_I.get("iyy")), float(_I.get("izz"))]}


def build(hz):
    global world, art, idx
    world = World(stage_units_in_meters=1.0, physics_dt=1.0/hz, rendering_dt=1/30.0)
    add_reference_to_stage(usd_path=get_assets_root_path() + cfg.ROBOT_USD_SUBPATH,
                           prim_path=cfg.ROBOT_PRIM)
    art = Articulation(cfg.ROBOT_PRIM)
    world.reset(); art.initialize()
    idx = np.arange(N)
    bodies = list(art.body_names); nb = len(bodies)
    art.set_gains(kps=np.full((1, N), 400.0*cfg.DEG_PER_RAD, dtype=np.float32),
                  kds=np.full((1, N), 80.0*cfg.DEG_PER_RAD, dtype=np.float32), joint_indices=idx)
    cur = np.asarray(art.get_body_masses()).reshape(nb)
    m = np.array([URDF[b]["mass"] if b in URDF else float(cur[i]) for i, b in enumerate(bodies)], np.float32)
    c = np.array([URDF[b]["com"] if b in URDF else [0.]*3 for b in bodies], np.float32)
    d = np.array([URDF[b]["diag"] if b in URDF else [1e-3]*3 for b in bodies], np.float32)
    q = np.tile(np.array([1., 0, 0, 0], np.float32), (nb, 1))
    art.set_body_masses(m.reshape(1, nb))
    art.set_body_coms(c.reshape(1, nb, 3), q.reshape(1, nb, 4))
    try:
        art.set_body_inertias(d.reshape(1, nb, 3))
    except Exception:
        f = np.zeros((nb, 9), np.float32); f[:, 0], f[:, 4], f[:, 8] = d[:, 0], d[:, 1], d[:, 2]
        art.set_body_inertias(f.reshape(1, nb, 9))
    art.set_armatures(cfg.ARMATURE.reshape(1, N).astype(np.float32), joint_indices=idx)
    art.set_friction_coefficients(np.zeros((1, N), np.float32), joint_indices=idx)


def snap():
    return (np.asarray(art.get_joint_positions()).flatten()[:N].copy(),
            np.asarray(art.get_joint_velocities()).flatten()[:N].copy(),
            np.asarray(art.get_generalized_gravity_forces()).flatten()[:N].copy(),
            np.asarray(art.get_coriolis_and_centrifugal_forces()).flatten()[:N].copy(),
            np.asarray(art.get_mass_matrices()).reshape(9, 9)[:N, :N].copy(),
            np.asarray(art.get_measured_joint_efforts()).flatten()[:N].copy())


print("\n" + "=" * 86)
print("  P1 — PRE-STEP vs POST-STEP state   (is the residual a one-step misalignment?)")
print("=" * 86)
HZ = 1000.0; DT = 1/HZ
build(HZ)
F0 = np.array([0.55, 0.45, 0.60, 0.50, 0.75, 0.70, 0.80])
K = sum(1./(h+1) for h in range(3))
AMP = 1.00 * cfg.QD_MAX / (2*np.pi*F0*K)      # peak commanded qd = 100% of limit


def traj(t):
    q = cfg.HOME_JOINTS.copy()
    for h in range(3):
        q += (AMP/(h+1)**2) * np.sin(2*np.pi*F0*(h+1)*t)
    return q


art.set_joint_positions(np.concatenate([traj(0.), [0.04, 0.04]]))
for _ in range(400):
    world.step(render=False)

res_post, res_pre, res_mid = [], [], []
qd_hist = []
prev = snap()
for k in range(4000):
    t = k*DT
    art.set_joint_position_targets(np.concatenate([traj(t), [0.04, 0.04]]).reshape(1, -1))
    world.step(render=False)
    cur = snap()
    q0, qd0, g0, c0, M0, tm0 = prev
    q1, qd1, g1, c1, M1, tm1 = cur
    qdd = (qd1 - qd0) / DT
    if k > 500:
        # tau_measured read AFTER the step, model from state AFTER the step
        res_post.append(tm1 - (M1 @ qdd + c1 + g1))
        # ... model from state BEFORE the step
        res_pre.append(tm1 - (M0 @ qdd + c0 + g0))
        # ... model from the midpoint, which is where a central difference belongs
        res_mid.append(tm1 - (0.5*(M0+M1) @ qdd + 0.5*(c0+c1) + 0.5*(g0+g1)))
        qd_hist.append(qd1)
    prev = cur

for lbl, R in (("post-step state", res_post), ("pre-step state", res_pre),
               ("midpoint state", res_mid)):
    R = np.array(R)
    rms = np.sqrt(np.mean(R**2, axis=0))
    print(f"  {lbl:<18} RMS {np.round(rms, 4)}  worst "
          f"{np.max(100*rms/cfg.TAU_MAX):.2f}% of limit")

R = np.array(res_post); qd_a = np.array(qd_hist)
print("\n  Is the residual proportional to velocity? (fit resid = a*qd + b per joint)")
print(f"  {'joint':<8}{'slope a':>10}{'N*m*s/rad':>12}{'R^2':>8}")
for i in range(N):
    A = np.vstack([qd_a[:, i], np.ones(len(qd_a))]).T
    coef, *_ = np.linalg.lstsq(A, R[:, i], rcond=None)
    pred = A @ coef
    ss = 1 - np.sum((R[:, i]-pred)**2)/max(np.sum((R[:, i]-np.mean(R[:, i]))**2), 1e-12)
    print(f"  j{i+1:<7}{coef[0]:>10.4f}{'':>12}{ss:>8.3f}")
print("  A high R^2 with a consistent negative slope means a velocity-proportional term")
print("  in the plant that the model lacks — i.e. damping, not a timing error.")

print("\n" + "=" * 86)
print("  P4 — PHYSICS RATE vs TELEMETRY THROUGHPUT")
print("=" * 86)
print(f"  {'rate':>8}{'steps/s':>10}{'x real time':>13}{'ms/step':>10}{'budget ms':>11}  verdict")
for hz in (1000, 500, 400, 250, 200, 120):
    build(float(hz))
    art.set_joint_positions(np.concatenate([cfg.HOME_JOINTS, [0.04, 0.04]]))
    for _ in range(100):
        world.step(render=False)
    n = 3000
    t0 = time.perf_counter()
    for _ in range(n):
        world.step(render=False)
        snap()
        np.asarray(art.get_jacobians())
    el = time.perf_counter() - t0
    sps = n/el
    print(f"  {hz:>7}Hz{sps:>10.0f}{sps/hz:>13.2f}{1e3*el/n:>10.3f}{1e3/hz:>11.3f}"
          f"  {'real time' if sps >= hz else 'too slow'}")

simulation_app.close()
