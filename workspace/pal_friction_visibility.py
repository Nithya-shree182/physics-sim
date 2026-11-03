#!/usr/bin/env python3
"""Can the residual see injected joint friction? Position mode vs effort mode, same trajectory.

Found while reviewing the O5 sweep: in POSITION mode r = tau_measured - (M qdd + C + g) stays
~0.001 N*m with friction x8 or a locked joint, because tau_measured behaves as the NET joint
torque (Newton's law holds for it whatever produced it). This probe asks the same question in
EFFORT mode, where the torque we apply is known: does `tau_cmd - (M qdd + C + g)` or
`tau_measured - (...)` show the friction, and how big is it compared with the friction injected?

Joint 1 (index 1) friction is raised x1 -> x8 mid-run; everything is measured before/after.

    tools/run_sim.sh pal_friction_visibility.py
"""
import json
import os

import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_friction_visibility.py running", flush=True)

import pal_config as cfg
from pal_plant import build_plant

ARM, J, MULT = 7, 1, 8.0
HZ = float(os.environ.get("PAL_PHYSICS_HZ", 1000.0))
plant = build_plant(HZ)
world, art, idx, fric = plant.world, plant.art, plant.idx, plant.fric
if os.environ.get("PAL_NO_FRICTION") == "1" or os.environ.get("PAL_FRICTION_API") == "effort":
    fric = np.zeros(ARM)
    art.set_friction_coefficients(fric.reshape(1, ARM).astype(np.float32), joint_indices=idx)
    print("[PAL] plant friction ZEROED for this run", flush=True)
EFFORT_API = os.environ.get("PAL_FRICTION_API") == "effort"
pv = art._physics_view


def set_joint_friction_nm(j, nm):
    """Torque-valued Coulomb friction on one joint via the new DOF friction-properties API."""
    props = np.asarray(pv.get_dof_friction_properties()).copy()      # (1, 9, 3): static, dynamic, viscous
    props[0, j, 0] = props[0, j, 1] = nm
    pv.set_dof_friction_properties(props.astype(np.float32), np.array([0], dtype=np.int32))


art.set_gains(kps=np.zeros((1, ARM), np.float32), kds=np.zeros((1, ARM), np.float32), joint_indices=idx)
art.switch_control_mode("effort", joint_indices=idx)

F0 = np.array([.55, .45, .60, .50, .75, .70, .80]); K = sum(1. / (h + 1) for h in range(3))
A = 0.20 * cfg.QD_MAX / (2 * np.pi * F0 * K)
KP_FB, KD_FB = 400.0, 40.0
vec = lambda x, n=ARM: np.asarray(x).flatten()[:n]


def tr(t):
    q = cfg.HOME_JOINTS.copy(); qd = np.zeros(ARM); qdd = np.zeros(ARM)
    for h in range(3):
        w = 2 * np.pi * F0 * (h + 1); a = A / (h + 1) ** 2
        q += a * np.sin(w * t); qd += a * w * np.cos(w * t); qdd -= a * w * w * np.sin(w * t)
    return q, qd, qdd


art.set_joint_positions(np.concatenate([tr(0)[0], [.04, .04]]))
art.set_joint_velocities(np.concatenate([tr(0)[1], [0., 0.]]).reshape(1, -1))
for _ in range(int(0.2 * HZ)):
    world.step(render=False)

N = int(6.0 * HZ); onset = int(3.0 * HZ)
rows = {k: [] for k in ("net", "cmd", "tm", "ta")}
tau = np.zeros(9, np.float32); qprev = None
for k in range(N):
    if k == onset and EFFORT_API:
        set_joint_friction_nm(J, 1.4)
    elif k == onset:
        f = fric.copy(); f[J] = max(fric[J], 0.175) * MULT
        art.set_friction_coefficients(f.reshape(1, ARM).astype(np.float32), joint_indices=idx)
    q, qd = vec(art.get_joint_positions()), vec(art.get_joint_velocities())
    g, c = vec(art.get_generalized_gravity_forces()), vec(art.get_coriolis_and_centrifugal_forces())
    M = np.asarray(art.get_mass_matrices()).reshape(9, 9)[:ARM, :ARM]
    qs, qds, qdds = tr(k / HZ)
    cmd = M @ (qdds + KP_FB * (qs - q) + KD_FB * (qds - qd)) + c + g
    tau[:ARM] = cmd
    art.set_joint_efforts(tau.reshape(1, 9))
    world.step(render=False)
    qd1 = vec(art.get_joint_velocities()); qdd = (qd1 - qd) / (1.0 / HZ)
    g1, c1 = vec(art.get_generalized_gravity_forces()), vec(art.get_coriolis_and_centrifugal_forces())
    M1 = np.asarray(art.get_mass_matrices()).reshape(9, 9)[:ARM, :ARM]
    rows["net"].append(M1 @ qdd + c1 + g1); rows["cmd"].append(cmd)
    rows["tm"].append(vec(art.get_measured_joint_efforts()))
    rows["ta"].append(vec(art.get_applied_joint_efforts()))
R = {k: np.array(v) for k, v in rows.items()}
pre, post = slice(int(1.0 * HZ), onset), slice(onset + int(0.5 * HZ), N)
inj = 1.4 if EFFORT_API else float(max(fric[J], 0.175) * MULT - fric[J])
print(f"\n  joint {J}: injected extra Coulomb friction {inj:.3f} N*m (coefficient set to {max(fric[J], 0.175) * MULT:.3f}, was {fric[J]:.3f})")
print(f"  {'quantity (joint %d)' % J:<44}{'pre':>10}{'post':>10}{'change':>10}")
out = {"joint": J, "injected_extra_nm": inj}
for name, x in (("tau_measured - net", R["tm"] - R["net"]), ("tau_cmd - net", R["cmd"] - R["net"]),
                ("tau_applied - net", R["ta"] - R["net"]), ("tau_cmd - tau_measured", R["cmd"] - R["tm"])):
    a, b = np.abs(x[pre, J]).mean(), np.abs(x[post, J]).mean()
    print(f"  mean |{name}|".ljust(46) + f"{a:>10.4f}{b:>10.4f}{b - a:>+10.4f}")
    out[name] = {"pre": float(a), "post": float(b)}
os.makedirs("/workspace/out", exist_ok=True)
json.dump(out, open("/workspace/out/friction_visibility.json", "w"), indent=2)
simulation_app.close()
