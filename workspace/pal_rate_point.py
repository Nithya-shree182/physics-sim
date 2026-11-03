#!/usr/bin/env python3
"""One rate, one process, one build — a clean point on the residual-vs-dt curve.

Why this exists: the rate sweep in pal_residual_diag.py built a new World per rate inside
a single process, and its 1000 Hz figure (2.78%) disagrees by 10x with two independent
single-build runs at the same rate and trajectory (pal_timing_rate 27.98%,
pal_fast_random 27.79%). Repeatedly constructing World and re-referencing the same prim
path in one Isaac process is the obvious suspect, so the sweep is redone with one
container per rate and nothing reused.

    PAL_PHYSICS_HZ=1000 PAL_FRAC=1.0 tools/run_sim.sh pal_rate_point.py
"""
import json
import os
import xml.etree.ElementTree as ET

import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_rate_point.py running", flush=True)

from pxr import PhysxSchema
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation
import pal_config as cfg

ARM = 7
HZ = float(os.environ.get("PAL_PHYSICS_HZ", 1000.0))
FRAC = float(os.environ.get("PAL_FRAC", 1.0))
KP = float(os.environ.get("PAL_KP", 400.0))
DT = 1.0 / HZ
SECONDS = 3.0                                  # same simulated duration at every rate
STEPS = int(SECONDS * HZ)
WARM = int(0.6 * HZ)

world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=1/30.0)
pc = world.get_physics_context()
pc.set_solver_type("TGS"); pc.enable_stablization(False); pc.set_gravity(cfg.GRAVITY)
sp = PhysxSchema.PhysxSceneAPI.Apply(pc.get_current_physics_scene_prim())
sp.CreateMinPositionIterationCountAttr(cfg.POSITION_ITERATIONS)
sp.CreateMinVelocityIterationCountAttr(cfg.VELOCITY_ITERATIONS)
add_reference_to_stage(usd_path=get_assets_root_path()+cfg.ROBOT_USD_SUBPATH, prim_path=cfg.ROBOT_PRIM)
art = Articulation(cfg.ROBOT_PRIM); world.reset(); art.initialize()
idx = np.arange(ARM)

root = ET.parse(cfg.URDF_PATH).getroot(); U = {}
for l in root.findall("link"):
    it = l.find("inertial")
    if it is None: continue
    m, o, I = it.find("mass"), it.find("origin"), it.find("inertia")
    U[l.get("name")] = {"mass": float(m.get("value")),
        "com": [float(x) for x in o.get("xyz").split()] if o is not None and o.get("xyz") else [0.]*3,
        "diag": [float(I.get("ixx")), float(I.get("iyy")), float(I.get("izz"))]}
bodies = list(art.body_names); nb = len(bodies)
cur = np.asarray(art.get_body_masses()).reshape(nb)
mm = np.array([U[b]["mass"] if b in U else float(cur[i]) for i, b in enumerate(bodies)], np.float32)
cc = np.array([U[b]["com"] if b in U else [0.]*3 for b in bodies], np.float32)
dd = np.array([U[b]["diag"] if b in U else [1e-3]*3 for b in bodies], np.float32)
qq = np.tile(np.array([1., 0, 0, 0], np.float32), (nb, 1))
art.set_gains(kps=np.full((1, ARM), KP*cfg.DEG_PER_RAD, np.float32),
              kds=np.full((1, ARM), 80.0*cfg.DEG_PER_RAD, np.float32), joint_indices=idx)
art.set_body_masses(mm.reshape(1, nb)); art.set_body_coms(cc.reshape(1, nb, 3), qq.reshape(1, nb, 4))
try: art.set_body_inertias(dd.reshape(1, nb, 3))
except Exception:
    f = np.zeros((nb, 9), np.float32); f[:, 0], f[:, 4], f[:, 8] = dd.T
    art.set_body_inertias(f.reshape(1, nb, 9))
art.set_armatures(cfg.ARMATURE.reshape(1, ARM).astype(np.float32), joint_indices=idx)
art.set_friction_coefficients(np.zeros((1, ARM), np.float32), joint_indices=idx)

F0 = np.array([.55, .45, .60, .50, .75, .70, .80])
K = sum(1./(h+1) for h in range(3))
A = FRAC*cfg.QD_MAX/(2*np.pi*F0*K)


def tr(t):
    q = cfg.HOME_JOINTS.copy()
    for h in range(3): q += (A/(h+1)**2)*np.sin(2*np.pi*F0*(h+1)*t)
    return q


art.set_joint_positions(np.concatenate([tr(0.), [.04, .04]]))
for _ in range(int(0.3*HZ)): world.step(render=False)

# Analytic qd/qdd of the commanded trajectory, so the finite-difference estimate can be
# compared against a noise-free reference. If the residual collapses when qdd comes from
# the trajectory instead of from differencing qd, the residual is FD noise, not dynamics.
def tr_d(t):
    qd = np.zeros(ARM); qdd = np.zeros(ARM)
    for h in range(3):
        w = 2*np.pi*F0*(h+1); a = A/(h+1)**2
        qd += a*w*np.cos(w*t); qdd -= a*w*w*np.sin(w*t)
    return qd, qdd


qp = np.zeros(ARM); R, QD = [], []
R_an, R_filt = [], []
qdd_f = np.zeros(ARM)
for k in range(STEPS):
    art.set_joint_position_targets(np.concatenate([tr(k*DT), [.04, .04]]).reshape(1, -1))
    world.step(render=False)
    qd = np.asarray(art.get_joint_velocities()).flatten()[:ARM]
    g = np.asarray(art.get_generalized_gravity_forces()).flatten()[:ARM]
    c = np.asarray(art.get_coriolis_and_centrifugal_forces()).flatten()[:ARM]
    M = np.asarray(art.get_mass_matrices()).reshape(9, 9)[:ARM, :ARM]
    tm = np.asarray(art.get_measured_joint_efforts()).flatten()[:ARM]
    qdd = (qd-qp)/DT; qp = qd.copy()
    qdd_f = 0.9*qdd_f + 0.1*qdd                       # light low pass on the FD estimate
    _, qdd_cmd = tr_d(k*DT)                           # analytic, noise free
    if k > WARM:
        R.append(tm-(M@qdd+c+g)); QD.append(qd.copy())
        R_an.append(tm-(M@qdd_cmd+c+g))
        R_filt.append(tm-(M@qdd_f+c+g))

R = np.array(R); QD = np.array(QD)
rms = np.sqrt(np.mean(R**2, axis=0)); pct = 100*rms/cfg.TAU_MAX
rms_an = np.sqrt(np.mean(np.array(R_an)**2, axis=0)); pct_an = 100*rms_an/cfg.TAU_MAX
rms_fl = np.sqrt(np.mean(np.array(R_filt)**2, axis=0)); pct_fl = 100*rms_fl/cfg.TAU_MAX
qdd_fd_std = float(np.std(np.diff(QD, axis=0)/DT))
out = {"hz": HZ, "frac": FRAC, "kp": KP, "steps": len(R),
       "qd_peak": float(np.max(np.abs(QD))),
       "rms": rms.tolist(), "pct_of_limit": pct.tolist(),
       "worst_pct": float(np.max(pct)), "mean_rms": float(np.mean(rms))}
print(f"\n[RESULT] hz={HZ:.0f} frac={FRAC} kp={KP:.0f}  qd_pk={out['qd_peak']:.2f}  "
      f"mean_rms={out['mean_rms']:.4f}  worst={out['worst_pct']:.2f}%", flush=True)
print(f"[RESULT] per-joint %: " + " ".join(f"{p:.2f}" for p in pct), flush=True)
print(f"[RESULT] qdd source   FD {np.max(pct):.2f}%   FD+filter {np.max(pct_fl):.2f}%   "
      f"analytic {np.max(pct_an):.2f}%", flush=True)
out["worst_pct_analytic_qdd"] = float(np.max(pct_an))
out["worst_pct_filtered_qdd"] = float(np.max(pct_fl))
with open(f"/workspace/out/rate_{int(HZ)}_{FRAC}_{int(KP)}.json", "w") as f:
    json.dump(out, f, indent=2)
simulation_app.close()
