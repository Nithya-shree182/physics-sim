#!/usr/bin/env python3
"""R2.3 from the analysis side: does the above-knee residual survive in EFFORT mode?

In position mode the reported tau_measured of a stiff implicit drive may not be the torque
that moved the arm. Here the drive is switched off (kp = kd = 0, control mode "effort") and
the arm is driven by explicit torques

    tau_cmd = M(q) (qdd_des + Kp e + Kd ed) + C(q,qd) qd + g(q)        e = q_des - q

so the applied torque is known exactly. If the residual r = tau_measured - (M qdd + C + g)
collapses relative to pal_sweep.py (position mode, same trajectory, same windows), the
artifact belongs to the position drive and O2/O5 should be recorded in effort mode. If it
stays, it is in the dynamics identity itself.

Also reported: tau_measured - tau_cmd, i.e. whether tau_measured is the torque we applied.

    PAL_PHYSICS_HZ=1000 tools/run_sim.sh pal_effort_test.py
"""
import json
import os
import xml.etree.ElementTree as ET

import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_effort_test.py running", flush=True)

from pxr import PhysxSchema
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation
import pal_config as cfg

ARM = 7
HZ = float(os.environ.get("PAL_PHYSICS_HZ", 1000.0))
KP_FB, KD_FB = 400.0, 40.0      # feedback gains in acceleration space
DT = 1.0 / HZ
SETTLE_S, WARM_S, MEASURE_S = 1.0, 1.0, 3.0      # simulated seconds, identical at every rate

world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=1/30.0)
pc = world.get_physics_context()
pc.set_solver_type(cfg.SOLVER_TYPE); pc.enable_ccd(cfg.ENABLE_CCD)
pc.enable_stablization(cfg.ENABLE_STABILIZATION); pc.set_gravity(cfg.GRAVITY)
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
B = list(art.body_names); NB = len(B)
cur = np.asarray(art.get_body_masses()).reshape(NB)
mm = np.array([U[b]["mass"] if b in U else float(cur[i]) for i, b in enumerate(B)], np.float32)
cc = np.array([U[b]["com"] if b in U else [0.]*3 for b in B], np.float32)
dd = np.array([U[b]["diag"] if b in U else [1e-3]*3 for b in B], np.float32)
qq = np.tile(np.array([1., 0, 0, 0], np.float32), (NB, 1))
art.set_body_masses(mm.reshape(1, NB)); art.set_body_coms(cc.reshape(1, NB, 3), qq.reshape(1, NB, 4))
try: art.set_body_inertias(dd.reshape(1, NB, 3))
except Exception:
    f = np.zeros((NB, 9), np.float32); f[:, 0], f[:, 4], f[:, 8] = dd.T
    art.set_body_inertias(f.reshape(1, NB, 9))
art.set_armatures(cfg.ARMATURE.reshape(1, ARM).astype(np.float32), joint_indices=idx)
art.set_friction_coefficients(np.zeros((1, ARM), np.float32), joint_indices=idx)

art.set_gains(kps=np.zeros((1, ARM), np.float32), kds=np.zeros((1, ARM), np.float32),
              joint_indices=idx)
art.switch_control_mode("effort", joint_indices=idx)
print(f"[PAL] control mode switched to effort; kp=kd=0", flush=True)

F0 = np.array([.55, .45, .60, .50, .75, .70, .80])
K = sum(1./(h+1) for h in range(3))


def vec(x, n=ARM): return np.asarray(x).flatten()[:n]


def measure(frac):
    A = frac*cfg.QD_MAX/(2*np.pi*F0*K)
    def tr(t):
        q = cfg.HOME_JOINTS.copy(); qd = np.zeros(ARM); qdd = np.zeros(ARM)
        for h in range(3):
            w = 2*np.pi*F0*(h+1); a = A/(h+1)**2
            q += a*np.sin(w*t); qd += a*w*np.cos(w*t); qdd += -a*w*w*np.sin(w*t)
        return q, qd, qdd
    q0 = np.concatenate([tr(0.0)[0], [.04, .04]])
    art.set_joint_positions(q0)
    art.set_joint_velocities(np.concatenate([tr(0.0)[1], [0., 0.]]).reshape(1, -1))
    art.set_joint_efforts(np.zeros((1, 9), np.float32))
    for _ in range(int(0.2*HZ)): world.step(render=False)       # let the fingers settle

    qp = None; R, E, U, TR, QD, QDD, CMD = [], [], [], [], [], [], []
    total = int((WARM_S + MEASURE_S)*HZ); warm = int(WARM_S*HZ)
    tau = np.zeros(9, np.float32)
    for k in range(total):
        q = vec(art.get_joint_positions()); qd = vec(art.get_joint_velocities())
        g = vec(art.get_generalized_gravity_forces())
        c = vec(art.get_coriolis_and_centrifugal_forces())
        M = np.asarray(art.get_mass_matrices()).reshape(9, 9)[:ARM, :ARM]
        qs, qds, qdds = tr(k*DT)
        tau_cmd = M@(qdds + KP_FB*(qs-q) + KD_FB*(qds-qd)) + c + g
        tau[:ARM] = tau_cmd
        art.set_joint_efforts(tau.reshape(1, 9))
        world.step(render=False)
        qd1 = vec(art.get_joint_velocities())
        tm = vec(art.get_measured_joint_efforts())
        qdd = (qd1-qd)/DT if qp is None else (qd1-qp)/DT
        qp = qd1.copy()
        # model terms at the state the step ended in, consistent with pal_sweep.py
        g1 = vec(art.get_generalized_gravity_forces())
        c1 = vec(art.get_coriolis_and_centrifugal_forces())
        M1 = np.asarray(art.get_mass_matrices()).reshape(9, 9)[:ARM, :ARM]
        if k >= warm:
            R.append(tm-(M1@qdd+c1+g1)); U.append(tm-tau_cmd)
            E.append(vec(art.get_joint_positions())-tr((k+1)*DT)[0])
            QD.append(qd1.copy()); QDD.append(qdd.copy()); CMD.append(tau_cmd.copy())
    R, U, E, QD, QDD, CMD = (np.array(x) for x in (R, U, E, QD, QDD, CMD))
    rms = np.sqrt(np.mean(R**2, axis=0)); pk = np.max(np.abs(R), axis=0)
    urms = np.sqrt(np.mean(U**2, axis=0))
    return {"frac": frac, "n": len(R),
            "qd_peak": float(np.max(np.abs(QD))), "qdd_peak": float(np.max(np.abs(QDD))),
            "track_err_peak_rad": float(np.max(np.abs(E))),
            "cmd_peak_pct_limit": float(np.max(100*np.max(np.abs(CMD), axis=0)/cfg.TAU_MAX)),
            "rms": rms.tolist(), "rms_pct": (100*rms/cfg.TAU_MAX).tolist(),
            "worst_rms_pct": float(np.max(100*rms/cfg.TAU_MAX)),
            "worst_peak_pct": float(np.max(100*pk/cfg.TAU_MAX)),
            "mean_rms": float(np.mean(rms)),
            "meas_vs_cmd_worst_rms_pct": float(np.max(100*urms/cfg.TAU_MAX))}


print(f"\n  EFFORT MODE | rate {HZ:.0f} Hz | settle/warm/measure {WARM_S}s/{MEASURE_S}s", flush=True)
print(f"  {'speed':>7}{'qd pk':>8}{'track pk':>10}{'cmd pk%':>9}{'worst RMS%':>12}"
      f"{'worst peak%':>13}{'meas-cmd%':>11}", flush=True)
rows = []
for frac in (0.10, 0.50, 0.75, 1.00):
    r = measure(frac); rows.append(r)
    print(f"  {frac:>6.0%}{r['qd_peak']:>8.2f}{r['track_err_peak_rad']:>10.4f}"
          f"{r['cmd_peak_pct_limit']:>8.0f}%{r['worst_rms_pct']:>11.2f}%"
          f"{r['worst_peak_pct']:>12.2f}%{r['meas_vs_cmd_worst_rms_pct']:>10.2f}%", flush=True)

with open(f"/workspace/out/effort_{int(HZ)}hz.json", "w") as f:
    json.dump({"hz": HZ, "mode": "effort", "kp_fb": KP_FB, "kd_fb": KD_FB, "rows": rows}, f, indent=2)
print(f"\n  written: out/effort_{int(HZ)}hz.json", flush=True)
simulation_app.close()
