#!/usr/bin/env python3
"""Controlled speed sweep at ONE physics rate. Run once per rate, in its own process.

Earlier speed and rate numbers disagreed between scripts — pal_fast_random at 1000 Hz gave
1.81% at half speed while pal_experiment at 500 Hz gave 6.47%, yet the rate sweep at full
speed said 500 Hz beat 1000 Hz. Those cross over, which no single mechanism explains, so
the scripts must have differed in something other than rate.

Everything that could differ is pinned here:
  - duration measured in SIMULATED seconds, not steps, so every rate sees the same motion
  - identical settle and warm-up, also in simulated seconds
  - trajectory amplitude depends only on the speed fraction, never on the rate
  - one build per process; a World is never constructed twice
  - RMS and peak reported separately, since mixing them was its own confusion

    PAL_PHYSICS_HZ=1000 tools/run_sim.sh pal_sweep.py
"""
import json
import os
import xml.etree.ElementTree as ET

import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_sweep.py running", flush=True)

from pxr import PhysxSchema
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation
import pal_config as cfg

ARM = cfg.DOF
HZ = float(os.environ.get("PAL_PHYSICS_HZ", 1000.0))
KP = float(os.environ.get("PAL_KP", 400.0))
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
art.set_gains(kps=np.full((1, ARM), KP*cfg.DEG_PER_RAD, np.float32),
              kds=np.full((1, ARM), 80.0*cfg.DEG_PER_RAD, np.float32), joint_indices=idx)
art.set_body_masses(mm.reshape(1, NB)); art.set_body_coms(cc.reshape(1, NB, 3), qq.reshape(1, NB, 4))
try: art.set_body_inertias(dd.reshape(1, NB, 3))
except Exception:
    f = np.zeros((NB, 9), np.float32); f[:, 0], f[:, 4], f[:, 8] = dd.T
    art.set_body_inertias(f.reshape(1, NB, 9))
art.set_armatures(cfg.ARMATURE.reshape(1, ARM).astype(np.float32), joint_indices=idx)
art.set_friction_coefficients(np.zeros((1, ARM), np.float32), joint_indices=idx)

F0 = np.array([.55, .45, .60, .50, .75, .70, .80])
K = sum(1./(h+1) for h in range(3))


def measure(frac):
    A = frac*cfg.QD_MAX/(2*np.pi*F0*K)      # depends on frac only, never on HZ
    def tr(t):
        q = cfg.HOME_JOINTS.copy()
        for h in range(3): q += (A/(h+1)**2)*np.sin(2*np.pi*F0*(h+1)*t)
        return q
    q0 = np.concatenate([tr(0.0), [.04, .04]])
    art.set_joint_positions(q0); art.set_joint_position_targets(q0.reshape(1, -1))
    for _ in range(int(SETTLE_S*HZ)): world.step(render=False)

    qp = np.zeros(ARM); R, QD, QDD = [], [], []
    total = int((WARM_S + MEASURE_S)*HZ); warm = int(WARM_S*HZ)
    for k in range(total):
        art.set_joint_position_targets(np.concatenate([tr(k*DT), [.04, .04]]).reshape(1, -1))
        world.step(render=False)
        qd = np.asarray(art.get_joint_velocities()).flatten()[:ARM]
        g = np.asarray(art.get_generalized_gravity_forces()).flatten()[:ARM]
        c = np.asarray(art.get_coriolis_and_centrifugal_forces()).flatten()[:ARM]
        M = np.asarray(art.get_mass_matrices()).reshape(9, 9)[:ARM, :ARM]
        tm = np.asarray(art.get_measured_joint_efforts()).flatten()[:ARM]
        qdd = (qd-qp)/DT; qp = qd.copy()
        if k >= warm:
            R.append(tm-(M@qdd+c+g)); QD.append(qd.copy()); QDD.append(qdd.copy())
    R = np.array(R); QD = np.array(QD); QDD = np.array(QDD)
    rms = np.sqrt(np.mean(R**2, axis=0)); pk = np.max(np.abs(R), axis=0)
    return {"frac": frac, "n": len(R),
            "qd_peak": float(np.max(np.abs(QD))), "qdd_peak": float(np.max(np.abs(QDD))),
            "rms": rms.tolist(), "peak": pk.tolist(),
            "rms_pct": (100*rms/cfg.TAU_MAX).tolist(),
            "worst_rms_pct": float(np.max(100*rms/cfg.TAU_MAX)),
            "worst_peak_pct": float(np.max(100*pk/cfg.TAU_MAX)),
            "mean_rms": float(np.mean(rms))}


print(f"\n  rate {HZ:.0f} Hz | kp {KP:.0f} N*m/rad | settle {SETTLE_S}s warm {WARM_S}s "
      f"measure {MEASURE_S}s (simulated)", flush=True)
print(f"  {'speed':>7}{'qd pk':>8}{'qdd pk':>9}{'RMS N*m':>10}{'worst RMS%':>12}"
      f"{'worst peak%':>13}", flush=True)
rows = []
for frac in (0.10, 0.20, 0.30, 0.50, 0.75, 1.00):
    r = measure(frac)
    rows.append(r)
    print(f"  {frac:>6.0%}{r['qd_peak']:>8.2f}{r['qdd_peak']:>9.1f}{r['mean_rms']:>10.4f}"
          f"{r['worst_rms_pct']:>11.2f}%{r['worst_peak_pct']:>12.2f}%", flush=True)

out = {"hz": HZ, "kp": KP, "settle_s": SETTLE_S, "warm_s": WARM_S,
       "measure_s": MEASURE_S, "rows": rows}
with open(f"/workspace/out/sweep_{int(HZ)}hz_kp{int(KP)}.json", "w") as f:
    json.dump(out, f, indent=2)
print(f"\n  written: out/sweep_{int(HZ)}hz_kp{int(KP)}.json", flush=True)
simulation_app.close()
