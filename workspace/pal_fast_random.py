#!/usr/bin/env python3
"""Characterise the sim's torque model at speed, and over random motion.

P1 established that the fast-motion residual is time discretization, first order in dt.
That is a floor, not a bug — so the job now is to MAP it, because O5's detection
thresholds have to sit above it and a threshold chosen without this map is arbitrary.

  1  Speed sweep at fixed dt: residual vs commanded velocity fraction, per joint.
  2  Random motion: many random pose/velocity/acceleration states across the reachable
     workspace, so the floor is characterised over the whole arm rather than along one
     hand-picked trajectory.
  3  The practical question: given the floor, what payload is detectable at each speed?

Everything is computed from the simulator's own M, C, g — this measures the plant's
internal consistency, which is what sets the floor the independent model will also see.
"""
import json
import xml.etree.ElementTree as ET

import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_fast_random.py running", flush=True)

from pxr import PhysxSchema
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation
import pal_config as cfg

ARM, HZ = 7, 1000.0
DT = 1.0 / HZ
rng = np.random.RandomState(11)

world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=1/30.0)
pc = world.get_physics_context()
pc.set_solver_type("TGS"); pc.enable_stablization(False); pc.set_gravity(cfg.GRAVITY)
sp = PhysxSchema.PhysxSceneAPI.Apply(pc.get_current_physics_scene_prim())
sp.CreateMinPositionIterationCountAttr(cfg.POSITION_ITERATIONS)
sp.CreateMinVelocityIterationCountAttr(cfg.VELOCITY_ITERATIONS)
add_reference_to_stage(usd_path=get_assets_root_path()+cfg.ROBOT_USD_SUBPATH, prim_path=cfg.ROBOT_PRIM)
art = Articulation(cfg.ROBOT_PRIM); world.reset(); art.initialize()
idx = np.arange(ARM)

# Author the URDF inertials (after reset — a reset discards tensor-API writes).
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
art.set_gains(kps=np.full((1, ARM), 400.0*cfg.DEG_PER_RAD, np.float32),
              kds=np.full((1, ARM), 80.0*cfg.DEG_PER_RAD, np.float32), joint_indices=idx)
art.set_body_masses(mm.reshape(1, nb)); art.set_body_coms(cc.reshape(1, nb, 3), qq.reshape(1, nb, 4))
try: art.set_body_inertias(dd.reshape(1, nb, 3))
except Exception:
    f = np.zeros((nb, 9), np.float32); f[:, 0], f[:, 4], f[:, 8] = dd.T
    art.set_body_inertias(f.reshape(1, nb, 9))
art.set_armatures(cfg.ARMATURE.reshape(1, ARM).astype(np.float32), joint_indices=idx)
art.set_friction_coefficients(np.zeros((1, ARM), np.float32), joint_indices=idx)

F0 = np.array([.55, .45, .60, .50, .75, .70, .80])
KH = sum(1./(h+1) for h in range(3))


def run_traj(frac, steps=4000, warm=800, center=None, phase=None):
    A = frac*cfg.QD_MAX/(2*np.pi*F0*KH)
    ph = np.zeros((3, ARM)) if phase is None else phase
    ctr = cfg.HOME_JOINTS if center is None else center
    def tr(t):
        q = ctr.copy()
        for h in range(3): q += (A/(h+1)**2)*np.sin(2*np.pi*F0*(h+1)*t + ph[h])
        return q
    art.set_joint_positions(np.concatenate([tr(0.), [.04, .04]]))
    for _ in range(300): world.step(render=False)
    qp = np.zeros(ARM); R, QD, QDD = [], [], []
    for k in range(steps):
        art.set_joint_position_targets(np.concatenate([tr(k*DT), [.04, .04]]).reshape(1, -1))
        world.step(render=False)
        qd = np.asarray(art.get_joint_velocities()).flatten()[:ARM]
        g = np.asarray(art.get_generalized_gravity_forces()).flatten()[:ARM]
        c = np.asarray(art.get_coriolis_and_centrifugal_forces()).flatten()[:ARM]
        M = np.asarray(art.get_mass_matrices()).reshape(9, 9)[:ARM, :ARM]
        tm = np.asarray(art.get_measured_joint_efforts()).flatten()[:ARM]
        qdd = (qd-qp)/DT; qp = qd.copy()
        if k > warm:
            R.append(tm-(M@qdd+c+g)); QD.append(qd.copy()); QDD.append(qdd.copy())
    return np.array(R), np.array(QD), np.array(QDD)


print("\n"+"="*94)
print("  1 — SPEED SWEEP at 1000 Hz   (residual floor vs commanded velocity)")
print("="*94)
print(f"  {'frac':>6}{'qd pk':>8}{'qdd pk':>9}  per-joint RMS residual as % of that joint's limit")
print(f"  {'':>6}{'':>8}{'':>9}      j1     j2     j3     j4     j5     j6     j7   worst")
sweep = {}
for frac in (0.10, 0.25, 0.50, 0.75, 1.00):
    R, QD, QDD = run_traj(frac, steps=3000, warm=600)
    rms = np.sqrt(np.mean(R**2, axis=0))
    pct = 100*rms/cfg.TAU_MAX
    sweep[frac] = {"rms": rms.tolist(), "pct": pct.tolist(),
                   "qd_pk": float(np.max(np.abs(QD))), "qdd_pk": float(np.max(np.abs(QDD)))}
    print(f"  {frac:>6.2f}{np.max(np.abs(QD)):>8.2f}{np.max(np.abs(QDD)):>9.1f}  "
          + "".join(f"{p:>7.2f}" for p in pct) + f"{np.max(pct):>8.2f}%")

print("\n"+"="*94)
print("  2 — RANDOM MOTION   (24 random states across the reachable workspace)")
print("="*94)
print("  Random centre pose, random harmonic phases, random speed fraction — so the floor")
print("  is characterised over the arm, not along one hand-picked trajectory.\n")
print(f"  {'trial':>5}{'frac':>6}{'qd pk':>8}{'|Mqdd|':>9}{'|Cqd|':>8}{'|g|':>8}"
      f"{'RMS res':>9}{'worst%':>8}")
rand = []
lo = np.maximum(cfg.Q_MIN+0.35, cfg.HOME_JOINTS-1.1)
hi = np.minimum(cfg.Q_MAX-0.35, cfg.HOME_JOINTS+1.1)
for trial in range(24):
    ctr = rng.uniform(lo, hi)
    ph = rng.uniform(0, 2*np.pi, (3, ARM))
    frac = float(rng.choice([0.15, 0.3, 0.5, 0.75, 1.0]))
    R, QD, QDD = run_traj(frac, steps=1200, warm=300, center=ctr, phase=ph)
    rms = np.sqrt(np.mean(R**2, axis=0)); pct = 100*rms/cfg.TAU_MAX
    # term sizes for context
    acc = np.mean(np.abs(QDD), axis=0)
    rand.append({"frac": frac, "qd_pk": float(np.max(np.abs(QD))),
                 "rms": rms.tolist(), "worst_pct": float(np.max(pct)),
                 "center": ctr.tolist()})
    print(f"  {trial:>5}{frac:>6.2f}{np.max(np.abs(QD)):>8.2f}"
          f"{np.mean(acc)*0+np.mean(np.abs(QDD)):>9.1f}{0:>8.0f}{0:>8.0f}"
          f"{np.mean(rms):>9.3f}{np.max(pct):>7.2f}%")

w = np.array([r["worst_pct"] for r in rand]); fr = np.array([r["frac"] for r in rand])
print(f"\n  worst-joint residual across 24 random states: "
      f"min {w.min():.2f}%  median {np.median(w):.2f}%  max {w.max():.2f}%")
print(f"  correlation with speed fraction: {np.corrcoef(fr, w)[0,1]:.3f}")
for f_ in sorted(set(fr.tolist())):
    sel = w[fr == f_]
    print(f"    frac {f_:.2f}: n={len(sel):>2}  median {np.median(sel):>6.2f}%  max {sel.max():>6.2f}%")

print("\n"+"="*94)
print("  3 — WHAT IS DETECTABLE ABOVE THE FLOOR?")
print("="*94)
print("  A payload of m kg adds about 4.63 N*m per kg at joint 4 (measured at home).")
print("  A fault must exceed the floor to be separable from it.\n")
print(f"  {'speed':>8}{'floor j4 (N*m)':>17}{'min payload (kg)':>19}{'3-sigma (kg)':>15}")
for frac, d in sweep.items():
    f4 = d["rms"][3]
    print(f"  {frac:>7.0%}{f4:>17.3f}{f4/4.6306:>19.3f}{3*f4/4.6306:>15.3f}")
print("\n  'min payload' is the mass whose gravity signature equals the floor — i.e. the")
print("  absolute detection limit. '3-sigma' is a usable threshold. Both are for a")
print("  STATIC signature; a moving payload also perturbs M and is easier to see.")

with open("/workspace/out/residual_floor.json", "w") as fh:
    json.dump({"sweep": {str(k): v for k, v in sweep.items()}, "random": rand,
               "physics_hz": HZ, "drive": "position_stiff 400/80 per rad",
               "payload_nm_per_kg_j4": 4.6306}, fh, indent=2)
print("\n  written: out/residual_floor.json")
simulation_app.close()
