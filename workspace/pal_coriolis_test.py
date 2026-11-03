#!/usr/bin/env python3
"""Which term is wrong? Residual grows superlinearly with speed, which is the signature
of a qd^2 term, so the Coriolis vector is the suspect.

get_coriolis_and_centrifugal_forces() could plausibly return C(q,qd)*qd, its negation, or
the nonlinear effects C*qd + g (in which case adding g separately double-counts it).
Each possibility is tried against the measured torque and the one that fits wins.
"""
import xml.etree.ElementTree as ET
import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_coriolis_test.py running", flush=True)

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation
import pal_config as cfg

N, HZ = 7, 1000.0
DT = 1/HZ
world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=1/30.0)
add_reference_to_stage(usd_path=get_assets_root_path() + cfg.ROBOT_USD_SUBPATH,
                       prim_path=cfg.ROBOT_PRIM)
art = Articulation(cfg.ROBOT_PRIM); world.reset(); art.initialize()
idx = np.arange(N)
art.set_gains(kps=np.full((1, N), 400.0*cfg.DEG_PER_RAD, np.float32),
              kds=np.full((1, N), 80.0*cfg.DEG_PER_RAD, np.float32), joint_indices=idx)

bodies = list(art.body_names); nb = len(bodies)
r = ET.parse("/workspace/assets/panda_arm_hand.urdf").getroot()
U = {}
for l in r.findall("link"):
    it = l.find("inertial")
    if it is None: continue
    m, o, I = it.find("mass"), it.find("origin"), it.find("inertia")
    U[l.get("name")] = {"mass": float(m.get("value")),
        "com": [float(x) for x in o.get("xyz").split()] if o is not None and o.get("xyz") else [0.]*3,
        "diag": [float(I.get("ixx")), float(I.get("iyy")), float(I.get("izz"))]}
cur = np.asarray(art.get_body_masses()).reshape(nb)
mm = np.array([U[b]["mass"] if b in U else float(cur[i]) for i, b in enumerate(bodies)], np.float32)
cc = np.array([U[b]["com"] if b in U else [0.]*3 for b in bodies], np.float32)
dd = np.array([U[b]["diag"] if b in U else [1e-3]*3 for b in bodies], np.float32)
qq = np.tile(np.array([1., 0, 0, 0], np.float32), (nb, 1))
art.set_body_masses(mm.reshape(1, nb)); art.set_body_coms(cc.reshape(1, nb, 3), qq.reshape(1, nb, 4))
try: art.set_body_inertias(dd.reshape(1, nb, 3))
except Exception:
    f = np.zeros((nb, 9), np.float32); f[:,0], f[:,4], f[:,8] = dd[:,0], dd[:,1], dd[:,2]
    art.set_body_inertias(f.reshape(1, nb, 9))
art.set_armatures(cfg.ARMATURE.reshape(1, N).astype(np.float32), joint_indices=idx)
art.set_friction_coefficients(np.zeros((1, N), np.float32), joint_indices=idx)

F0 = np.array([.55,.45,.60,.50,.75,.70,.80]); K = sum(1./(h+1) for h in range(3))
AMP = 1.00*cfg.QD_MAX/(2*np.pi*F0*K)
def traj(t):
    q = cfg.HOME_JOINTS.copy()
    for h in range(3): q += (AMP/(h+1)**2)*np.sin(2*np.pi*F0*(h+1)*t)
    return q

art.set_joint_positions(np.concatenate([traj(0.), [.04, .04]]))
for _ in range(400): world.step(render=False)

qd_prev = np.zeros(N)
S = {k: [] for k in ("Mqdd+C+g", "Mqdd-C+g", "Mqdd+g", "Mqdd+C", "M_full", "C_only_is_nle")}
for k in range(4000):
    art.set_joint_position_targets(np.concatenate([traj(k*DT), [.04, .04]]).reshape(1, -1))
    world.step(render=False)
    q = np.asarray(art.get_joint_positions()).flatten()[:N]
    qd = np.asarray(art.get_joint_velocities()).flatten()[:N]
    g = np.asarray(art.get_generalized_gravity_forces()).flatten()[:N]
    c = np.asarray(art.get_coriolis_and_centrifugal_forces()).flatten()[:N]
    M9 = np.asarray(art.get_mass_matrices()).reshape(9, 9)
    M = M9[:N, :N]
    tm = np.asarray(art.get_measured_joint_efforts()).flatten()[:N]
    qdd = (qd - qd_prev)/DT; qd_prev = qd.copy()
    if k <= 500: continue
    S["Mqdd+C+g"].append(tm - (M @ qdd + c + g))
    S["Mqdd-C+g"].append(tm - (M @ qdd - c + g))
    S["Mqdd+g"].append(tm - (M @ qdd + g))
    S["Mqdd+C"].append(tm - (M @ qdd + c))
    # full 9x9 M against a 9-vector qdd, in case dropping the finger block matters
    qdd9 = np.zeros(9); qdd9[:N] = qdd
    S["M_full"].append(tm - ((M9 @ qdd9)[:N] + c + g))
    # if the "coriolis" getter actually returns nonlinear effects (C*qd + g)
    S["C_only_is_nle"].append(tm - (M @ qdd + c))

print("\n" + "=" * 78)
print("  WHICH MODEL FORM MATCHES tau_measured?   (peak qd = 100% of datasheet limit)")
print("=" * 78)
print(f"  {'model form':<22}{'mean RMS':>11}{'worst % of limit':>19}")
best = None
for k, v in S.items():
    R = np.array(v); rms = np.sqrt(np.mean(R**2, axis=0))
    w = float(np.max(100*rms/cfg.TAU_MAX))
    if best is None or w < best[1]: best = (k, w)
    print(f"  {k:<22}{np.mean(rms):>11.4f}{w:>18.2f}%")
print(f"\n  best: {best[0]} at {best[1]:.2f}% of limit")

print("\n" + "=" * 78)
print("  SANITY: term magnitudes and whether C is already inside the gravity getter")
print("=" * 78)
art.set_joint_positions(np.concatenate([cfg.HOME_JOINTS, [.04, .04]]))
art.set_joint_velocities(np.zeros(9))
for _ in range(400): world.step(render=False)
g0 = np.asarray(art.get_generalized_gravity_forces()).flatten()[:N]
c0 = np.asarray(art.get_coriolis_and_centrifugal_forces()).flatten()[:N]
print(f"  at rest  g(q) = {np.round(g0, 4)}")
print(f"  at rest  C    = {np.round(c0, 6)}   (must be ~0 if it is C*qd alone)")
if np.max(np.abs(c0)) > 0.05:
    print("  C is NOT zero at rest -> the getter returns nonlinear effects (C*qd + g),")
    print("  so adding g separately DOUBLE COUNTS gravity.")
else:
    print("  C is zero at rest -> the getter returns C*qd alone, as assumed.")
simulation_app.close()
