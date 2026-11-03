#!/usr/bin/env python3
"""What in the simulator is making the fast-motion residual large?

Already ruled out across six runs: timing misalignment, Coriolis sign/definition,
finite-difference noise, velocity-limit saturation, mass-matrix slicing. The model form is
confirmed and closes to 0.15% at rest.

Remaining suspects are all simulator configuration. Each is varied alone here:

  A  effective drive stiffness   measured, not assumed — a known displacement, a measured
                                 restoring torque. The drive test showed 0.14 rad of
                                 tracking error producing only ~10 N*m, which a 400 N*m/rad
                                 drive cannot do.
  B  solver iteration count      an unconverged solver shows up directly as residual
  C  physics rate                does the residual scale with dt?
  D  armature                    in the plant but possibly not in the reported M
  E  drive type                  'force' vs 'acceleration' scales the drive by inertia
"""
import xml.etree.ElementTree as ET
import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
from pxr import PhysxSchema
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation
import pal_config as cfg

N = 7
_r = ET.parse("/workspace/assets/panda_arm_hand.urdf").getroot()
U = {}
for l in _r.findall("link"):
    it = l.find("inertial")
    if it is None: continue
    m, o, I = it.find("mass"), it.find("origin"), it.find("inertia")
    U[l.get("name")] = {"mass": float(m.get("value")),
        "com": [float(x) for x in o.get("xyz").split()] if o is not None and o.get("xyz") else [0.]*3,
        "diag": [float(I.get("ixx")), float(I.get("iyy")), float(I.get("izz"))]}

world = art = idx = None

def build(hz=500.0, iters=32, armature=True, kp=400.0, kd=80.0, author=True):
    global world, art, idx
    world = World(stage_units_in_meters=1.0, physics_dt=1.0/hz, rendering_dt=1/30.0)
    pc = world.get_physics_context(); pc.set_solver_type("TGS"); pc.enable_stablization(False)
    sp = PhysxSchema.PhysxSceneAPI.Apply(pc.get_current_physics_scene_prim())
    sp.CreateMinPositionIterationCountAttr(iters); sp.CreateMinVelocityIterationCountAttr(4)
    add_reference_to_stage(usd_path=get_assets_root_path()+cfg.ROBOT_USD_SUBPATH, prim_path=cfg.ROBOT_PRIM)
    art = Articulation(cfg.ROBOT_PRIM); world.reset(); art.initialize(); idx = np.arange(N)
    art.set_gains(kps=np.full((1,N), kp*cfg.DEG_PER_RAD, np.float32),
                  kds=np.full((1,N), kd*cfg.DEG_PER_RAD, np.float32), joint_indices=idx)
    if author:
        bodies = list(art.body_names); nb = len(bodies)
        cur = np.asarray(art.get_body_masses()).reshape(nb)
        mm = np.array([U[b]["mass"] if b in U else float(cur[i]) for i,b in enumerate(bodies)], np.float32)
        cc = np.array([U[b]["com"] if b in U else [0.]*3 for b in bodies], np.float32)
        dd = np.array([U[b]["diag"] if b in U else [1e-3]*3 for b in bodies], np.float32)
        qq = np.tile(np.array([1.,0,0,0], np.float32), (nb,1))
        art.set_body_masses(mm.reshape(1,nb)); art.set_body_coms(cc.reshape(1,nb,3), qq.reshape(1,nb,4))
        try: art.set_body_inertias(dd.reshape(1,nb,3))
        except Exception:
            f = np.zeros((nb,9), np.float32); f[:,0],f[:,4],f[:,8] = dd[:,0],dd[:,1],dd[:,2]
            art.set_body_inertias(f.reshape(1,nb,9))
    art.set_armatures((cfg.ARMATURE if armature else np.zeros(N)).reshape(1,N).astype(np.float32),
                      joint_indices=idx)
    art.set_friction_coefficients(np.zeros((1,N), np.float32), joint_indices=idx)


def residual_rms(hz, steps=3000, warm=600, frac=1.00):
    dt = 1.0/hz
    F0 = np.array([.55,.45,.60,.50,.75,.70,.80]); K = sum(1./(h+1) for h in range(3))
    A = frac*cfg.QD_MAX/(2*np.pi*F0*K)
    def tr(t):
        q = cfg.HOME_JOINTS.copy()
        for h in range(3): q += (A/(h+1)**2)*np.sin(2*np.pi*F0*(h+1)*t)
        return q
    art.set_joint_positions(np.concatenate([tr(0.), [.04,.04]]))
    for _ in range(300): world.step(render=False)
    qp = np.zeros(N); R = []
    for k in range(steps):
        art.set_joint_position_targets(np.concatenate([tr(k*dt), [.04,.04]]).reshape(1,-1))
        world.step(render=False)
        qd = np.asarray(art.get_joint_velocities()).flatten()[:N]
        g = np.asarray(art.get_generalized_gravity_forces()).flatten()[:N]
        c = np.asarray(art.get_coriolis_and_centrifugal_forces()).flatten()[:N]
        M = np.asarray(art.get_mass_matrices()).reshape(9,9)[:N,:N]
        tm = np.asarray(art.get_measured_joint_efforts()).flatten()[:N]
        qdd = (qd-qp)/dt; qp = qd.copy()
        if k > warm: R.append(tm - (M@qdd + c + g))
    R = np.array(R); rms = np.sqrt(np.mean(R**2, axis=0))
    return float(np.max(100*rms/cfg.TAU_MAX)), float(np.mean(rms))


print("\n"+"="*80)
print("  A — EFFECTIVE DRIVE STIFFNESS   (measured, not assumed)")
print("="*80)
for kp_set in (80.0, 400.0, 1000.0):
    build(kp=kp_set, kd=0.0)
    home9 = np.concatenate([cfg.HOME_JOINTS, [.04,.04]])
    art.set_joint_positions(home9)
    art.set_joint_position_targets(home9.reshape(1,-1))
    for _ in range(1500): world.step(render=False)
    g_ref = np.asarray(art.get_generalized_gravity_forces()).flatten()[:N].copy()
    t_ref = np.asarray(art.get_measured_joint_efforts()).flatten()[:N].copy()
    DQ = 0.05
    disp = cfg.HOME_JOINTS.copy(); disp[0] += DQ          # joint 1: no gravity load
    art.set_joint_positions(np.concatenate([disp, [.04,.04]]))
    for _ in range(1500): world.step(render=False)
    q_now = np.asarray(art.get_joint_positions()).flatten()[:N]
    t_now = np.asarray(art.get_measured_joint_efforts()).flatten()[:N]
    err = home9[0] - q_now[0]
    dtau = t_now[0] - t_ref[0]
    k_eff = dtau/err if abs(err) > 1e-9 else float('nan')
    print(f"  kp set {kp_set:>7.0f} N*m/rad ->  err {err:+.5f} rad   dtau {dtau:+.4f} N*m"
          f"   k_eff {k_eff:>9.1f} N*m/rad   ratio {k_eff/kp_set:>6.3f}")

print("\n"+"="*80)
print("  B — SOLVER ITERATIONS")
print("="*80)
print(f"  {'pos iters':>10}{'worst %lim':>13}{'mean RMS':>11}")
for it in (4, 16, 32, 64, 128):
    build(iters=it)
    w, m = residual_rms(500.0)
    print(f"  {it:>10}{w:>12.2f}%{m:>11.4f}")

print("\n"+"="*80)
print("  C — PHYSICS RATE   (does the residual scale with dt?)")
print("="*80)
print(f"  {'rate':>8}{'worst %lim':>13}{'mean RMS':>11}")
for hz in (200.0, 500.0, 1000.0, 2000.0):
    build(hz=hz)
    w, m = residual_rms(hz)
    print(f"  {hz:>7.0f}Hz{w:>12.2f}%{m:>11.4f}")

print("\n"+"="*80)
print("  D — ARMATURE   (in the plant but maybe not in the reported M)")
print("="*80)
for arm in (False, True):
    build(armature=arm)
    w, m = residual_rms(500.0)
    print(f"  armature {'0.0061' if arm else '0.0   '}  worst {w:>6.2f}%   mean RMS {m:.4f}")

print("\n"+"="*80)
print("  E — DRIVE STIFFNESS vs RESIDUAL")
print("="*80)
print(f"  {'kp':>8}{'worst %lim':>13}{'mean RMS':>11}")
for kp in (80.0, 400.0, 1000.0, 5000.0):
    build(kp=kp, kd=2*np.sqrt(kp))
    w, m = residual_rms(500.0)
    print(f"  {kp:>8.0f}{w:>12.2f}%{m:>11.4f}")

simulation_app.close()
