#!/usr/bin/env python3
"""What is tau_measured, actually?

Ruled out so far for the fast-motion residual: timing misalignment (pre/post/midpoint
identical), Coriolis sign or definition (the assumed form wins, and C is zero at rest),
finite-difference noise (filtering changes it 2%), velocity-limit saturation (the sweep
never saturates), and the mass-matrix slice (full 9x9 identical).

The model FORM is confirmed correct and it closes to 0.15% at rest. So the remaining
question is what the measured effort represents during motion. Compare it against the
drive torque the PD law should be producing:

    tau_drive = kp*(q_target - q) + kd*(qd_target - qd)

If tau_measured tracks tau_drive, it is the actuator's output and the dynamics must
balance it. If it does not, the gap says what is missing.
"""
import xml.etree.ElementTree as ET
import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_drive_test.py running", flush=True)
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation
import pal_config as cfg

N, HZ = 7, 1000.0; DT = 1/HZ
KP, KD = 400.0, 80.0                      # per radian
world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=1/30.0)
add_reference_to_stage(usd_path=get_assets_root_path()+cfg.ROBOT_USD_SUBPATH, prim_path=cfg.ROBOT_PRIM)
art = Articulation(cfg.ROBOT_PRIM); world.reset(); art.initialize(); idx = np.arange(N)
art.set_gains(kps=np.full((1,N), KP*cfg.DEG_PER_RAD, np.float32),
              kds=np.full((1,N), KD*cfg.DEG_PER_RAD, np.float32), joint_indices=idx)
bodies=list(art.body_names); nb=len(bodies)
r=ET.parse("/workspace/assets/panda_arm_hand.urdf").getroot(); U={}
for l in r.findall("link"):
    it=l.find("inertial")
    if it is None: continue
    m,o,I=it.find("mass"),it.find("origin"),it.find("inertia")
    U[l.get("name")]={"mass":float(m.get("value")),
        "com":[float(x) for x in o.get("xyz").split()] if o is not None and o.get("xyz") else [0.]*3,
        "diag":[float(I.get("ixx")),float(I.get("iyy")),float(I.get("izz"))]}
cur=np.asarray(art.get_body_masses()).reshape(nb)
mm=np.array([U[b]["mass"] if b in U else float(cur[i]) for i,b in enumerate(bodies)],np.float32)
cc=np.array([U[b]["com"] if b in U else [0.]*3 for b in bodies],np.float32)
dd=np.array([U[b]["diag"] if b in U else [1e-3]*3 for b in bodies],np.float32)
qq=np.tile(np.array([1.,0,0,0],np.float32),(nb,1))
art.set_body_masses(mm.reshape(1,nb)); art.set_body_coms(cc.reshape(1,nb,3),qq.reshape(1,nb,4))
try: art.set_body_inertias(dd.reshape(1,nb,3))
except Exception:
    f=np.zeros((nb,9),np.float32); f[:,0],f[:,4],f[:,8]=dd[:,0],dd[:,1],dd[:,2]
    art.set_body_inertias(f.reshape(1,nb,9))
art.set_armatures(cfg.ARMATURE.reshape(1,N).astype(np.float32), joint_indices=idx)
art.set_friction_coefficients(np.zeros((1,N),np.float32), joint_indices=idx)

F0=np.array([.55,.45,.60,.50,.75,.70,.80]); K=sum(1./(h+1) for h in range(3))
AMP=1.00*cfg.QD_MAX/(2*np.pi*F0*K)
def traj(t):
    q=cfg.HOME_JOINTS.copy(); qd=np.zeros(N)
    for h in range(3):
        w=2*np.pi*F0*(h+1); a=AMP/(h+1)**2
        q+=a*np.sin(w*t); qd+=a*w*np.cos(w*t)
    return q,qd

art.set_joint_positions(np.concatenate([traj(0.)[0],[.04,.04]]))
for _ in range(400): world.step(render=False)

qd_prev=np.zeros(N); D=[]
for k in range(4000):
    t=k*DT; qt,qdt=traj(t)
    art.set_joint_position_targets(np.concatenate([qt,[.04,.04]]).reshape(1,-1))
    world.step(render=False)
    q=np.asarray(art.get_joint_positions()).flatten()[:N]
    qd=np.asarray(art.get_joint_velocities()).flatten()[:N]
    g=np.asarray(art.get_generalized_gravity_forces()).flatten()[:N]
    c=np.asarray(art.get_coriolis_and_centrifugal_forces()).flatten()[:N]
    M=np.asarray(art.get_mass_matrices()).reshape(9,9)[:N,:N]
    tm=np.asarray(art.get_measured_joint_efforts()).flatten()[:N]
    qdd=(qd-qd_prev)/DT; qd_prev=qd.copy()
    if k<=500: continue
    tau_drive=KP*(qt-q)+KD*(qdt-qd)
    tau_model=M@qdd+c+g
    D.append((tau_drive,tau_model,tm,q,qt,qd))

tau_drive=np.array([d[0] for d in D]); tau_model=np.array([d[1] for d in D])
tm=np.array([d[2] for d in D]); q=np.array([d[3] for d in D]); qt=np.array([d[4] for d in D])

print("\n"+"="*94)
print("  WHAT IS tau_measured?   peak commanded qd = 100% of the datasheet limit")
print("="*94)
print(f"  {'joint':<7}{'RMS tau_m':>11}{'RMS drive':>11}{'RMS model':>11}"
      f"{'|tm-drive|':>12}{'|tm-model|':>12}{'pos err rms':>13}")
for i in range(N):
    print(f"  j{i+1:<6}{np.sqrt(np.mean(tm[:,i]**2)):>11.3f}"
          f"{np.sqrt(np.mean(tau_drive[:,i]**2)):>11.3f}"
          f"{np.sqrt(np.mean(tau_model[:,i]**2)):>11.3f}"
          f"{np.sqrt(np.mean((tm[:,i]-tau_drive[:,i])**2)):>12.3f}"
          f"{np.sqrt(np.mean((tm[:,i]-tau_model[:,i])**2)):>12.3f}"
          f"{np.sqrt(np.mean((qt[:,i]-q[:,i])**2)):>13.4f}")

print("\n  correlation of tau_measured with each candidate:")
for lbl,X in (("drive law",tau_drive),("dynamics model",tau_model)):
    cs=[np.corrcoef(tm[:,i],X[:,i])[0,1] for i in range(N)]
    print(f"    {lbl:<16} {np.round(cs,3)}")

print("\n  Does the DRIVE torque match the DYNAMICS model? (both exclude tau_measured)")
r2=np.sqrt(np.mean((tau_drive-tau_model)**2,axis=0))
print(f"    RMS |drive - model| = {np.round(r2,3)}")
print("\n  If tau_measured tracks the drive law but the dynamics model does not match")
print("  either, the gap is a real actuator term (the drive is doing work the rigid-body")
print("  model does not account for). If tau_measured matches NEITHER, it is reporting")
print("  something other than the actuated joint torque.")
simulation_app.close()
