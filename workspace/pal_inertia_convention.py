#!/usr/bin/env python3
"""Which convention does set_body_coms / set_body_inertias use for off-diagonal inertia?

Tries each plausible way of writing a full inertia tensor and compares the simulator's resulting mass
matrix with the independent numpy one (pal_kinematics). The winner goes into pal_plant.py.
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_inertia_convention.py running", flush=True)
import numpy as np
import pal_config as cfg
import pal_trajectories as T
from pal_plant import build_plant
from pal_kinematics import Urdf
from pal_runner import Runner, Scenario

# (helpers kept here as evidence; they are NOT used by the plant)
def _quat_wxyz(R):
    """Unit quaternion (w, x, y, z) of a proper rotation matrix (Shepperd's method)."""
    t = np.trace(R)
    if t > 0:
        r = np.sqrt(1.0 + t); w = 0.5 * r; k = 0.5 / r
        q = [w, (R[2, 1] - R[1, 2]) * k, (R[0, 2] - R[2, 0]) * k, (R[1, 0] - R[0, 1]) * k]
    else:
        i = int(np.argmax(np.diag(R))); j, l = (i + 1) % 3, (i + 2) % 3
        r = np.sqrt(1.0 + R[i, i] - R[j, j] - R[l, l]); k = 0.5 / r
        v = [0.0, 0.0, 0.0]; v[i] = 0.5 * r
        v[j] = (R[j, i] + R[i, j]) * k; v[l] = (R[l, i] + R[i, l]) * k
        q = [(R[l, j] - R[j, l]) * k] + v
    q = np.array(q); return q / np.linalg.norm(q)


def principal_axes(tensor):
    """Principal moments and the quaternion of the principal-axes frame, from a full inertia tensor."""
    w, V = np.linalg.eigh(tensor)
    if np.linalg.det(V) < 0:
        V[:, 2] *= -1
    return w, _quat_wxyz(V)




def principal_axes(tensor):
    w, V = np.linalg.eigh(tensor)
    if np.linalg.det(V) < 0:
        V[:, 2] *= -1
    return w, _quat_wxyz(V)


plant = build_plant(1000.0, mass_scale=1.0)
runner = Runner(plant, "/workspace/out", mj_every=10)
runner.run(Scenario(name="x", traj=T.hold(cfg.HOME_JOINTS), traj_type="hold", duration_s=0.6, mass_scale=1.0), write=False)
q = np.asarray(plant.art.get_joint_positions()).flatten()[:7]
M_ref = Urdf(cfg.URDF_PATH).mass_matrix(q, 0.04, cfg.ARMATURE[0])
art, bodies, urdf = plant.art, plant.bodies, plant.urdf
nb = len(bodies)


def conj(q): return np.array([q[0], -q[1], -q[2], -q[3]])
def xyzw(q): return np.array([q[1], q[2], q[3], q[0]])


def apply(name, mode, quat_fn=None):
    """mode: 'full' = full URDF 3x3 in link frame; 'diag_link' = old behaviour (link-frame diagonal only);
    'principal' = eigenvalues as a diagonal 3x3 together with principal-axes quaternion quat_fn(q)."""
    coms, mats, quats = [], [], []
    for i, b in enumerate(bodies):
        if b not in urdf:
            coms.append(np.zeros(3)); mats.append(np.diag(np.full(3, 1e-3))); quats.append(np.array([1., 0, 0, 0])); continue
        u = urdf[b]; w, qq = principal_axes(u["tensor"])
        coms.append(u["com"])
        if mode == "full":
            mats.append(u["tensor"]); quats.append(np.array([1., 0, 0, 0]))
        elif mode == "diag_link":
            mats.append(np.diag(np.diag(u["tensor"]))); quats.append(np.array([1., 0, 0, 0]))
        else:
            mats.append(np.diag(w)); quats.append(quat_fn(qq))
    art.set_body_coms(np.array(coms, np.float32).reshape(1, nb, 3), np.array(quats, np.float32).reshape(1, nb, 4))
    art.set_body_inertias(np.array(mats, np.float32).reshape(1, nb, 9))
    plant.world.step(render=False)
    M = np.asarray(art.get_mass_matrices()).reshape(9, 9)[:7, :7] + cfg.ARMATURE[0] * np.eye(7)
    d = np.abs(M - M_ref)
    print(f"  {name:<52} worst {d.max():.5f}  (diagonal {np.abs(np.diag(d)).max():.5f})", flush=True)


apply("A  old plant: link-frame diagonal only", "diag_link")
apply("B  full 3x3 in link frame, identity orientation", "full")
apply("C  principal moments + quaternion wxyz", "principal", lambda x: x)
apply("D  principal moments + quaternion conjugated", "principal", conj)
simulation_app.close()
