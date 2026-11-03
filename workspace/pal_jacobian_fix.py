#!/usr/bin/env python3
"""Find the Jacobian link index that matches the gravity term.

get_jacobians() returns (1, 10, 6, 9) — ten link Jacobians. The audit took the last one,
and joints 3 and 5 then disagreed with g(q) by ~1 N*m. Rather than guess, add a known
point mass to a known body and sweep every index: the correct frame is the one where
J_v^T W reproduces the measured change in the gravity term.
"""
import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_jacobian_fix.py running", flush=True)

from pxr import UsdPhysics, Usd, UsdGeom
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation
import pal_config as cfg

N, M_PAY = 7, 3.0
world = World(stage_units_in_meters=1.0, physics_dt=1/1000.0, rendering_dt=1/30.0)
add_reference_to_stage(usd_path=get_assets_root_path() + cfg.ROBOT_USD_SUBPATH,
                       prim_path=cfg.ROBOT_PRIM)
art = Articulation(cfg.ROBOT_PRIM)
world.reset(); art.initialize()
home = np.concatenate([cfg.HOME_JOINTS, [0.04, 0.04]])
hand = world.stage.GetPrimAtPath(f"{cfg.ROBOT_PRIM}/panda_hand")
mapi = UsdPhysics.MassAPI.Apply(hand)


def hold(steps=800):
    art.set_joint_positions(home); art.set_joint_position_targets(home)
    for _ in range(steps):
        world.step(render=False)


print("\n  articulation body order (what the Jacobian's 10 rows should follow):")
try:
    names = art.body_names
    for i, n in enumerate(names):
        print(f"    {i:>3}  {n}")
except Exception as e:
    print(f"    body_names unavailable: {e}")

mapi.CreateMassAttr(cfg.HAND_MASS); world.reset(); hold()
g0 = np.asarray(art.get_generalized_gravity_forces()).flatten()[:N].copy()
mapi.CreateMassAttr(cfg.HAND_MASS + M_PAY); world.reset(); hold()
g1 = np.asarray(art.get_generalized_gravity_forces()).flatten()[:N].copy()
J = np.asarray(art.get_jacobians())
d_g = g1 - g0
W = np.array([0.0, 0.0, -M_PAY * 9.81])

Jb = J.reshape(J.shape[0], -1, J.shape[-2], J.shape[-1])[0]
print(f"\n  jacobian tensor {J.shape} -> {Jb.shape[0]} link Jacobians of {Jb.shape[1]}x{Jb.shape[2]}")
print(f"  measured  d_g = {np.round(d_g, 4)}")

print("\n" + "=" * 72)
print("  SWEEP — which link index reproduces d_g?")
print("=" * 72)
print(f"  {'idx':>4}{'max err':>11}{'RMS err':>11}   verdict")
best, best_err = None, 1e9
for i in range(Jb.shape[0]):
    Jv = Jb[i][:3, :N]
    tau = -Jv.T @ W
    err = np.max(np.abs(d_g - tau))
    rms = np.sqrt(np.mean((d_g - tau) ** 2))
    if err < best_err:
        best, best_err = i, err
    flag = "  <<< MATCH" if err < 0.05 else ("  close" if err < 0.5 else "")
    print(f"  {i:>4}{err:>11.4f}{rms:>11.4f}{flag}")

print(f"\n  best index {best}, max error {best_err:.5f} N*m")
Jv = Jb[best][:3, :N]
tau = -Jv.T @ W
print(f"\n  {'joint':<9}{'d g(q)':>12}{'J_v^T W':>12}{'diff':>11}")
for i in range(N):
    print(f"  joint{i+1:<4}{d_g[i]:>12.4f}{tau[i]:>12.4f}{d_g[i]-tau[i]:>11.5f}")

print("\n" + "=" * 72)
print("  TCP OFFSET — does the match improve with the 0.0584 m fingertip offset?")
print("=" * 72)
# A wrench resolved at the wrong point along the hand's z axis carries a spurious moment.
# Shift the Jacobian to the TCP: J_tcp = J_link + [omega x r] contribution, i.e.
# Jv_tcp = Jv_link - skew(r_world) @ Jw_link  for an offset r expressed in world frame.
hand_prim = world.stage.GetPrimAtPath(f"{cfg.ROBOT_PRIM}/panda_hand")
Tw = UsdGeom.Xformable(hand_prim).ComputeLocalToWorldTransform(0)
R = np.array([[Tw[r][c] for c in range(3)] for r in range(3)])
r_world = R.T @ np.array([0.0, 0.0, cfg.TCP_OFFSET])


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


for label, idx in (("best", best), ("last", Jb.shape[0] - 1)):
    Jl = Jb[idx]
    Jv_t = Jl[:3, :N] - skew(r_world) @ Jl[3:6, :N]
    e = np.max(np.abs(d_g - (-Jv_t.T @ W)))
    e0 = np.max(np.abs(d_g - (-Jl[:3, :N].T @ W)))
    print(f"  index {idx} ({label}): max err  no offset {e0:.5f}  |  with TCP offset {e:.5f}")

simulation_app.close()
