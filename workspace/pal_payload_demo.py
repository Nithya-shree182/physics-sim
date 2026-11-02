#!/usr/bin/env python3
"""O4 end to end: recover the mass of an unknown held object from torque alone.

Also demonstrates the structural point the work split depends on: the twin's model must
NOT know about the payload. Read g(q) back from the same simulator that is carrying the
object and the residual is identically zero, because the simulator has already accounted
for it. The nominal model has to come from somewhere else.
"""
import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from pxr import UsdPhysics
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation
import pal_config as cfg

N, G = 7, 9.81
world = World(stage_units_in_meters=1.0, physics_dt=1/1000.0, rendering_dt=1/30.0)
add_reference_to_stage(usd_path=get_assets_root_path() + cfg.ROBOT_USD_SUBPATH,
                       prim_path=cfg.ROBOT_PRIM)
art = Articulation(cfg.ROBOT_PRIM)
world.reset(); art.initialize()
hand = world.stage.GetPrimAtPath(f"{cfg.ROBOT_PRIM}/panda_hand")
mapi = UsdPhysics.MassAPI.Apply(hand)


def go(q7, payload_kg, steps=800):
    mapi.CreateMassAttr(cfg.HAND_MASS + float(payload_kg))
    world.reset()
    full = np.concatenate([q7, [0.04, 0.04]])
    art.set_joint_positions(full)
    art.set_joint_position_targets(full)
    for _ in range(steps):
        world.step(render=False)


def read():
    g = np.asarray(art.get_generalized_gravity_forces()).flatten()[:N]
    tm = np.asarray(art.get_measured_joint_efforts()).flatten()[:N]
    J = np.asarray(art.get_jacobians())
    Jl = J.reshape(J.shape[0], -1, J.shape[-2], J.shape[-1])[0, cfg.EE_JACOBIAN_ROW]
    return g, tm, Jl[:, :N]


print("\n" + "=" * 76)
print("  THE TRAP — using the simulator's own g(q) as the model")
print("=" * 76)
go(cfg.HOME_JOINTS, 3.0)
g_sim, tm, _ = read()
print(f"  3 kg held. tau_measured - g(q)_from_simulator = {np.round(tm - g_sim, 5)}")
print("  Zero. The simulator's gravity term already contains the payload, so comparing")
print("  against it cancels exactly the signal we are trying to measure.")
print("  => the twin's model must be INDEPENDENT. That is why Person B builds it in")
print("     Pinocchio from the URDF with nominal masses, not read back from the sim.")

print("\n" + "=" * 76)
print("  STEP 1 — build the nominal model (empty gripper) at each pose")
print("=" * 76)
POSES = {"home": cfg.HOME_JOINTS,
         "horizontal_reach": cfg.PROBE_POSES["horizontal_reach"],
         "near_singular_extended": cfg.PROBE_POSES["near_singular_extended"]}
nominal = {}
for name, q in POSES.items():
    go(q, 0.0)
    g0, _, J0 = read()
    sv = np.linalg.svd(J0, compute_uv=False)
    c = sv.min() / sv.max()
    nominal[name] = (g0.copy(), c)
    print(f"  {name:<24} g(q) stored, c(q) = {c:.4f}")
print("  (stands in for the twin's g_nominal(q); B's Pinocchio model plays this role)")

print("\n" + "=" * 76)
print("  STEP 2-6 — recover unknown masses at the home pose")
print("=" * 76)
g_nom, c_home = nominal["home"]
print(f"  {'true kg':>8}{'r j2':>9}{'r j4':>9}{'Fz (N)':>10}{'est kg':>9}"
      f"{'err g':>9}{'err %':>8}")
for true_m in [0.10, 0.25, 0.50, 1.00, 2.00, 3.00]:
    go(cfg.HOME_JOINTS, true_m)
    _, tm, J = read()
    r = tm - g_nom                               # residual against the NOMINAL model
    Jv = J[:3, :]                                # linear rows only
    F = -np.linalg.pinv(Jv.T) @ r                # wrench at the hand
    est = abs(F[2]) / G
    print(f"  {true_m:>8.2f}{r[1]:>9.3f}{r[3]:>9.3f}{F[2]:>10.3f}{est:>9.4f}"
          f"{1000*(est-true_m):>9.1f}{100*(est-true_m)/true_m:>7.2f}%")

print("\n" + "=" * 76)
print("  STEP 7 — the confidence gate: same 1 kg object, three poses")
print("=" * 76)
print(f"  {'pose':<24}{'c(q)':>8}{'est kg':>9}{'err %':>9}   gate")
for name, q in POSES.items():
    g_nom_p, c = nominal[name]
    go(q, 1.0)
    _, tm, J = read()
    r = tm - g_nom_p
    F = -np.linalg.pinv(J[:3, :].T) @ r
    est = abs(F[2]) / G
    gated = c < cfg.CONFIDENCE_THRESHOLD
    print(f"  {name:<24}{c:>8.4f}{est:>9.4f}{100*(est-1.0):>8.2f}%"
          f"   {'PAUSED (low trust)' if gated else 'accepted'}")

simulation_app.close()
