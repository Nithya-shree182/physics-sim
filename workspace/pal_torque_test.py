#!/usr/bin/env python3
"""Settle one question with data: is tau_applied the same signal as tau_measured?

If a stiff PD drive simply pumps out whatever torque the load demands, then the drive's
output IS the joint's effort, the two readings coincide, and the residual is the same
whichever one is used. Tested at rest and under load, in position mode and in effort mode.
"""
import os
import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_torque_test.py running", flush=True)

from pxr import PhysxSchema
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation
import pal_config as cfg

world = World(stage_units_in_meters=1.0, physics_dt=1/1000.0, rendering_dt=1/30.0)
add_reference_to_stage(usd_path=get_assets_root_path() + cfg.ROBOT_USD_SUBPATH,
                       prim_path=cfg.ROBOT_PRIM)
art = Articulation(cfg.ROBOT_PRIM)
world.reset()
art.initialize()

N = 7
home = np.concatenate([cfg.HOME_JOINTS, [0.04, 0.04]])


def settle(steps=600):
    for _ in range(steps):
        world.step(render=False)


def sample(label):
    g = np.asarray(art.get_generalized_gravity_forces()).flatten()[:N]
    tm = np.asarray(art.get_measured_joint_efforts()).flatten()[:N]
    ta = np.asarray(art.get_applied_joint_efforts()).flatten()[:N]
    print(f"\n  {label}")
    print(f"  {'joint':<9}{'g(q)':>10}{'tau_measured':>14}{'tau_applied':>13}{'meas-appl':>12}")
    for i in range(N):
        print(f"  joint{i+1:<4}{g[i]:>10.4f}{tm[i]:>14.4f}{ta[i]:>13.4f}{tm[i]-ta[i]:>12.4f}")
    print(f"  max |tau_measured - tau_applied| = {np.max(np.abs(tm-ta)):.4f} N*m")
    return g, tm, ta


print("\n" + "=" * 74)
print("  TEST 1 — position mode (kp=400/rad), arm at rest, NO payload")
print("=" * 74)
art.set_gains(kps=np.full((1, N), 400.0*cfg.DEG_PER_RAD, dtype=np.float32),
              kds=np.full((1, N), 80.0*cfg.DEG_PER_RAD, dtype=np.float32),
              joint_indices=np.arange(N))
art.set_joint_positions(home)
art.set_joint_position_targets(home)
settle()
g0, tm0, ta0 = sample("no payload, holding home pose")

print("\n" + "=" * 74)
print("  TEST 2 — same pose, 3 kg added to the hand (the 'unknown payload' case)")
print("=" * 74)
# Raise the hand's own mass by 3 kg: the cleanest stand-in for a gripped payload, with no
# contact dynamics to muddy the reading.
hand = world.stage.GetPrimAtPath(f"{cfg.ROBOT_PRIM}/panda_hand")
mass_api = PhysxSchema.PhysxRigidBodyAPI.Apply(hand) if hand.IsValid() else None
from pxr import UsdPhysics
mapi = UsdPhysics.MassAPI.Apply(hand)
base_mass = cfg.HAND_MASS
mapi.CreateMassAttr(base_mass + 3.0)
world.reset()
art.set_joint_positions(home)
art.set_joint_position_targets(home)
settle()
g1, tm1, ta1 = sample("3 kg payload, same pose")

print(f"\n  gravity term change  g1 - g0 = {np.round(g1-g0, 3)}")
print(f"  measured change     tm1 - tm0 = {np.round(tm1-tm0, 3)}")
print(f"  applied  change     ta1 - ta0 = {np.round(ta1-ta0, 3)}")

print("\n" + "=" * 74)
print("  TEST 3 — effort/torque mode: command a known torque, read both back")
print("=" * 74)
mapi.CreateMassAttr(base_mass)          # payload removed
world.reset()
art.set_joint_positions(home)
try:
    art.switch_control_mode("effort")
    print("  switched to effort control mode")
except Exception as e:
    print(f"  switch_control_mode('effort') failed: {e}")
    try:
        art.set_gains(kps=np.zeros((1, N), dtype=np.float32),
                      kds=np.zeros((1, N), dtype=np.float32), joint_indices=np.arange(N))
        print("  fell back to zeroed gains")
    except Exception as e2:
        print(f"  fallback failed: {e2}")

# Command exactly the gravity torque: the arm should hold station, and whatever we
# commanded is by definition tau_applied.
g_now = np.asarray(art.get_generalized_gravity_forces()).flatten()[:N]
cmd = np.zeros(9, dtype=np.float32)
cmd[:N] = g_now
art.set_joint_efforts(cmd.reshape(1, 9))
settle(300)
sample("effort mode, commanding tau = g(q)")
print(f"  commanded           {np.round(g_now, 4)}")

simulation_app.close()
