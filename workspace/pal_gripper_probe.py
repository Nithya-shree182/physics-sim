#!/usr/bin/env python3
"""Measure the gripper in the simulator: which variant, how are the fingers driven, and where are the fingertips relative to panda_hand.

    tools/run_sim.sh pal_gripper_probe.py
"""
from pal_app import HEADLESS, start_app
simulation_app = start_app("pal_gripper_probe.py")

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom
import pal_config as cfg
from pal_plant import build_plant

plant = build_plant(1000.0, gripper=True)
art, pv = plant.art, plant.pv
print("joint names:", list(art.dof_names), flush=True)
kps, kds = art.get_gains()
print("stock gains kp:", np.round(np.asarray(kps).flatten(), 1).tolist(), flush=True)
print("stock gains kd:", np.round(np.asarray(kds).flatten(), 1).tolist(), flush=True)
print("max efforts   :", np.round(np.asarray(art.get_max_efforts()).flatten(), 1).tolist(), flush=True)
print("joint limits  :", np.round(np.asarray(art.get_dof_limits()).reshape(-1, 2)[7:], 3).tolist(), flush=True)

q0 = np.concatenate([cfg.HOME_JOINTS, [0.04, 0.04]])
art.set_joint_positions(q0)
art.set_joint_position_targets(q0.reshape(1, -1))
for _ in range(200):
    plant.world.step(render=False)

stage = omni.usd.get_context().get_stage()
bodies = list(art.body_names)
T = np.asarray(pv.get_link_transforms()).reshape(len(bodies), 7)      # x y z qx qy qz qw


def rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


h = bodies.index("panda_hand"); Rh, ph = rot(T[h, 3:]), T[h, :3]
for name in ("panda_hand", "panda_leftfinger", "panda_rightfinger"):
    i = bodies.index(name)
    print(f"{name:<18} frame in the hand frame: {np.round(Rh.T @ (T[i, :3] - ph), 4).tolist()}", flush=True)
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
for name in ("panda_leftfinger", "panda_rightfinger"):
    box = cache.ComputeWorldBound(stage.GetPrimAtPath(f"{cfg.ROBOT_PRIM}/{name}")).ComputeAlignedBox()
    corners = np.array([[x, y, z] for x in (box.GetMin()[0], box.GetMax()[0]) for y in (box.GetMin()[1], box.GetMax()[1])
                        for z in (box.GetMin()[2], box.GetMax()[2])])
    loc = (Rh.T @ (corners - ph).T).T
    print(f"{name:<18} bounding box in the hand frame: min {np.round(loc.min(0), 4).tolist()}  max {np.round(loc.max(0), 4).tolist()}", flush=True)
print("finger joint positions now:", np.round(np.asarray(art.get_joint_positions()).flatten()[7:], 4).tolist(), flush=True)
simulation_app.close()
