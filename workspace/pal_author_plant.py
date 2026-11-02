#!/usr/bin/env python3
"""Author the plant's inertial parameters explicitly from the URDF.

Neither source is authoritative. Franka publishes no dynamic parameters for the Panda —
Gaz et al. 2019 exists precisely because of that, and says so in its opening paragraph.
The URDF carries franka_description's values; the USD carries NVIDIA's; they disagree by
up to 347 percent per link. Whichever is closer to a real robot, inheriting the difference
by accident is the problem: the proposal's section 9 calls for a twin/plant mismatch that
is CHOSEN and quantified, and a 347 percent unexplained offset is not that.

So the plant is authored from the URDF. Then the twin and the plant share one parameter
set by construction, the deliberate perturbation sits on top of it where it can be
measured, and the Gaz friction values are used alongside the inertial parameters they were
identified next to rather than against a different robot's.

Also verifies the readbacks, since the first audit's COM and inertia columns were
implausible (all-zero COMs, ~1e-6 inertias) and must not be built on.
"""
import json
import xml.etree.ElementTree as ET

import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation
import pal_config as cfg

N = 7
world = World(stage_units_in_meters=1.0, physics_dt=1/1000.0, rendering_dt=1/30.0)
add_reference_to_stage(usd_path=get_assets_root_path() + cfg.ROBOT_USD_SUBPATH,
                       prim_path=cfg.ROBOT_PRIM)
art = Articulation(cfg.ROBOT_PRIM)
world.reset(); art.initialize()
for _ in range(60):
    world.step(render=False)
bodies = list(art.body_names)
home = np.concatenate([cfg.HOME_JOINTS, [0.04, 0.04]])


def settle(steps=600):
    art.set_joint_positions(home); art.set_joint_position_targets(home)
    for _ in range(steps):
        world.step(render=False)


print("\n" + "=" * 80)
print("  1 — RAW READBACK SHAPES   (the previous audit mis-parsed these)")
print("=" * 80)
for name in ("get_body_masses", "get_body_inertias", "get_body_coms", "get_body_inv_inertias"):
    fn = getattr(art, name, None)
    if fn is None:
        print(f"  {name:<24} absent")
        continue
    try:
        r = fn()
        if isinstance(r, (tuple, list)):
            print(f"  {name:<24} tuple of {len(r)}: "
                  f"{[np.asarray(x).shape for x in r]}")
        else:
            a = np.asarray(r)
            print(f"  {name:<24} shape {a.shape}  range [{a.min():.3e}, {a.max():.3e}]")
    except Exception as e:
        print(f"  {name:<24} raised {e}")

inert = np.asarray(art.get_body_inertias())
print(f"\n  get_body_inertias first body, raw: {np.asarray(inert).flatten()[:9]}")
print("  A 2.8 kg link should have principal inertias of order 1e-3 to 1e-1 kg*m^2.")

# ── URDF ──────────────────────────────────────────────────────────────────────────────
root = ET.parse("/workspace/assets/panda_arm_hand.urdf").getroot()
U = {}
for link in root.findall("link"):
    it = link.find("inertial")
    if it is None:
        continue
    m = it.find("mass"); o = it.find("origin"); I = it.find("inertia")
    U[link.get("name")] = {
        "mass": float(m.get("value")),
        "com": [float(x) for x in o.get("xyz").split()] if o is not None and o.get("xyz") else [0, 0, 0],
        "I": np.array([[float(I.get("ixx")), float(I.get("ixy")), float(I.get("ixz"))],
                       [float(I.get("ixy")), float(I.get("iyy")), float(I.get("iyz"))],
                       [float(I.get("ixz")), float(I.get("iyz")), float(I.get("izz"))]]),
    }

print("\n" + "=" * 80)
print("  2 — BASELINE: g(q) at home with the ASSET's own parameters")
print("=" * 80)
settle()
g_asset = np.asarray(art.get_generalized_gravity_forces()).flatten()[:N].copy()
m_asset = np.asarray(art.get_body_masses()).flatten().copy()
print(f"  g(q) = {np.round(g_asset, 4)}")
print(f"  total mass = {m_asset.sum():.4f} kg")

print("\n" + "=" * 80)
print("  3 — AUTHOR the plant from the URDF")
print("=" * 80)
new_m = np.array([U[b]["mass"] if b in U else float(m_asset[i])
                  for i, b in enumerate(bodies)], dtype=np.float32)
ok_m = ok_i = ok_c = False
try:
    art.set_body_masses(new_m.reshape(1, -1))
    ok_m = True
    print(f"  masses set:  {np.round(new_m, 4)}")
except Exception as e:
    print(f"  [FAIL] set_body_masses: {e}")

try:
    # Principal (diagonal) inertia per body, in the order the setter expects.
    diag = np.array([[U[b]["I"][0, 0], U[b]["I"][1, 1], U[b]["I"][2, 2]] if b in U else [1e-3]*3
                     for b in bodies], dtype=np.float32)
    try:
        art.set_body_inertias(diag.reshape(1, len(bodies), 3))
    except Exception:
        full = np.array([U[b]["I"].flatten() if b in U else np.eye(3).flatten()*1e-3
                         for b in bodies], dtype=np.float32)
        art.set_body_inertias(full.reshape(1, len(bodies), 9))
    ok_i = True
    print(f"  inertias set (diagonal, first body: {diag[0]})")
except Exception as e:
    print(f"  [FAIL] set_body_inertias: {e}")

try:
    com = np.array([U[b]["com"] if b in U else [0, 0, 0] for b in bodies], dtype=np.float32)
    quat = np.tile(np.array([1, 0, 0, 0], dtype=np.float32), (len(bodies), 1))
    art.set_body_coms(com.reshape(1, len(bodies), 3), quat.reshape(1, len(bodies), 4))
    ok_c = True
    print(f"  COMs set (first body: {com[0]})")
except Exception as e:
    print(f"  [FAIL] set_body_coms: {e}")

print("\n" + "=" * 80)
print("  4 — VERIFY by readback and by the change in g(q)")
print("=" * 80)
settle()
m_back = np.asarray(art.get_body_masses()).flatten()
g_urdf = np.asarray(art.get_generalized_gravity_forces()).flatten()[:N]
print(f"  mass readback matches what was written: "
      f"{np.allclose(m_back[:len(new_m)], new_m, atol=1e-4)}")
print(f"  total mass now {m_back.sum():.4f} kg (URDF total 17.6218)")
print(f"\n  g(q) with ASSET params : {np.round(g_asset, 4)}")
print(f"  g(q) with URDF  params : {np.round(g_urdf, 4)}")
d = g_urdf - g_asset
print(f"  difference             : {np.round(d, 4)}")
print(f"  as %% of torque limit   : "
      f"{np.round(100*np.abs(d)/cfg.TAU_MAX, 2)}")
print(f"\n  worst joint {int(np.argmax(np.abs(d)/cfg.TAU_MAX))+1}: "
      f"{100*np.max(np.abs(d)/cfg.TAU_MAX):.2f}% of its limit")
print(f"  for scale, the section 7.2 calibration tolerance is 5%")
print(f"  and 1 kg of payload is worth 4.63 N*m on joint 4, so this offset is")
print(f"  equivalent to about {abs(d[3])/4.6306:.3f} kg of phantom payload")

out = {"bodies": bodies,
       "asset_masses": m_asset.tolist(), "urdf_masses": new_m.tolist(),
       "g_asset": g_asset.tolist(), "g_urdf": g_urdf.tolist(),
       "set_ok": {"masses": ok_m, "inertias": ok_i, "coms": ok_c}}
with open("/workspace/out/plant_authoring.json", "w") as f:
    json.dump(out, f, indent=2)
print("\n  written to out/plant_authoring.json")
simulation_app.close()
