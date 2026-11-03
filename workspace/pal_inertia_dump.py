#!/usr/bin/env python3
"""Full per-body dump: mass, centre of mass and inertia, from three places at once.

  articulation  what PhysX is simulating   (tensor API)
  USD prim      what the asset authors     (UsdPhysics.MassAPI attributes)
  URDF          what the Pinocchio twin will be built from

Plus a round-trip test on the inertia setter, because the articulation reports ~1e-6
kg*m^2 on every body and that is either a unit convention or a genuinely inertia-free
asset. Those two possibilities have very different consequences and guessing between them
is not acceptable.
"""
import csv
import json
import xml.etree.ElementTree as ET

import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_inertia_dump.py running", flush=True)

from pxr import UsdPhysics, Usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation
import pal_config as cfg

world = World(stage_units_in_meters=1.0, physics_dt=1/1000.0, rendering_dt=1/30.0)
add_reference_to_stage(usd_path=get_assets_root_path() + cfg.ROBOT_USD_SUBPATH,
                       prim_path=cfg.ROBOT_PRIM)
art = Articulation(cfg.ROBOT_PRIM)
world.reset(); art.initialize()
for _ in range(60):
    world.step(render=False)
bodies = list(art.body_names)
B = len(bodies)

masses = np.asarray(art.get_body_masses()).reshape(B)
inert = np.asarray(art.get_body_inertias()).reshape(B, 9)
com_p, com_q = art.get_body_coms()
com_p = np.asarray(com_p).reshape(B, 3)

# ── USD authored attributes ───────────────────────────────────────────────────────────
usd = {}
for b in bodies:
    prim = world.stage.GetPrimAtPath(f"{cfg.ROBOT_PRIM}/{b}")
    e = {}
    if prim.IsValid():
        for attr, key in (("physics:mass", "mass"),
                          ("physics:centerOfMass", "com"),
                          ("physics:diagonalInertia", "diag"),
                          ("physics:principalAxes", "axes")):
            a = prim.GetAttribute(attr)
            v = a.Get() if a and a.IsValid() else None
            e[key] = (list(v) if hasattr(v, "__len__") else v) if v is not None else None
    usd[b] = e

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
        "com": [float(x) for x in o.get("xyz").split()] if o is not None and o.get("xyz") else [0.0, 0.0, 0.0],
        "diag": [float(I.get("ixx")), float(I.get("iyy")), float(I.get("izz"))],
        "off": [float(I.get("ixy")), float(I.get("ixz")), float(I.get("iyz"))],
    }

print("\n" + "=" * 112)
print("  MASS  (kg)")
print("=" * 112)
print(f"  {'body':<20}{'articulation':>14}{'USD authored':>14}{'URDF':>12}{'art/URDF':>11}")
for i, b in enumerate(bodies):
    u = U.get(b, {}).get("mass")
    ua = usd[b].get("mass")
    ratio = masses[i]/u if u else float('nan')
    print(f"  {b:<20}{masses[i]:>14.6f}"
          f"{(f'{ua:.6f}' if ua is not None else '—'):>14}"
          f"{(f'{u:.6f}' if u else '—'):>12}{ratio:>11.3f}")
print(f"  {'TOTAL':<20}{masses.sum():>14.4f}{'':>14}"
      f"{sum(v['mass'] for v in U.values()):>12.4f}")

print("\n" + "=" * 112)
print("  CENTRE OF MASS  (m, body frame)")
print("=" * 112)
print(f"  {'body':<20}{'articulation xyz':>30}{'USD authored xyz':>30}{'URDF xyz':>30}")
com_diff = []
for i, b in enumerate(bodies):
    uc = U.get(b, {}).get("com")
    ua = usd[b].get("com")
    f = lambda v: ("[" + ",".join(f"{x:+.4f}" for x in v) + "]") if v is not None else "—"
    print(f"  {b:<20}{f(com_p[i]):>30}{f(ua):>30}{f(uc):>30}")
    if uc is not None:
        com_diff.append((b, float(np.linalg.norm(com_p[i] - np.array(uc)))))

print("\n" + "=" * 112)
print("  INERTIA DIAGONAL  (kg*m^2)")
print("=" * 112)
print(f"  {'body':<20}{'articulation ixx,iyy,izz':>36}{'USD authored':>30}{'URDF':>26}")
for i, b in enumerate(bodies):
    a = [inert[i][0], inert[i][4], inert[i][8]]
    ud = usd[b].get("diag")
    uu = U.get(b, {}).get("diag")
    f = lambda v, p=6: ("[" + ",".join(f"{x:.{p}f}" for x in v) + "]") if v is not None else "—"
    print(f"  {b:<20}{f(a):>36}{f(ud):>30}{f(uu):>26}")

print("\n" + "=" * 112)
print("  ROUND TRIP — is 1e-6 a unit convention, or a genuinely inertia-free asset?")
print("=" * 112)
probe = np.zeros((B, 3), dtype=np.float32)
probe[:] = [0.1, 0.2, 0.3]
try:
    art.set_body_inertias(probe.reshape(1, B, 3))
except Exception:
    full = np.zeros((B, 9), dtype=np.float32)
    full[:, 0], full[:, 4], full[:, 8] = 0.1, 0.2, 0.3
    art.set_body_inertias(full.reshape(1, B, 9))
for _ in range(10):
    world.step(render=False)
back = np.asarray(art.get_body_inertias()).reshape(B, 9)
print(f"  wrote diag [0.1, 0.2, 0.3] to every body")
print(f"  read back body 0: [{back[0][0]:.6g}, {back[0][4]:.6g}, {back[0][8]:.6g}]")
if np.isclose(back[0][0], 0.1, rtol=1e-3):
    print("  ROUND TRIP EXACT -> the API is in kg*m^2 and the asset genuinely")
    print("  ships ~1e-6, i.e. its links have essentially NO rotational inertia.")
else:
    r = back[0][0] / 0.1 if back[0][0] else float('nan')
    print(f"  MISMATCH, readback/written = {r:.6g} -> there is a conversion; the")
    print(f"  asset's apparent 1e-6 may be an artefact of reading the wrong units.")

# ── Export for the analysis side ──────────────────────────────────────────────────────
rows = []
for i, b in enumerate(bodies):
    u = U.get(b, {})
    rows.append({
        "body": b,
        "art_mass": float(masses[i]),
        "urdf_mass": u.get("mass"),
        "art_com_x": float(com_p[i][0]), "art_com_y": float(com_p[i][1]), "art_com_z": float(com_p[i][2]),
        "urdf_com_x": (u.get("com") or [None]*3)[0],
        "urdf_com_y": (u.get("com") or [None]*3)[1],
        "urdf_com_z": (u.get("com") or [None]*3)[2],
        "art_ixx": float(inert[i][0]), "art_iyy": float(inert[i][4]), "art_izz": float(inert[i][8]),
        "urdf_ixx": (u.get("diag") or [None]*3)[0],
        "urdf_iyy": (u.get("diag") or [None]*3)[1],
        "urdf_izz": (u.get("diag") or [None]*3)[2],
    })
with open("/workspace/out/inertial_dump.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader(); w.writerows(rows)
with open("/workspace/out/inertial_dump.json", "w") as f:
    json.dump({"bodies": bodies, "rows": rows, "usd_authored": usd, "urdf": U}, f, indent=2)

print("\n  COM distance articulation vs URDF (m):")
for b, d in com_diff:
    print(f"    {b:<22}{d:>10.5f}{'   <<< >1mm' if d > 1e-3 else ''}")
print("\n  written: out/inertial_dump.csv  and  out/inertial_dump.json")
simulation_app.close()
