#!/usr/bin/env python3
"""Ground truth for the twin: read the PLANT's inertial parameters and diff them
against the URDF the Pinocchio model will be built from.

Pinocchio needs the mass, centre of mass and inertia tensor of the arm that is actually
being simulated. We have been assuming the URDF describes that arm. It is two different
files from two different sources, and a 0.8 N*m discrepancy at the home pose already
suggests they disagree somewhere.

Any difference found here is a twin/plant mismatch nobody chose, sitting underneath every
residual. Output is written as JSON so the analysis side can load the plant's real numbers
directly rather than trusting the URDF.
"""
import json
import xml.etree.ElementTree as ET

import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_inertia_audit.py running", flush=True)

from pxr import UsdPhysics, Usd, Gf
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
print(f"\n  articulation reports {len(bodies)} bodies")

# ── Plant side: read from the live articulation ──────────────────────────────────────
plant = {}
masses = np.asarray(art.get_body_masses()).flatten()
try:
    inertias = np.asarray(art.get_body_inertias()).reshape(len(bodies), -1)
except Exception as e:
    inertias = None
    print(f"  [warn] get_body_inertias unavailable: {e}")
coms = None
try:
    raw = art.get_body_coms()
    if isinstance(raw, (tuple, list)) and len(raw) == 2:
        coms = np.asarray(raw[0]).reshape(-1, 3)       # positions; [1] is orientation
    else:
        coms = np.asarray(raw).reshape(-1, 3)
    print(f"  body COM read OK, shape {coms.shape}")
except Exception as e:
    print(f"  [warn] get_body_coms unavailable: {e}")

for i, name in enumerate(bodies):
    entry = {"mass": float(masses[i])}
    if inertias is not None:
        entry["inertia_flat"] = [float(x) for x in inertias[i]]
    if coms is not None and i < len(coms):
        entry["com"] = [float(x) for x in coms[i][:3]]
    # Cross-read the USD MassAPI, which is what the asset actually authors.
    prim = world.stage.GetPrimAtPath(f"{cfg.ROBOT_PRIM}/{name}")
    if prim.IsValid():
        m_api = UsdPhysics.MassAPI(prim)
        a = m_api.GetMassAttr()
        entry["usd_authored_mass"] = float(a.Get()) if a and a.Get() is not None else None
        c = m_api.GetCenterOfMassAttr()
        entry["usd_com"] = [float(v) for v in c.Get()] if c and c.Get() is not None else None
        d = m_api.GetDiagonalInertiaAttr()
        entry["usd_diag_inertia"] = [float(v) for v in d.Get()] if d and d.Get() is not None else None
    plant[name] = entry

# ── URDF side ─────────────────────────────────────────────────────────────────────────
URDF = "/workspace/assets/panda_arm_hand.urdf"
urdf = {}
try:
    root = ET.parse(URDF).getroot()
    for link in root.findall("link"):
        inert = link.find("inertial")
        if inert is None:
            continue
        m = inert.find("mass")
        o = inert.find("origin")
        it = inert.find("inertia")
        urdf[link.get("name")] = {
            "mass": float(m.get("value")) if m is not None else None,
            "com": [float(x) for x in o.get("xyz").split()] if o is not None and o.get("xyz") else None,
            "inertia": {k: float(it.get(k)) for k in
                        ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")} if it is not None else None,
        }
except Exception as e:
    print(f"  [warn] could not parse URDF at {URDF}: {e}")

print("\n" + "=" * 84)
print("  MASS — plant (simulator) vs URDF")
print("=" * 84)
print(f"  {'body':<22}{'plant kg':>11}{'URDF kg':>11}{'diff':>11}{'diff %':>10}")
tot_p = tot_u = 0.0
mismatches = []
for name in bodies:
    p = plant[name]["mass"]
    u = urdf.get(name, {}).get("mass")
    tot_p += p
    if u is None:
        print(f"  {name:<22}{p:>11.6f}{'—':>11}{'—':>11}{'—':>10}   not in URDF")
        continue
    tot_u += u
    d = p - u
    pct = 100 * d / u if u else 0.0
    flag = "   <<<" if abs(pct) > 1.0 else ""
    if abs(pct) > 1.0:
        mismatches.append((name, p, u, pct))
    print(f"  {name:<22}{p:>11.6f}{u:>11.6f}{d:>11.6f}{pct:>9.2f}%{flag}")
print(f"  {'TOTAL':<22}{tot_p:>11.4f}{tot_u:>11.4f}{tot_p-tot_u:>11.4f}")

print("\n" + "=" * 84)
print("  CENTRE OF MASS — plant vs URDF (metres)")
print("=" * 84)
print(f"  {'body':<22}{'plant xyz':>30}{'URDF xyz':>30}{'|d|':>9}")
com_bad = []
for name in bodies:
    pc = plant[name].get("com") or plant[name].get("usd_com")
    uc = urdf.get(name, {}).get("com")
    if pc is None or uc is None:
        continue
    d = np.linalg.norm(np.array(pc[:3]) - np.array(uc))
    if d > 1e-3:
        com_bad.append((name, pc[:3], uc, d))
    ps = "[" + ", ".join(f"{v:+.4f}" for v in pc[:3]) + "]"
    us = "[" + ", ".join(f"{v:+.4f}" for v in uc) + "]"
    print(f"  {name:<22}{ps:>30}{us:>30}{d:>9.5f}")

print("\n" + "=" * 84)
print("  INERTIA DIAGONAL — plant vs URDF (kg*m^2)")
print("=" * 84)
print(f"  {'body':<22}{'plant ixx':>11}{'URDF ixx':>11}{'plant izz':>11}{'URDF izz':>11}")
for i, name in enumerate(bodies):
    pi = plant[name].get("inertia_flat")
    ui = urdf.get(name, {}).get("inertia")
    if not pi or not ui:
        continue
    pix = pi[0]
    piz = pi[8] if len(pi) >= 9 else (pi[2] if len(pi) >= 3 else float("nan"))
    print(f"  {name:<22}{pix:>11.6f}{ui['ixx']:>11.6f}{piz:>11.6f}{ui['izz']:>11.6f}")

print("\n" + "=" * 84)
print("  VERDICT")
print("=" * 84)
if mismatches:
    print(f"  {len(mismatches)} body/bodies differ in MASS by more than 1%:")
    for n, p, u, pct in mismatches:
        print(f"    {n}: plant {p:.4f} kg vs URDF {u:.4f} kg  ({pct:+.2f}%)")
else:
    print("  masses agree within 1% on every body")
if com_bad:
    print(f"  {len(com_bad)} body/bodies differ in CENTRE OF MASS by more than 1 mm:")
    for n, pc, uc, d in com_bad:
        print(f"    {n}: offset {1000*d:.2f} mm")
else:
    print("  centres of mass agree within 1 mm on every body")
print("\n  Any difference above is a twin/plant mismatch that nobody chose. It sits")
print("  underneath every residual and will be attributed to payload or to a fault.")

out = {"plant": plant, "urdf": urdf, "bodies": bodies,
       "usd_asset": cfg.ROBOT_USD_SUBPATH, "urdf_file": URDF}
with open("/workspace/out/inertial_parameters.json", "w") as f:
    json.dump(out, f, indent=2, default=str)
print("\n  plant parameters written to out/inertial_parameters.json")
simulation_app.close()
