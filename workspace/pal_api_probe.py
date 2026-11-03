"""Throwaway introspection: which Isaac APIs apply an external force at the gripper?"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_api_probe.py running", flush=True)
import inspect
import numpy as np
from pal_plant import build_plant
p = build_plant(1000.0)
a = p.art
pv = a._physics_view
for obj, tag in ((a, "art"), (pv, "pv")):
    for n in sorted(dir(obj)):
        if ("force" in n.lower() or "wrench" in n.lower()) and not n.startswith("_") and "friction" not in n.lower():
            f = getattr(obj, n)
            print(f"\n=== {tag}.{n}{inspect.signature(f) if callable(f) else ''}")
            print((inspect.getdoc(f) or "")[:700])
print("\nbody_names:", list(a.body_names))
simulation_app.close()
