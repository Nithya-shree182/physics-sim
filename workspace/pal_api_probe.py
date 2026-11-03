"""Print the signature and docstring of the articulation friction APIs. Throwaway introspection."""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_api_probe.py running", flush=True)
import inspect
import numpy as np
import pal_config as cfg
from pal_plant import build_plant
p = build_plant(1000.0)
a = p.art
import inspect
pv = getattr(a, "_physics_view", None)
print("physics view:", type(pv))
for n in sorted(dir(pv)):
    if "friction" in n.lower():
        f = getattr(pv, n)
        print(f"\n=== pv.{n}")
        print((inspect.getdoc(f) or str(f))[:1100])
try:
    v = np.asarray(pv.get_dof_friction_properties())
    print("\nget_dof_friction_properties shape", v.shape, "first joints:\n", v.reshape(-1, v.shape[-1])[:3])
except Exception as e:
    print("pv.get_dof_friction_properties failed:", repr(e))
simulation_app.close()
