"""Start Isaac Sim, headless or in a window, from one place.

run_sim.sh passes ISAAC_SIM_HEADLESS=1 by default and 0 when you run `HEADLESS=0 tools/run_sim.sh ...`. Every script that used to
call SimulationApp({"headless": True}) directly now calls start_app(), so the same script runs either way. Headless keeps the exact
settings it always had; a window adds a size and the renderer the old scene script used. Physics does not depend on rendering, and
tools/compare_runs.py shows it: a windowed run matches the headless one.
"""
import os

HEADLESS = os.environ.get("ISAAC_SIM_HEADLESS", "1") != "0"


def start_app(name):
    from isaacsim import SimulationApp
    cfg = {"headless": True} if HEADLESS else {"headless": False, "width": 1280, "height": 720,
                                               "renderer": "RaytracedLighting", "anti_aliasing": 3}
    app = SimulationApp(cfg)
    print(f"[PAL] {name} running ({'headless' if HEADLESS else 'windowed'})", flush=True)
    return app
