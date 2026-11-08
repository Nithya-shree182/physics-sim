#!/usr/bin/env python3
"""Play suites of scenarios (pal_suites.py) in one Isaac Sim process, headless or in a window.

    tools/pal_suite.sh o5                 # headless, then convert and check
    tools/pal_suite.sh all --window       # every objective's tests, drawn in a window so the arm is seen moving
    PAL_SUITE=o3,o4 HEADLESS=0 tools/run_sim.sh pal_run_suite.py      # the raw form

PAL_SUITE: comma-separated suite names, or `all`. PAL_RENDER_EVERY: draw a frame every N physics steps (windowed default 33).
One plant is built for the whole process; every run rewrites and reads back the inertials and friction first, so the runs do not
depend on their order.
"""
import os
import time

from pal_app import HEADLESS, start_app
simulation_app = start_app("pal_run_suite.py")

import pal_config as cfg
import pal_suites as S
from pal_plant import build_plant
from pal_runner import Runner

HZ = float(os.environ.get("PAL_PHYSICS_HZ", cfg.PHYSICS_HZ))
want = os.environ.get("PAL_SUITE", "all")
names = S.ORDER if want == "all" else [n.strip() for n in want.split(",") if n.strip()]
for n in names:
    if n not in S.SUITES:
        raise SystemExit(f"unknown suite {n!r}; choose from {S.ORDER}")
render_every = int(os.environ.get("PAL_RENDER_EVERY", 0 if HEADLESS else 33))
MJ_EVERY = {"push": 1}                  # the push test wants a fresh Jacobian every step

FRAMES = os.environ.get("PAL_FRAMES", "")        # e.g. PAL_FRAMES=pick_s_curve_1kg saves viewport frames of that run in out/frames


def frame_hook(sc):
    """Save viewport PNGs of one named run at the grasp, mid-carry, release and the end of an ellipse, to check the scene by eye."""
    if HEADLESS or not FRAMES or sc.name not in FRAMES.split(","):
        return None
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
    os.makedirs("/workspace/out/frames", exist_ok=True)
    if sc.task is not None and sc.task.t_attach:
        marks = {"grasp": sc.task.t_attach - 0.1, "carry": 0.5 * (sc.task.t_attach + sc.task.t_release), "release": sc.task.t_release + 0.1}
    else:
        marks = {"a": 2.0, "b": 5.0, "c": 8.0}
    steps = {int(round(t * HZ)): n for n, t in marks.items()}

    def hook(k, t, st):
        if k in steps:
            capture_viewport_to_file(get_active_viewport(), f"/workspace/out/frames/{sc.name}_{steps[k]}.png")
    return hook


needs_cell = [n in S.NEEDS_CELL for n in names]
if any(needs_cell) and not all(needs_cell):
    raise SystemExit(f"suites needing the conveyor cell ({sorted(S.NEEDS_CELL)}) cannot share a process with the others; "
                     f"run them separately (tools/pal_suite.sh does)")
plant = build_plant(HZ, scene=not HEADLESS, cell=any(needs_cell))
viz = None
if not HEADLESS and not any(needs_cell):       # a really gripped box needs no drawn stand-in
    from pal_kinematics import Urdf
    from pal_view import PickScene
    viz = PickScene(Urdf(cfg.URDF_PATH))
t_all = time.time()
total = sum(len([r for r in S.SUITES[n][1]() if not os.environ.get('PAL_ONLY') or r.name in os.environ['PAL_ONLY'].split(',')]) for n in names)
done = 0
for name in names:
    out_dir, make = S.SUITES[name]
    runs = make()
    only = [x for x in os.environ.get("PAL_ONLY", "").split(",") if x]
    if only:                                        # PAL_ONLY=name1,name2 runs just those scenarios of the suite
        runs = [r for r in runs if r.name in only]
    off = int(os.environ.get("PAL_SEED_OFFSET", "0"))
    if off:                                         # PAL_SEED_OFFSET=N: same scenarios, different sensor noise, saved under a name ending _sN
        import dataclasses
        runs = [dataclasses.replace(r, name=f"{r.name}_s{off}", noise_seed=None if r.noise_seed is None else r.noise_seed + off) for r in runs]
    runner = Runner(plant, f"/workspace/{out_dir}", mj_every=MJ_EVERY.get(name, 10), render_every=render_every, viz=viz)
    print(f"\n=== suite {name}: {len(runs)} runs -> {out_dir} ({'headless' if HEADLESS else 'windowed'}) ===", flush=True)
    for sc in runs:
        t0 = time.time()
        log = runner.run(sc, on_step=frame_hook(sc))
        done += 1
        f = sc.fault
        what = "healthy" if f.type == "none" else f"{f.type} j{f.joint + 1} {f.param:g}"
        extra = f" +{sc.payload_kg:g} kg" if sc.payload_kg else ""
        print(f"  [{done:>3}/{total}] {sc.name:<40} {what + extra:<28} peak speed {log['speed_fraction']:>4.0%}  "
              f"{sc.duration_s / (time.time() - t0):4.2f}x real time", flush=True)
print(f"\n[PAL] all suites done in {time.time() - t_all:.0f} s", flush=True)
simulation_app.close()
