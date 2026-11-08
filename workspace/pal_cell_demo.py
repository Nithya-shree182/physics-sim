#!/usr/bin/env python3
"""The conveyor-cell demo: a UR10e picks three cardboard boxes off an infeed table with a suction
tool and sets them on a running conveyor, over and over, in a window.

    ./start_sim_docker.sh                      # this, windowed
    HEADLESS=1 ./start_sim_docker.sh           # no window
    PAL_BOX_KG=2.5 ./start_sim_docker.sh       # heavier boxes
    PAL_STYLE=trapezoidal ./start_sim_docker.sh
    PAL_SPEED=0.5 ./start_sim_docker.sh        # fraction of the trustworthy speed band
    PAL_CAM=iso_close ./start_sim_docker.sh    # camera: iso (default), iso_high, iso_close, side, top

What it is for: watching the cell run, and judging how it looks and how smoothly it moves. It is
not a data run — nothing is written and nothing is scored. `pal_live.py` is the same cell driven
by the dashboard; `tools/pal_suite.sh` is the one that records data.

Smoothness. The old live script drew one frame every 25 physics steps (10 fps of a 250 Hz
simulation), which is what made the window look like it was stuttering. Here the render rate is a
separate knob from the physics rate: PAL_RENDER_HZ frames a second, default 60, with the physics
still stepping at PAL_PHYSICS_HZ. `rendering_dt` stays pinned to the physics step, so one rendered
call is still exactly one physics step and the time-warp bug of 2026-11-04 (REVIEW F13) cannot
come back.
"""
import os
import time

from pal_app import HEADLESS, start_app

simulation_app = start_app("pal_cell_demo.py")

import numpy as np                                             # noqa: E402

import pal_cell as C                                           # noqa: E402
import pal_config as cfg                                       # noqa: E402
import pal_tasks as K                                          # noqa: E402
from pal_plant import build_plant                              # noqa: E402
from pal_runner import KD_NORMAL, KP_NORMAL, Runner            # noqa: E402

HZ = float(os.environ.get("PAL_PHYSICS_HZ", 250.0))
SHOTS = os.environ.get("PAL_SHOTS", "")        # e.g. "3,8,13" - sim times (s) to save a PNG at
# 30 fps keeps the cell at about real time on an RTX 5090 laptop; 60 fps looked smoother per
# frame but ran the whole cell at 0.65x, which reads as sluggish. Measured 2026-11-08.
RENDER_HZ = float(os.environ.get("PAL_RENDER_HZ", "30"))
STYLE = os.environ.get("PAL_STYLE", "s_curve")
BOX_KG = float(os.environ.get("PAL_BOX_KG", C.BOX_DEFAULT_KG))
SPEED = float(os.environ.get("PAL_SPEED", "0.7"))              # of the <=50% velocity band
REALTIME = os.environ.get("PAL_REALTIME", "1") != "0"
CYCLES = int(os.environ.get("PAL_CYCLES", "0"))                # 0 = forever

BAND = cfg.SPEED_BAND           # 1.0 on the speed knob is half the datasheet velocity limit (D1)


def build_task(style=STYLE):
    """The three-box cycle. One definition, in pal_tasks.cell_cycle, shared with the live server
    and the recorded suites, so a cycle time quoted from one is a cycle time about the others."""
    return K.cell_cycle(style, speed=SPEED, band=BAND)


def _ensure_view(plant, where="start"):
    """Make sure the articulation still has a physics view, and rebuild it if not.

    Isaac drops the view now and then — the same flakiness this project already works around at
    boot in tools/run_sim.sh. Without this the next read of a joint torque raises an opaque
    "cannot reshape array of size 1" and the run dies; with it the cell carries on."""
    art = plant.art
    if getattr(art, "_physics_view", None) is not None:
        return True
    print(f"[PAL] physics view missing at {where}; re-initialising the articulation", flush=True)
    try:
        art.initialize()
    except Exception as exc:
        print(f"[PAL] re-initialise failed: {exc!r}", flush=True)
        return False
    ok = getattr(art, "_physics_view", None) is not None
    print(f"[PAL] re-initialise {'succeeded' if ok else 'did NOT restore the view'}", flush=True)
    return ok


def main():
    plant = build_plant(HZ, scene=True, cell=True)
    cell = plant.cell
    tool = cell.mount_tool(cfg.ROBOT_PRIM)
    cell.frame_camera()

    render_every = max(1, round(HZ / RENDER_HZ)) if RENDER_HZ > 0 else 0
    runner = Runner(plant, "/workspace/out/cell", mj_every=0, render_every=render_every)
    art = plant.art

    task = build_task()
    print(f"[PAL] cell demo ready: {cfg.ROBOT.name}, {C.N_BOXES} boxes of {BOX_KG} kg, style {STYLE}, "
          f"cycle {task.duration:.1f} s, physics {HZ:.0f} Hz, render {HZ / render_every:.0f} fps", flush=True)

    _ensure_view(plant)
    art.set_joint_positions(np.array(cfg.HOME_JOINTS, np.float32).reshape(1, -1))
    art.set_joint_velocities(np.zeros((1, cfg.DOF), np.float32))
    cell.reset_boxes(kg=BOX_KG)

    tau_lim = np.asarray(cfg.TAU_MAX, float)
    shot_times = sorted(float(v) for v in SHOTS.split(",") if v.strip())
    shots_done = []
    if shot_times:
        os.makedirs("/workspace/out/cell", exist_ok=True)
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

    cycle, t_wall0 = 0, time.time()
    while simulation_app.is_running() and (CYCLES == 0 or cycle < CYCLES):
        cycle += 1
        cell.reset_boxes(kg=BOX_KG)
        tool.set_vacuum(False, 0.0)
        _ensure_view(plant, f"cycle {cycle}")
        print(f"[PAL] cycle {cycle} start: boxes at "
              + ", ".join(f"{b.name}={np.round(b.position(), 3).tolist()}" for b in cell.boxes), flush=True)
        pending = list(task.events)
        n_steps = int(task.duration * HZ)
        sim_t0 = time.time()
        held_name = None

        t_ctl = t_phys = t_cell = t_tool = 0.0
        for k in range(n_steps + 1):
            t = k / HZ
            rendering = bool(render_every) and k % render_every == 0
            while pending and pending[0][0] <= t:
                _, kind, idx = pending.pop(0)
                if kind == "vacuum_on":
                    tool.set_vacuum(True, t)
                elif kind == "vacuum_off":
                    tool.set_vacuum(False, t)

            _w = time.perf_counter()
            try:
                tau, *_ = runner._command(task.traj, t, KP_NORMAL, KD_NORMAL, True)
            except (ValueError, AttributeError):
                # Isaac occasionally drops the articulation's physics view mid-run. Rebuild it and
                # carry on rather than losing the whole cycle; the step is simply skipped.
                if not _ensure_view(plant, f"t={t:.2f}s"):
                    raise
                continue
            runner._apply(tau)
            t_ctl += time.perf_counter() - _w

            _w = time.perf_counter()
            plant.world.step(render=rendering)
            t_phys += time.perf_counter() - _w

            _w = time.perf_counter()
            change = tool.update(t, [b for b in cell.boxes])
            t_tool += time.perf_counter() - _w
            if change == "grip":
                held_name = tool.held
                for b in cell.boxes:
                    if b.path == held_name:
                        print(f"[PAL] cycle {cycle}: gripped {b.name} at t={t:.2f}s", flush=True)
            elif change == "release":
                for b in cell.boxes:
                    if b.path == held_name:
                        b.placed = True
                        print(f"[PAL] cycle {cycle}: placed {b.name} on the belt at t={t:.2f}s", flush=True)
                held_name = None

            _w = time.perf_counter()
            cell.step(1.0 / HZ, tau_frac=np.abs(tau) / tau_lim, visual=rendering)
            t_cell += time.perf_counter() - _w

            if shot_times and cycle == 1 and rendering:
                due = [v for v in shot_times if v <= t and v not in shots_done]
                for v in due:
                    shots_done.append(v)
                    capture_viewport_to_file(get_active_viewport(), f"/workspace/out/cell/shot_{v:g}s.png")
                    print(f"[PAL] saved out/cell/shot_{v:g}s.png", flush=True)

            if REALTIME:
                ahead = t - (time.time() - sim_t0)
                if ahead > 0:
                    time.sleep(min(ahead, 0.05))

        placed = sum(1 for b in cell.boxes if b.placed)
        wall = time.time() - sim_t0
        print(f"[PAL] cycle {cycle} done in {wall:.1f} s wall ({task.duration:.1f} s sim, "
              f"{task.duration / wall:.2f}x real time), {placed}/{len(cell.boxes)} boxes placed", flush=True)
        print(f"[PAL] where the time went: physics+render {t_phys:.1f}s | controller {t_ctl:.1f}s | "
              f"cell {t_cell:.1f}s | suction {t_tool:.1f}s | other {wall - t_phys - t_ctl - t_cell - t_tool:.1f}s",
              flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
