#!/usr/bin/env python3
"""The conveyor-cell demo: a UR10e picks three cardboard boxes off an infeed table with a suction
tool and sets them on a running conveyor, over and over, in a window.

    ./start_sim_docker.sh                      # this, windowed
    HEADLESS=1 ./start_sim_docker.sh           # no window
    PAL_BOX_KG=2.5 ./start_sim_docker.sh       # heavier boxes
    PAL_STYLE=trapezoidal ./start_sim_docker.sh
    PAL_SPEED=0.5 ./start_sim_docker.sh        # fraction of the trustworthy speed band

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
import pal_suction as SU                                       # noqa: E402
import pal_tasks as K                                          # noqa: E402
from pal_plant import build_plant                              # noqa: E402
from pal_runner import KD_NORMAL, KP_NORMAL, Runner            # noqa: E402

HZ = float(os.environ.get("PAL_PHYSICS_HZ", 250.0))
RENDER_HZ = float(os.environ.get("PAL_RENDER_HZ", "60"))
STYLE = os.environ.get("PAL_STYLE", "s_curve")
BOX_KG = float(os.environ.get("PAL_BOX_KG", C.BOX_DEFAULT_KG))
SPEED = float(os.environ.get("PAL_SPEED", "0.7"))              # of the <=50% velocity band
REALTIME = os.environ.get("PAL_REALTIME", "1") != "0"
CYCLES = int(os.environ.get("PAL_CYCLES", "0"))                # 0 = forever

BAND = 0.5                      # 1.0 on the speed knob is half the datasheet velocity limit (D1)


def build_task(style=STYLE):
    """The three-box cycle, aimed at the cup faces rather than the flange."""
    top_z = C.WORK_Z + C.BOX[2]
    return K.palletize(
        style,
        [(x, C.BOX_Y) for x in C.BOX_X[:C.N_BOXES]],
        C.PLACE_XY,
        box_top_z=top_z,
        place_top_z=top_z,
        cup_offset=(0.0, 0.0, SU.CUP_PLANE_Z),
        vscale=BAND * SPEED, ascale=BAND * SPEED,
        name=f"cell_{style}")


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

    art.set_joint_positions(np.array(cfg.HOME_JOINTS, np.float32).reshape(1, -1))
    art.set_joint_velocities(np.zeros((1, cfg.DOF), np.float32))
    cell.reset_boxes(kg=BOX_KG)

    tau_lim = np.asarray(cfg.TAU_MAX, float)
    cycle, t_wall0 = 0, time.time()
    while simulation_app.is_running() and (CYCLES == 0 or cycle < CYCLES):
        cycle += 1
        cell.reset_boxes(kg=BOX_KG)
        tool.set_vacuum(False, 0.0)
        pending = list(task.events)
        n_steps = int(task.duration * HZ)
        sim_t0 = time.time()
        held_name = None

        for k in range(n_steps + 1):
            t = k / HZ
            while pending and pending[0][0] <= t:
                _, kind, idx = pending.pop(0)
                if kind == "vacuum_on":
                    tool.set_vacuum(True, t)
                elif kind == "vacuum_off":
                    tool.set_vacuum(False, t)

            tau, *_ = runner._command(task.traj, t, KP_NORMAL, KD_NORMAL, True)
            runner._apply(tau)
            plant.world.step(render=bool(render_every) and k % render_every == 0)

            change = tool.update(t, [b for b in cell.boxes])
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

            cell.step(1.0 / HZ, tau_frac=np.abs(tau) / tau_lim)

            if REALTIME:
                ahead = t - (time.time() - sim_t0)
                if ahead > 0:
                    time.sleep(min(ahead, 0.05))

        placed = sum(1 for b in cell.boxes if b.placed)
        print(f"[PAL] cycle {cycle} done in {time.time() - sim_t0:.1f} s wall "
              f"({task.duration:.1f} s sim), {placed}/{len(cell.boxes)} boxes placed", flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
