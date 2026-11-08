#!/usr/bin/env python3
"""The live simulator: a windowed (or headless) Isaac Sim that runs real cell cycles on command and
streams them to the dashboard.

    ./pal.sh                                       # starts this and the dashboard (see pal.sh)
    HEADLESS=0 tools/run_sim.sh pal_live.py        # the raw form

What it does: builds the conveyor cell (infeed table, three cardboard boxes, running belt, suction
tool) and then loops forever. With nothing queued it holds the arm at home; when the dashboard
sends a command it runs that cycle — style, box mass, optional fault — in real time. Everything it
does is served on http://127.0.0.1:$PAL_LIVE_PORT (default 8765, host networking):

  GET  /state?since=N   {"seq": last row number,
                         "rows": [[t, q*dof, qd*dof, tau_measured*dof, q_cmd*dof, tau_applied*dof,
                                   held_kg, run_id], ...] after row N,
                         "status": {...}}
  GET  /frame.png       the latest viewport picture
  POST /cmd             {"cmd": "run", "style": "s_curve", "kg": 1.5,
                         "fault": "none"|"friction"|"stuck_joint"|"delay", "joint": 3,
                         "loop": false, "speed": 0.7}
                        (speed: the dashboard speed bar as a fraction, 0.05-1.0; 1.0 is
                         cfg.SPEED_BAND of the velocity limit, i.e. half)
                        {"cmd": "stop"}   (abort the cycle and stay at home, cancels repeat)

Row width follows the arm: `dof` is read from the robot spec and published in the status as
`dof`, so the dashboard never assumes a joint count.

Rows are HZ/DECIM per second (250 Hz at the default 250 Hz physics: every step; 200 Hz at 1000 Hz:
every 5th step, with torque the mean of the 5). Sensor noise is off unless $PAL_SENSOR_NOISE asks
for it, the same default the recorded runs use. Time is pace-limited to real time.

Ported to the conveyor cell on 2026-11-08. It used to build a two-finger workcell and pick a
single cube, which is the arm this project no longer has.
"""
import json
import os
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from pal_app import HEADLESS, start_app
simulation_app = start_app("pal_live.py")

import numpy as np

import pal_config as cfg
import pal_suites as S
from pal_plant import build_plant
from pal_runner import KD_NORMAL, KP_NORMAL, Runner
from pal_sensors import from_env as sensor_from_env

PORT = int(os.environ.get("PAL_LIVE_PORT", "8765"))
HZ = float(os.environ.get("PAL_PHYSICS_HZ", cfg.LIVE_PHYSICS_HZ))
DOF = cfg.DOF
DECIM = max(1, round(HZ / 200))           # one row per DECIM steps: 250 Hz rows at 250 Hz physics, 200 Hz rows at 1000 Hz (the dashboard reads row_hz)
RENDER_HZ = float(os.environ.get("PAL_LIVE_RENDER_HZ", "20"))   # save more frames for the dashboard without making the render path the bottleneck (0 = never)
REALTIME = os.environ.get("PAL_LIVE_REALTIME", "1") != "0"
FRAME_EVERY_S = 0.10
KEEP_ROWS = 6000                       # 24 s of 250 Hz rows


class Abort(Exception):
    pass


class Hub:
    """Shared between the HTTP threads and the simulation (main) thread."""

    def __init__(self):
        self.lock = threading.Lock()
        self.rows = deque(maxlen=KEEP_ROWS)
        self.seq = 0
        self.cmds = deque()
        self.stop_flag = False
        self.status = {"state": "booting", "run": "", "style": "", "kg": 0.0, "fault": "none",
                       "joint": -1, "loop": False, "t": 0.0, "duration": 0.0, "box": None,
                       "boxes_placed": 0, "held": 0.0, "last": None, "windowed": True,
                       "frames": 0, "dof": DOF, "robot": cfg.ROBOT_NAME,
                       "speed_band": cfg.SPEED_BAND}
        self.frame_path = None

    def push(self, row):
        with self.lock:
            self.seq += 1
            self.rows.append((self.seq, row))

    def since(self, n, limit=600):
        with self.lock:
            got = [(i, r) for i, r in self.rows if i > n][:limit]
            return {"seq": self.seq, "last": got[-1][0] if got else n, "rows": [r for _, r in got], "status": dict(self.status)}

    def set(self, **kw):
        with self.lock:
            self.status.update(kw)


HUB = Hub()
HUB.set(hz=HZ, row_hz=HZ / DECIM)                 # the dashboard reads the row rate from here


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path, _, query = self.path.partition("?")
        if path == "/state":
            since = 0
            for kv in query.split("&"):
                if kv.startswith("since="):
                    since = int(kv[6:] or 0)
            self._send(200, json.dumps(HUB.since(since)).encode())
        elif path == "/frame.png":
            p = HUB.frame_path
            try:
                self._send(200, open(p, "rb").read(), "image/png") if p else self._send(404, b"no frame yet", "text/plain")
            except OSError:
                self._send(404, b"no frame yet", "text/plain")
        else:
            self._send(404, b"{}")

    def do_POST(self):
        try:
            msg = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
            if msg.get("cmd") == "run":
                # Build the scenario here, on the request thread, so a bad command is a 400 with a
                # reason rather than an exception inside the simulation loop.
                S.live_cycle(msg.get("style", "s_curve"), msg.get("kg"), msg.get("fault", "none"),
                             int(msg.get("joint", 3)), speed=float(msg.get("speed", 0.7)))
                with HUB.lock:
                    HUB.cmds.append(msg)
                    HUB.stop_flag = False
            elif msg.get("cmd") == "stop":
                with HUB.lock:
                    HUB.cmds.clear()
                    HUB.stop_flag = True
                    HUB.status["loop"] = False
            else:
                raise ValueError("unknown cmd")
            self._send(200, b'{"ok": true}')
        except Exception as exc:                                    # bad request: say why, keep serving
            self._send(400, json.dumps({"ok": False, "error": repr(exc)}).encode())


threading.Thread(target=lambda: ThreadingHTTPServer(("127.0.0.1", PORT), Handler).serve_forever(), daemon=True).start()
print(f"[PAL] live server on http://127.0.0.1:{PORT}  ({'headless' if HEADLESS else 'windowed'})", flush=True)

# The scene is drawn and the viewport captured even without a window: the dashboard shows the picture.
plant = build_plant(HZ, scene=True, cell=True)
render_every = max(1, round(HZ / RENDER_HZ)) if RENDER_HZ > 0 else 0
runner = Runner(plant, "/workspace/out/live", mj_every=10, render_every=render_every)
art, cell = plant.art, plant.cell
tool = cell.mount_tool(cfg.ROBOT_PRIM)
cell.frame_camera()
# Ideal sensing unless $PAL_SENSOR_NOISE asks otherwise; an ideal sensor is a cheap no-op, so the
# streaming path does not branch on whether noise is on.
noise = sensor_from_env(seed=0, dof=DOF)

if True:
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
    os.makedirs("/workspace/out/live", exist_ok=True)
frame_n, last_frame = 0, 0.0


def grab_frame():
    """Save the viewport to one of two files and serve the previous one, so a half-written picture is never sent."""
    global frame_n, last_frame
    if time.time() - last_frame < FRAME_EVERY_S:
        return
    last_frame = time.time()
    capture_viewport_to_file(get_active_viewport(), f"/workspace/out/live/frame_{frame_n % 2}.png")
    if frame_n > 0:
        HUB.frame_path = f"/workspace/out/live/frame_{(frame_n - 1) % 2}.png"
        HUB.set(frames=frame_n)
    frame_n += 1

state = {"run_id": 0, "t_base": 0.0, "tau_acc": np.zeros(DOF), "app_acc": np.zeros(DOF),
         "n_acc": 0, "wall0": 0.0, "sim0": 0.0, "held": 0.0,
         "rt_wall": time.time(), "rt_sim": 0.0}


def emit(t_local, qd, q_des, tau_applied):
    """Called after every physics step: accumulate torque, publish a row every DECIM steps, pace to real time, draw frames."""
    state["tau_acc"] += np.asarray(art.get_measured_joint_efforts()).flatten()[:DOF]
    state["app_acc"] += np.asarray(tau_applied).flatten()[:DOF]    # commanded: shows friction and faults
    state["n_acc"] += 1
    if state["n_acc"] < DECIM:
        return
    row = {"tau_measured": state["tau_acc"] / DECIM,
           "tau_applied": state["app_acc"] / DECIM,
           "q": np.asarray(art.get_joint_positions()).flatten()[:DOF].astype(float),
           "qd": np.asarray(qd).flatten()[:DOF].astype(float)}
    flat = {f"{k}_{i}": float(v[i]) for k, v in row.items() for i in range(DOF)}
    noise.apply(flat)
    get = lambda k: np.array([flat[f"{k}_{i}"] for i in range(DOF)])    # noqa: E731
    state["tau_acc"][:] = 0.0
    state["app_acc"][:] = 0.0
    state["n_acc"] = 0
    t = state["t_base"] + t_local
    HUB.push([round(t, 4)]
             + [round(float(x), 6) for x in np.concatenate(
                 [get("q"), get("qd"), get("tau_measured"),
                  np.asarray(q_des).flatten()[:DOF], get("tau_applied")])]
             + [state["held"], state["run_id"]])
    HUB.set(t=round(t_local, 3), held=round(float(state["held"]), 3),
            box=[round(float(x), 4) for x in cell.position()],
            boxes_placed=sum(1 for b in cell.boxes if b.on_belt()))
    wall = time.time()
    if wall - state["rt_wall"] >= 2.0:                       # how fast the simulation really runs, shown in status
        HUB.set(rt=round((t - state["rt_sim"]) / (wall - state["rt_wall"]), 2), hz=HZ)
        state["rt_wall"], state["rt_sim"] = wall, t
    if REALTIME:
        ahead = (t - state["sim0"]) - (time.time() - state["wall0"])
        if ahead > 0:
            time.sleep(min(ahead, 0.2))
    grab_frame()


def begin(run_id):
    state.update(run_id=run_id, wall0=time.time(), sim0=state["t_base"])


def hold_step(q_hold, k):
    """One physics step holding the arm still, used between cycles. The belt keeps running and the
    boxes stay where the last cycle left them, so the idle view is the cell at rest, not frozen."""
    tau, qs, qds, _ = runner._command(lambda t: (q_hold, np.zeros(DOF), np.zeros(DOF)),
                                      0.0, KP_NORMAL, KD_NORMAL, True)
    runner._apply(tau)
    plant.world.step(render=bool(render_every) and k % render_every == 0)
    cell.step(1.0 / HZ, visual=bool(render_every) and k % render_every == 0)
    emit(k / HZ, np.asarray(art.get_joint_velocities()).flatten()[:DOF], q_hold, tau)


def run_cycle(msg):
    style = msg.get("style", "s_curve")
    sc = S.live_cycle(style, msg.get("kg"), msg.get("fault", "none"), int(msg.get("joint", 3)),
                      name=f"live_{style}", speed=float(msg.get("speed", 0.7)))
    task = sc.task
    cell.reset_boxes(kg=sc.box_kg)
    tool.set_vacuum(False, 0.0)
    state["run_id"] += 1
    state["t_base"] += 0.5          # a visible gap; the dashboard restarts its window per run id
    begin(state["run_id"])
    f = sc.fault
    HUB.set(state="running", run=sc.name, style=style, kg=sc.box_kg, fault=f.type,
            joint=f.joint if f.type != "none" else -1, loop=bool(msg.get("loop")),
            duration=round(sc.duration_s, 2), t=0.0, boxes=task.n_boxes, boxes_placed=0,
            speed=round(float(msg.get("speed", 0.7)), 2),
            fault_onset=(round(f.onset_s, 2) if f.type != "none" else None))

    # When each box is expected to be on the tool, so the status can say what is held without
    # asking the tool every step. "picked" and "placed" are the task's own event names.
    held_windows = []
    for i in range(task.n_boxes):
        a = next((e[0] for e in task.events if e[1] == "picked" and e[2] == i), None)
        b = next((e[0] for e in task.events if e[1] == "placed" and e[2] == i), None)
        if a is not None and b is not None:
            held_windows.append((a, b))

    def on_step(k, t, st):
        state["held"] = sc.box_kg if any(a <= t < b for a, b in held_windows) else 0.0
        if HUB.stop_flag:
            raise Abort()
        cell.step(1.0 / HZ, visual=bool(render_every) and k % render_every == 0)
        emit(t, st["qd"], st["q_des"], st["tau"])

    aborted = False
    try:
        runner.run(sc, write=False, on_step=on_step)
    except Abort:
        aborted = True
        runner.inj.clear()
        tool.set_vacuum(False, 0.0)
    placed = sum(1 for b in cell.boxes if b.on_belt())
    last = {"run": sc.name, "kg": sc.box_kg, "fault": f.type, "aborted": aborted,
            "boxes": task.n_boxes, "boxes_placed": placed,
            "ok": (not aborted) and placed == task.n_boxes}
    state["t_base"] += sc.duration_s
    state["held"] = 0.0
    HUB.set(state="idle", last=last, boxes_placed=placed)
    print(f"[PAL] cycle done: {last}", flush=True)


# start-up: boxes on the infeed table, arm at home, vacuum off
cell.reset_boxes()
tool.set_vacuum(False, 0.0)
q_home = cfg.HOME_JOINTS.copy()
art.set_joint_positions(np.asarray(q_home, np.float32).reshape(1, -1))
art.set_joint_velocities(np.zeros((1, DOF), np.float32))
begin(0)
HUB.set(state="idle")
print(f"[PAL] live simulator ready: {cfg.ROBOT_NAME}, {cell.n_boxes if hasattr(cell, 'n_boxes') else len(cell.boxes)} boxes, "
      f"{HZ:.0f} Hz physics; waiting for commands from the dashboard", flush=True)

k = 0                                   # physics steps of the current idle stretch
while simulation_app.is_running():
    with HUB.lock:
        msg = HUB.cmds.popleft() if HUB.cmds else None
    if msg is not None:
        state["t_base"] += k / HZ               # keep the stream's clock moving forward across idle stretches
        k = 0
        run_cycle(msg)
        if HUB.status["loop"] and not HUB.stop_flag:
            with HUB.lock:
                HUB.cmds.append(dict(msg, loop=True))
        else:
            HUB.set(loop=False)
        begin(state["run_id"])
        continue
    hold_step(q_home, k)
    k += 1
simulation_app.close()
