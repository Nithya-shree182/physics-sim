#!/usr/bin/env python3
"""The live simulator: a windowed (or headless) Isaac Sim that runs real picks on command and streams them to the dashboard.

    ./pal.sh                       # starts this and the dashboard (see pal.sh)
    HEADLESS=0 tools/run_sim.sh pal_live.py        # the raw form

What it does: builds the gripper plant (table, physical cube, fingers), then loops forever: when no task is queued it holds the arm at home;
when the dashboard sends a command it runs that pick (style, cube mass, optional fault) in real time. Everything it does is served on
http://127.0.0.1:$PAL_LIVE_PORT (default 8765, host networking):

  GET  /state?since=N   {"seq": last row number, "rows": [[t, q*7, qd*7, tau_measured*7, q_cmd*7, held_kg, run_id], ...] after row N, "status": {...}}
  GET  /frame.png       the latest viewport picture (windowed runs only)
  POST /cmd             {"cmd": "run", "style": "s_curve", "kg": 1.0, "fault": "none"|"friction"|"stuck_joint"|"delay", "joint": 3, "loop": false}
                        {"cmd": "stop"}  (abort the current pick and stay at home, cancels repeat)

Rows are 200 Hz (every 5th physics step; the torque is the mean of the 5), with the same sensor noise the data runs use. Time is pace-limited to real time.
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
import pal_tasks as K
from pal_plant import build_plant
from pal_runner import KD_NORMAL, KP_NORMAL, Runner
from pal_sensors import DEFAULT_SIGMA

PORT = int(os.environ.get("PAL_LIVE_PORT", "8765"))
HZ = float(os.environ.get("PAL_PHYSICS_HZ", cfg.PHYSICS_HZ))
DECIM = 5
REALTIME = os.environ.get("PAL_LIVE_REALTIME", "1") != "0"
FRAME_EVERY_S = 0.3
KEEP_ROWS = 6000                       # 30 s of 200 Hz rows


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
        self.status = {"state": "booting", "run": "", "style": "", "kg": 0.0, "fault": "none", "joint": -1, "loop": False,
                       "t": 0.0, "duration": 0.0, "cube": None, "lifted": False, "last": None, "windowed": True, "frames": 0}
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
                S.live_pick(msg.get("style", "s_curve"), float(msg.get("kg", 1.0)), msg.get("fault", "none"), int(msg.get("joint", 3)))   # validate now
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
plant = build_plant(HZ, scene=True, gripper=True)
runner = Runner(plant, "/workspace/out/live", mj_every=10, render_every=33)
art, cell = plant.art, plant.cell
rng = np.random.default_rng(0)
sig = DEFAULT_SIGMA
render_every = 33

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


state = {"run_id": 0, "t_base": 0.0, "tau_acc": np.zeros(7), "n_acc": 0, "wall0": 0.0, "sim0": 0.0, "held": 0.0}


def emit(t_local, qd, q_des):
    """Called after every physics step: accumulate torque, publish a row every DECIM steps, pace to real time, draw frames."""
    state["tau_acc"] += np.asarray(art.get_measured_joint_efforts()).flatten()[:7]
    state["n_acc"] += 1
    if state["n_acc"] < DECIM:
        return
    tau = state["tau_acc"] / DECIM + rng.normal(0, sig["tau"], 7)
    q = np.asarray(art.get_joint_positions()).flatten()[:7] + rng.normal(0, sig["q"], 7)
    qd = np.asarray(qd).flatten()[:7] + rng.normal(0, sig["qd"], 7)
    state["tau_acc"][:] = 0.0
    state["n_acc"] = 0
    t = state["t_base"] + t_local
    HUB.push([round(t, 4)] + [round(float(x), 6) for x in np.concatenate([q, qd, tau, np.asarray(q_des).flatten()[:7]])]
             + [state["held"], state["run_id"]])
    HUB.set(t=round(t_local, 3), cube=[round(float(x), 4) for x in cell.position()])
    if REALTIME:
        ahead = (t - state["sim0"]) - (time.time() - state["wall0"])
        if ahead > 0:
            time.sleep(min(ahead, 0.2))
    grab_frame()


def begin(run_id):
    state.update(run_id=run_id, wall0=time.time(), sim0=state["t_base"])


def hold_step(q_hold, k):
    """One physics step holding the arm still (used between picks; the cube stays where the last pick left it)."""
    tau, qs, qds, _ = runner._command(lambda t: (q_hold, np.zeros(7), np.zeros(7)), 0.0, KP_NORMAL, KD_NORMAL, True)
    runner._apply(tau)
    plant.world.step(render=bool(render_every) and k % render_every == 0)
    emit(k / HZ, np.asarray(art.get_joint_velocities()).flatten()[:7], q_hold)


def run_pick(msg):
    sc = S.live_pick(msg.get("style", "s_curve"), float(msg.get("kg", 1.0)), msg.get("fault", "none"), int(msg.get("joint", 3)),
                     name=f"live_{msg.get('style', 's_curve')}")
    task = sc.task
    state["run_id"] += 1
    state["t_base"] += 0.5                      # a visible gap between picks; the dashboard restarts its window at each new run id
    begin(state["run_id"])
    f = sc.fault
    HUB.set(state="running", run=sc.name, style=msg.get("style", "s_curve"), kg=sc.cube_kg, fault=f.type, joint=f.joint if f.type != "none" else -1,
            loop=bool(msg.get("loop")), duration=round(sc.duration_s, 2), t=0.0)

    def on_step(k, t, st):
        state["held"] = sc.cube_kg if task.t_close <= t < task.t_open else 0.0
        if HUB.stop_flag:
            raise Abort()
        emit(t, st["qd"], st["q_des"])

    aborted = False
    try:
        runner.run(sc, write=False, on_step=on_step)
    except Abort:
        aborted = True
        runner.inj.clear()
        plant.set_fingers(0.04)
    cube = np.array(cell.position())
    err = float(np.linalg.norm(cube[:2] - np.array(task.place_xy)))
    last = {"run": sc.name, "kg": sc.cube_kg, "fault": f.type, "aborted": aborted, "placed_error_m": round(err, 4),
            "cube_z": round(float(cube[2]), 4), "ok": (not aborted) and err < 0.03}
    state["t_base"] += sc.duration_s
    state["held"] = 0.0
    HUB.set(state="idle", last=last)
    print(f"[PAL] pick done: {last}", flush=True)


# start-up: cube on the table at the pick location, arm at home, fingers open
task0 = K.pick_place("s_curve", physical=True)
cell.place(task0.pick_xy, 1.0)
plant.set_fingers(0.04)
q_home = cfg.HOME_JOINTS.copy()
art.set_joint_positions(np.concatenate([q_home, [.04, .04]]))
art.set_joint_velocities(np.zeros((1, 9)))
begin(0)
HUB.set(state="idle")
print("[PAL] live simulator ready; waiting for commands from the dashboard", flush=True)

k = 0                                   # physics steps of the current idle stretch
while simulation_app.is_running():
    with HUB.lock:
        msg = HUB.cmds.popleft() if HUB.cmds else None
    if msg is not None:
        state["t_base"] += k / HZ               # keep the stream's clock moving forward across idle stretches
        k = 0
        run_pick(msg)
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
