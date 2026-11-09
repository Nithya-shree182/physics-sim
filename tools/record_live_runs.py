#!/usr/bin/env python3
"""Record raw live-simulator rows for tuning the dashboard's alarm rule offline.

    PAL_SENSOR_NOISE=1 HEADLESS=1 tools/run_sim.sh pal_live.py       # the live server, noise on
    .venv-pal/bin/python tools/record_live_runs.py                    # then this: ~25 min

Per box mass (2 kg = tuning, 5 kg = held-out test): one healthy run (which the dashboard records as that task's reference), five more
healthy runs, then every dashboard fault on every joint (friction x8 and a stuck joint on each of the six joints, and the command delay).
Each run is saved as out/livedet/<name>.npz with the full row stream and the status the server reported, so tools/tune_dashboard_rule.py
can replay it through the dashboard's own Analyzer with different thresholds, at no cost.
"""
import json
import sys
import time
import urllib.request
from pathlib import Path

import numpy as np

U = "http://127.0.0.1:8765"
OUT = Path(__file__).resolve().parents[1] / "out" / "livedet"
OUT.mkdir(parents=True, exist_ok=True)
DOF = 6


def get(since):
    return json.load(urllib.request.urlopen(f"{U}/state?since={since}", timeout=5))


def post(msg):
    return json.loads(urllib.request.urlopen(urllib.request.Request(U + "/", json.dumps(msg).encode()), timeout=5).read())


def record(name, kg, fault="none", joint=3, speed=0.6, style="s_curve"):
    path = OUT / f"{name}.npz"
    if path.exists():
        print("have", name, flush=True)
        return
    seq = get(0)["last"]
    r = post(dict(cmd="run", style=style, kg=kg, fault=fault, joint=joint, speed=speed))
    assert r.get("ok"), r
    rows, status, t0 = [], None, time.time()
    started = False
    while time.time() - t0 < 240:
        d = get(seq)
        rows += d["rows"]
        seq = d["last"]
        st = d["status"]
        if st.get("state") == "running" and not started:
            started, status = True, dict(st)
        if started and st.get("state") == "idle":
            break
        time.sleep(0.8)
    d = get(seq)
    rows += d["rows"]
    a = np.array(rows, float)
    np.savez_compressed(path, rows=a, status=json.dumps(status), last=json.dumps(d["status"].get("last")))
    print(f"saved {name}: {len(a)} rows, placed {d['status'].get('last', {}).get('boxes_placed')}", flush=True)


if __name__ == "__main__":
    for tag, kg in (("tune", 2.0), ("test", 5.0)):
        record(f"{tag}_ref_healthy", kg)
        for i in range(5):
            record(f"{tag}_healthy_{i}", kg)
        for j in range(DOF):
            record(f"{tag}_friction_j{j}", kg, "friction", j)
            record(f"{tag}_stuck_j{j}", kg, "stuck_joint", j)
        record(f"{tag}_delay", kg, "delay", 0)
    print("RECORDING DONE", flush=True)
