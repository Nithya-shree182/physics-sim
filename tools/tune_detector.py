#!/usr/bin/env python3
"""Tune the fault detector on the UR10e's own runs, then score it once on data it has never seen.

    .venv-pal/bin/python tools/tune_detector.py tune            # grid over dtune_* only, print the best settings
    .venv-pal/bin/python tools/tune_detector.py score <json>    # one settings file, on dtune, dtest and dfast, per fault

The runs come from `tools/pal_suite.sh detect,detectfast` (workspace/pal_suites.py):

    dtune_*   fault seed 1000 plus eight healthy seeds   <- the ONLY data a threshold may be fitted to
    dtest_*   other seeds (2000, 2010-2017)              <- scored after the choice, never tuned on
    dfast_*   40% of the velocity limit, seed 3000       <- does the choice survive faster motion

Selection rule (decided before looking at the numbers): among settings with no alarm on any healthy tuning run AND still none
when the threshold is raised by SAFETY (so the choice is not sitting on the edge of the nine healthy runs), take the most detections;
ties go to the shorter mean delay. A setting that finds more by false-alarming is not an improvement.
"""
import argparse
import itertools
import json
import pickle
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "observers"), str(ROOT / "model"), str(ROOT / "workspace")]
import pal_config as cfg                                              # noqa: E402
from detector import consecutive_rows, detect, evaluate               # noqa: E402
from pinocchio_model import load_robot                                # noqa: E402
from residual import DEFAULT_CUTOFF_HZ, compute_residual, fault_torque, load_run  # noqa: E402

SAFETY = 1.5
CACHE = Path("/tmp/claude-1000/-home-thinkspace-Desktop-Physics-Based-RoboArm/0b19776f-b886-4997-ba4a-fab7b4fe516f/scratchpad/detect_cache.pkl")
DEFAULTS = dict(baseline="none", z=8.0, consecutive_s=0.050,   # the OLD Panda-era rule: the baseline the tuning is compared against
                floor_frac=0.0, floor_nm=0.02, floor_track=2e-4, cal_s=3.0, cutoff=DEFAULT_CUTOFF_HZ)


def load_runs(prefix_list, cutoff):
    """(run, residual) for every parquet whose name starts with a prefix. The residual is computed once per cutoff."""
    model = load_robot()
    out = {}
    for pre in prefix_list:
        for d in ("out/detect", "out/detectfast"):
            for p in sorted((ROOT / d).glob(f"{pre}_*.parquet")):
                run = load_run(p)
                res = compute_residual(run, model, torque=fault_torque(run), cutoff_hz=cutoff)
                out[p.stem.replace("_1000hz", "")] = (run, res)
    return out


def floors(s):
    """Per-joint residual floor: the larger of an absolute floor and a fraction of the joint's torque limit."""
    return np.maximum(s["floor_nm"], s["floor_frac"] * np.asarray(cfg.TAU_MAX))


def run_one(run, res, s, z=None):
    det = detect(run, res, calibration_s=s["cal_s"], z_threshold=s["z"] if z is None else z, consecutive_s=s["consecutive_s"],
                 baseline=s["baseline"], floor_residual=floors(s), floor_tracking=s["floor_track"])
    return evaluate(run, det)


def score(runs, s):
    """Per run: detected, delay, false alarm, joint right. Healthy runs have fault 'none'."""
    rows = []
    for name, (run, res) in runs.items():
        e = run_one(run, res, s)
        e["name"] = name
        e["fault_type"] = str(run.df["fault_type"].iloc[0])
        rows.append(e)
    return rows


def summarise(rows):
    healthy = [r for r in rows if r["fault_type"] == "none"]
    faulty = [r for r in rows if r["fault_type"] != "none"]
    det = [r for r in faulty if r["detected"]]
    return dict(false_alarms=sum(bool(r["false_alarm"]) for r in healthy), n_healthy=len(healthy), detected=len(det), n_faulty=len(faulty),
                delay=float(np.mean([r["delay_s"] for r in det])) if det else float("nan"),
                joint_right=sum(1 for r in det if r["suspected_joint"] == r["true_joint"] or r["fault_type"] == "delay"))


def tune(cutoffs=(10.0, 20.0)):
    best, table = None, []
    for cutoff in cutoffs:
        runs = load_runs(["dtune"], cutoff)
        healthy = {k: v for k, v in runs.items() if "healthy" in k}
        grid = itertools.product(("none", "gain", "friction"), (4.0, 5.0, 6.0, 8.0), (0.02, 0.05, 0.10), (0.0, 0.0005, 0.001, 0.002), (3.0,))
        for baseline, z, cons, ff, cal in grid:
            s = dict(DEFAULTS, baseline=baseline, z=z, consecutive_s=cons, floor_frac=ff, cal_s=cal, cutoff=cutoff)
            # healthy first: most settings die here, and it is the cheap half
            if any(run_one(r, res, s)["false_alarm"] or run_one(r, res, s, z=z * SAFETY)["false_alarm"] for r, res in healthy.values()):
                continue
            sm = summarise(score(runs, s))
            table.append((sm["detected"], -sm["delay"] if sm["detected"] else 0, s, sm))
            print(f"  ok  cutoff={cutoff:g} {baseline:8s} z={z:g} cons={cons:g}s floor_frac={ff:g}  -> {sm['detected']}/{sm['n_faulty']} found, "
                  f"delay {sm['delay']:.2f}s, joint right {sm['joint_right']}", flush=True)
    table.sort(key=lambda x: (x[0], x[1]), reverse=True)
    if not table:
        print("no setting is clean on the healthy tuning runs")
        return
    top = table[0]
    print("\nBEST:", json.dumps(top[2]), "\n", top[3])
    Path("/tmp/claude-1000/-home-thinkspace-Desktop-Physics-Based-RoboArm/0b19776f-b886-4997-ba4a-fab7b4fe516f/scratchpad/best.json").write_text(json.dumps(top[2]))


def report(s):
    for pre, label in (("dtune", "TUNING (fitted on this)"), ("dtest", "HELD-OUT TEST"), ("dfast", "40% SPEED")):
        runs = load_runs([pre], s["cutoff"])
        rows = score(runs, s)
        sm = summarise(rows)
        print(f"\n== {label}: {sm['detected']}/{sm['n_faulty']} faults found, {sm['false_alarms']}/{sm['n_healthy']} healthy runs alarmed, "
              f"mean delay {sm['delay']:.2f}s, joint named correctly {sm['joint_right']}/{sm['detected']}")
        by = {}
        for r in rows:
            if r["fault_type"] == "none":
                continue
            key = r["name"].split("_", 1)[1]
            kind = r["fault_type"]
            by.setdefault(kind, []).append((key, r))
        for kind, items in by.items():
            hit = [k for k, r in items if r["detected"]]
            miss = [k for k, r in items if not r["detected"]]
            print(f"  {kind:12s} {len(hit)}/{len(items)}   missed: {', '.join(m.replace(kind + '_', '') for m in miss) or '-'}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=("tune", "score"))
    ap.add_argument("settings", nargs="?", type=Path)
    a = ap.parse_args()
    if a.cmd == "tune":
        tune()
    else:
        report(dict(DEFAULTS, **json.loads(a.settings.read_text())) if a.settings else DEFAULTS)
