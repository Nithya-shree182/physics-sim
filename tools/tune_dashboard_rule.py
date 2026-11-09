#!/usr/bin/env python3
"""Tune the dashboard's own alarm rule (dashboard/engine.py, Analyzer) on recorded live-simulator rows.

    .venv-pal/bin/python tools/tune_dashboard_rule.py baseline         # the rule as shipped, on tuning (2 kg) and test (5 kg)
    .venv-pal/bin/python tools/tune_dashboard_rule.py tune             # search settings on the 2 kg runs ONLY
    .venv-pal/bin/python tools/tune_dashboard_rule.py score <json>     # one settings file on both

The rows come from tools/record_live_runs.py: per box mass one healthy reference run, five more healthy runs, and every dashboard fault on
every joint. Each run is replayed through the SAME Analyzer the dashboard runs (reference recording, calibration, alerts), so the score is
the dashboard's behaviour, not a re-implementation.

Per fault run we ask four things: was anything raised, was it the right KIND, was it on the right JOINT, and how much ELSE was raised beside
it (an extra alert is a nuisance, a wrong one is a lie). Healthy runs count every alert as a false alarm.

Selection rule, fixed before looking: no alarm on any healthy tuning run; then most faults named correctly (kind and joint); then fewest
extra alerts; then shortest delay. 5 kg is scored once, after the choice.
"""
import itertools
import json
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "dashboard"), str(ROOT / "workspace"), str(ROOT / "observers"), str(ROOT / "model")]
DATA = ROOT / "out" / "livedet"
OUT = Path("/tmp/claude-1000/-home-thinkspace-Desktop-Physics-Based-RoboArm/0b19776f-b886-4997-ba4a-fab7b4fe516f/scratchpad")

DEFAULTS = dict(z=6.0, consec_s=0.15, abs_nm=0.10, abs_frac=0.005, settle_top=3, stuck_track=0.03, global_joints=None, sd_floor_frac=0.0, event_mask_s=0.0, lag_min=0.02, lag_sd=0.005, lag_mask=0.0)
KIND = {"friction": "friction", "stuck_joint": "Stuck joint", "delay": "Command delay"}


def load(tag):
    runs = []
    for p in sorted(DATA.glob(f"{tag}_*.npz")):
        z = np.load(p, allow_pickle=True)
        runs.append((p.stem, z["rows"], json.loads(str(z["status"]))))
    # the reference first: it must be recorded before anything is compared with it
    runs.sort(key=lambda r: (0 if "ref_healthy" in r[0] else 1, r[0]))
    return runs


def replay(runs, s):
    """Replay runs through one Analyzer with settings `s`; return one result dict per run."""
    import engine as E
    from pinocchio_model import load_robot
    E.LIVE_Z, E.CONSEC_S, E.ABS_MIN_NM, E.ABS_MIN_FRAC = s["z"], s["consec_s"], s["abs_nm"], s["abs_frac"]
    E.SETTLE_TOP, E.STUCK_MIN_TRACK_RAD = s["settle_top"], s["stuck_track"]
    E.SD_FLOOR_FRAC, E.EVENT_MASK_S = s.get("sd_floor_frac", 0.0), s.get("event_mask_s", 0.0)
    E.LAG_MIN_S, E.LAG_SD_FLOOR_S, E.LAG_MASK_S = s.get("lag_min", 0.02), s.get("lag_sd", 0.005), s.get("lag_mask", 0.0)
    if s.get("global_joints"):
        E.GLOBAL_JOINTS = s["global_joints"]
    twin = load_robot(E.ROBOT)
    an = E.Analyzer(twin, "backward", detector=True, picks=True)
    refs, out = {}, []
    an.refs = refs
    n = E.BLOCK_STEPS // E.DECIM
    for name, rows, st in runs:
        ids = rows[:, -1]
        rid = ids.max()
        r = rows[ids == rid]
        r = r[: int((float(st.get("duration", 1e9)) + 0.5) * float(st.get("row_hz") or 250.0))]       # the run itself, not the arm idling at home afterwards
        key = (st.get("style"), round(float(st.get("kg", 0.0)), 2), round(float(st.get("speed", 0.0)), 2))
        dt = 1.0 / float(st.get("row_hz") or 250.0)
        an.reset_window()
        an.recalibrate()
        t0 = float(r[0, 0]) - n * dt
        an.begin_run(key, t0, st.get("fault", "none"), st.get("fault_onset"), dict(st))
        ctl = E.Controls()
        ctl.fault = st.get("fault", "none")
        D = E.DOF
        t_run, first = 0.0, None
        onset = st.get("fault_onset")
        t_start = float(r[0, 0])
        for i in range(0, len(r) - n + 1, n):
            a = r[i:i + n]
            blk = E.Block(t=float(a[-1, 0]), q=a[:, 1:1 + D], qd=a[:, 1 + D:1 + 2 * D], tau=a[:, 1 + 2 * D:1 + 3 * D], q_cmd=a[:, 1 + 3 * D:1 + 4 * D],
                          truth={"load": float(a[-1, -2]), "fault": st.get("fault", "none"), "fault_joint": st.get("joint", -1)},
                          tau_applied=a[:, 1 + 4 * D:1 + 5 * D], dt=dt)
            t_run += n * dt
            an.feed(blk, ctl, t_run)
            if first is None and any(a_["sev"] > 0 and k != "payload" for k, a_ in an.alerts.items()):
                first = float(a[-1, 0]) - t_start
        alerts = list(an.run_stats["alerts"].values())          # (where, title) of everything raised during the run
        an.end_run()
        out.append(dict(name=name, fault=st.get("fault", "none"), joint=st.get("joint", -1), onset=onset, alerts=alerts, first=first,
                        recorded=an.recording if False else None))
    return out


def judge(res):
    """Per run: detected, right kind, right joint, number of extra alerts, delay after onset."""
    rows = []
    for r in res:
        f = r["fault"]
        al = r["alerts"]
        if f == "none":
            rows.append(dict(name=r["name"], fault=f, false_alarm=len(al) > 0, n=len(al)))
            continue
        kind = KIND[f]
        right_kind = [a for a in al if kind in a[1]]
        if f == "delay":
            ok_joint = bool(right_kind)
        else:
            ok_joint = any(a[0] == f"Joint {r['joint'] + 1}" for a in right_kind)
        # a stuck joint is also a friction-like disturbance: count the right joint under EITHER label as located, but only the
        # right label as the right kind
        located = ok_joint or any(a[0] == f"Joint {r['joint'] + 1}" for a in al)
        rows.append(dict(name=r["name"], fault=f, joint=r["joint"], detected=len(al) > 0, right_kind=bool(right_kind), located=located,
                         exact=ok_joint, extra=max(0, len(al) - (1 if ok_joint else 0)), delay=(r["first"] - (r["onset"] or 0)) if r["first"] is not None and r["onset"] else None,
                         alerts=[f"{a[0]}: {a[1]}" for a in al]))
    return rows


def summarise(rows):
    h = [r for r in rows if r["fault"] == "none"]
    f = [r for r in rows if r["fault"] != "none"]
    d = [r["delay"] for r in f if r.get("delay") is not None]
    return dict(false_alarms=sum(r["false_alarm"] for r in h), n_healthy=len(h), detected=sum(r["detected"] for r in f),
                exact=sum(r["exact"] for r in f), located=sum(r["located"] for r in f), n_faulty=len(f),
                extra=sum(r["extra"] for r in f), delay=float(np.mean(d)) if d else float("nan"))


def evaluate(args):
    s, tag = args
    return summarise(judge(replay(load(tag), s)))


def tune():
    """Greedy search: each round replays every single-setting change from the current best, all at once, and keeps the best one if it beats the
    current. Runs on the 2 kg runs ONLY."""
    grid = dict(z=(5.0, 8.0, 10.0, 14.0), consec_s=(0.08, 0.25, 0.40), abs_frac=(0.002, 0.01, 0.02), abs_nm=(0.3, 0.6),
                sd_floor_frac=(0.001, 0.002, 0.005, 0.01), event_mask_s=(0.2, 0.4, 0.6), lag_min=(0.03, 0.04, 0.06), lag_sd=(0.01, 0.02, 0.04),
                lag_mask=(0.6, 1.2), settle_top=(0, 6), stuck_track=(0.01, 0.06))

    def key(sm):
        return (-sm["false_alarms"], sm["exact"], -sm["extra"], -(sm["delay"] if sm["delay"] == sm["delay"] else 9))

    cur = dict(DEFAULTS)
    cur_sm = evaluate((cur, "tune"))
    print("start", cur, cur_sm, flush=True)
    for rnd in range(6):
        cands = [dict(cur, **{n: v}) for n, vs in grid.items() for v in vs if v != cur[n]]
        with Pool(min(len(cands), 22)) as pool:
            sms = pool.map(evaluate, [(c, "tune") for c in cands])
        best_c, best_sm = max(zip(cands, sms), key=lambda cs: key(cs[1]))
        diff = {k: v for k, v in best_c.items() if best_c[k] != cur[k]}
        print(f"[round {rnd}] best change {diff}: {best_sm}", flush=True)
        if key(best_sm) <= key(cur_sm):
            break
        cur, cur_sm = best_c, best_sm
    print("\nBEST", json.dumps(cur), "\n", cur_sm, flush=True)
    (OUT / "best_dash.json").write_text(json.dumps(cur))


def report(s):
    for tag, label in (("tune", "TUNING 2 kg"), ("test", "HELD-OUT 5 kg")):
        rows = judge(replay(load(tag), s))
        sm = summarise(rows)
        print(f"\n== {label}: detected {sm['detected']}/{sm['n_faulty']}, right kind and joint {sm['exact']}/{sm['n_faulty']}, joint located {sm['located']}, "
              f"extra alerts {sm['extra']}, healthy runs alarmed {sm['false_alarms']}/{sm['n_healthy']}, mean delay {sm['delay']:.2f}s")
        for r in rows:
            if r["fault"] == "none":
                if r["false_alarm"]:
                    print(f"   FALSE ALARM {r['name']}")
            else:
                tag_ = "ok " if r["exact"] and not r["extra"] else ("ok+" if r["exact"] else ("loc" if r["located"] else ("det" if r["detected"] else "MISS")))
                print(f"   {tag_} {r['name']:22s} {'; '.join(r['alerts'])}")


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "baseline":
        report(dict(DEFAULTS, z=6.0))
    elif cmd == "tune":
        tune()
    else:
        report(dict(DEFAULTS, **json.loads(Path(sys.argv[2]).read_text())))
