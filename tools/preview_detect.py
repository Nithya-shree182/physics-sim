#!/usr/bin/env python3
"""Preview of the O5 result with an independent numpy twin of the analysis side's model and a
re-implementation of their detector rule. NOT their code and NOT Pinocchio: a stand-in so we can see,
before hand-over, whether the injected faults rise above the mass mismatch and the sensor noise.

    .venv-tools/bin/python tools/preview_detect.py out/o5/*.parquet [--torque applied|measured]

Model (pal_kinematics.py, numpy, from the URDF with NO mismatch and NO payload, armature 0.0061):
    tau_model = M(q) qdd + c(q, qd) + g(q),  c from the Christoffel symbols of M(q) (finite differences)
Residual: r = tau_<torque> - tau_model, on a 50 Hz subsample, qd smoothed by a 20 ms zero-phase boxcar.
Detector rule, as in observers/detector.py on the nithya branch: z = (x - mean) / max(std, floor) against the
first 3 s of the run; alarm after |z| > 8 for 50 ms in a row; two channels (residual, tracking q_cmd - q);
floors 0.02 N*m and 2e-4 rad. No pose-confidence gate here (their gate never fired on well-conditioned poses).

Reports per run: detected, detection delay after onset, false alarm before onset, which joint alarmed first and
on which channel. Healthy runs give the false-alarm picture.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "workspace"))
from pal_kinematics import Urdf  # noqa: E402

ARM = 7
STEP = 20                       # subsample: 1 kHz -> 50 Hz
Z, HOLD_S, CALIB_S = 8.0, 0.05, 3.0
FLOOR_R, FLOOR_E = 0.02, 2e-4
ARMATURE = 0.0061


def coriolis(urdf, q, qd, M0, eps=1e-6):
    dM = np.zeros((ARM, ARM, ARM))                         # dM[i] = dM / dq_i
    for i in range(ARM):
        e = np.zeros(ARM); e[i] = eps
        dM[i] = (urdf.mass_matrix(q + e, 0.04, ARMATURE) - M0) / eps
    c = np.zeros(ARM)
    for k in range(ARM):
        for i in range(ARM):
            for j in range(ARM):
                c[k] += (dM[i][k, j] - 0.5 * dM[k][i, j]) * qd[i] * qd[j]
    return c


def model_torque(urdf, q, qd, qdd):
    M0 = urdf.mass_matrix(q, 0.04, ARMATURE)
    return M0 @ qdd + coriolis(urdf, q, qd, M0) + urdf.gravity(q, 0.04)


def standardise(x, calib, floor):
    return (x - x[calib].mean(axis=0)) / np.maximum(x[calib].std(axis=0), floor)


def run_one(path, torque, urdf):
    d = pd.read_parquet(path)
    arr = lambda p: np.stack([d[f"{p}_{i}"].to_numpy() for i in range(ARM)], axis=1)
    dt = float(d["dt"].iloc[0]); n = len(d)
    t = d["t"].to_numpy()
    q, qd, qc = arr("q"), arr("qd"), arr("q_cmd")
    tau = arr("tau_applied" if torque == "applied" else "tau_measured")
    w = 20
    box = lambda x: np.stack([np.convolve(x[:, j], np.ones(w) / w, mode="same") for j in range(ARM)], axis=1)
    qd_s, tau_s = box(qd), box(tau)
    qdd = np.vstack([np.zeros((1, ARM)), np.diff(qd_s, axis=0) / dt])
    idx = np.arange(w, n - w, STEP)
    r = np.array([tau_s[k] - model_torque(urdf, q[k], qd_s[k], qdd[k]) for k in idx])
    e = (qc - q)[idx]
    ts = t[idx]
    active = d["fault_active"].to_numpy(dtype=bool)[idx]
    calib = (ts - ts[0]) < CALIB_S
    zr, ze = standardise(r, calib, FLOOR_R), standardise(e, calib, FLOOR_E)
    need = max(1, int(round(HOLD_S / (STEP * dt))))
    out = {}
    for name, z in (("residual", zr), ("tracking", ze)):
        over = np.abs(z) > Z; over[calib] = False
        run = np.zeros_like(over, dtype=int); cnt = np.zeros(ARM, int)
        for k in range(len(over)):
            cnt = np.where(over[k], cnt + 1, 0); run[k] = cnt
        hit = run >= need
        alarm = hit.any(axis=1)
        out[name] = (alarm, hit, z)
    alarm = out["residual"][0] | out["tracking"][0]
    onset = float(ts[active][0]) if active.any() else None
    false_alarm = bool(alarm[~active].any())
    detected = bool(alarm[active].any()) if active.any() else None
    delay = float(ts[np.argmax(alarm & active)] - onset) if detected else None
    first = None
    if detected:
        k = int(np.argmax(alarm & active))
        for name in ("residual", "tracking"):
            if out[name][0][k]:
                j = int(np.argmax(np.abs(out[name][2][k]) * out[name][1][k])); first = (name, j + 1); break
    ft = str(d["fault_type"].iloc[0]); fj = int(d["fault_joint"].iloc[0])
    return {"run": Path(path).stem.replace("_1000hz", ""), "fault": ft, "joint": fj + 1 if fj >= 0 else None,
            "detected": detected, "delay": delay, "false_alarm": false_alarm, "first": first,
            "res_pre_rms": float(np.sqrt((r[~active & ~calib] ** 2).mean())) if (~active & ~calib).any() else float("nan")}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("parquet", nargs="+", type=Path)
    ap.add_argument("--torque", default="applied", choices=("applied", "measured"))
    a = ap.parse_args()
    urdf = Urdf(ROOT / "assets/franka_panda/panda_arm_hand.urdf")          # nominal: no mismatch, no payload
    rows = [run_one(p, a.torque, urdf) for p in a.parquet]
    print(f"torque signal: tau_{a.torque}\n")
    print(f"{'run':<34}{'fault':<13}{'joint':>6}{'detected':>10}{'delay s':>9}{'false alarm':>13}  first alarm")
    for r in rows:
        print(f"{r['run']:<34}{r['fault']:<13}{str(r['joint'] or '-'):>6}{str(r['detected']) if r['detected'] is not None else '-':>10}"
              f"{(format(r['delay'], '.2f') if r['delay'] is not None else '-'):>9}{str(r['false_alarm']):>13}  "
              f"{(r['first'][0] + ' j' + str(r['first'][1])) if r['first'] else '-'}")
    faulty = [r for r in rows if r["fault"] != "none"]; healthy = [r for r in rows if r["fault"] == "none"]
    print(f"\nfaulty runs detected: {sum(bool(r['detected']) for r in faulty)}/{len(faulty)} | "
          f"false alarms on healthy runs: {sum(r['false_alarm'] for r in healthy)}/{len(healthy)} | "
          f"before-onset false alarms on faulty runs: {sum(r['false_alarm'] for r in faulty)}/{len(faulty)}")


if __name__ == "__main__":
    main()
