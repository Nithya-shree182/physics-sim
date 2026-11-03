#!/usr/bin/env python3
"""Experiment: does a per-joint recalibration of the residual, fitted on the calibration window only, widen what the detector finds?

    .venv-pal/bin/python tools/try_recalibration.py out/o5/*.parquet out/extra/c4_*.parquet out/clean/*.parquet

The healthy residual is not white noise: with a 5% arm-mass mismatch it contains a part proportional to the model torque, and
Coulomb friction that the model only approximates contributes a sign(qd) term. Variants, all fitted by least squares on the
detector's own 3 s calibration window (before any fault) and then subtracted before the detector's usual z-score:

  none    the analysis side's residual as it is
  gain    r - (a_j * tau_model_j + b_j)
  gainfr  r - (a_j * tau_model_j + b_j + c_j * tanh(qd_j / 0.05))

No parameter is tuned on the runs. Reports detections by fault type and false alarms, for the applied torque.
"""
import dataclasses
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "observers"), str(ROOT / "model")]
from detector import CALIBRATION_S, detect, evaluate         # noqa: E402
from pinocchio_model import load_panda                         # noqa: E402
from residual import DEFAULT_CUTOFF_HZ, compute_residual, load_run  # noqa: E402


def recalibrate(res, mode):
    if mode == "none":
        return res
    t0 = res.t[0]
    calib = ((res.t - t0) < CALIBRATION_S) & res.valid
    r = res.r.copy()
    for j in range(7):
        cols = [res.tau_model[:, j], np.ones(len(res.t))]
        if mode == "gainfr":
            cols.append(np.tanh(res.qd[:, j] / 0.05))
        X = np.stack(cols, axis=1)
        coef, *_ = np.linalg.lstsq(X[calib], res.r[calib, j], rcond=None)
        r[:, j] = res.r[:, j] - X @ coef
    return dataclasses.replace(res, r=r)


def kind(e, name):
    if e["detected"] is None:
        return "healthy"
    f = e["fault"].split(" (")[0]
    if f.startswith("friction"):
        stem = name.replace("_1000hz", "")
        return "friction x3 (C4)" if stem.startswith("c4_") else "friction x" + stem.rsplit("_", 1)[-1]
    return f


def main(paths):
    model = load_panda()
    runs = [load_run(p) for p in paths]
    out = {}
    for mode in ("none", "gain", "gainfr"):
        tally = defaultdict(lambda: [0, 0])
        false = 0
        for run in runs:
            res = recalibrate(compute_residual(run, model, torque="applied", cutoff_hz=DEFAULT_CUTOFF_HZ), mode)
            e = evaluate(run, detect(run, res))
            k = kind(e, run.name)
            if e["detected"] is None:
                false += int(e["false_alarm"]); tally["healthy runs"][1] += 1
            else:
                tally[k][1] += 1; tally[k][0] += int(bool(e["detected"])); false += int(e["false_alarm"])
        out[mode] = (tally, false)
    kinds = sorted({k for t, _ in out.values() for k in t if k != "healthy runs"})
    print(f"{'fault':<26}{'none':>10}{'gain':>10}{'gainfr':>10}")
    for k in kinds:
        print(f"{k:<26}" + "".join(f"{out[m][0][k][0]}/{out[m][0][k][1]}".rjust(10) for m in ("none", "gain", "gainfr")))
    tot = {m: sum(v[0] for k, v in out[m][0].items() if k != "healthy runs") for m in out}
    den = sum(v[1] for k, v in out["none"][0].items() if k != "healthy runs")
    print(f"{'ALL FAULTY':<26}" + "".join(f"{tot[m]}/{den}".rjust(10) for m in ("none", "gain", "gainfr")))
    print(f"{'false alarms (any run)':<26}" + "".join(str(out[m][1]).rjust(10) for m in ("none", "gain", "gainfr")))


if __name__ == "__main__":
    main(sys.argv[1:])
