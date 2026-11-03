"""Fault and tamper detection (O5), baseline: thresholds on two signals.

1. Residual channel: r = tau_measured - tau_model per joint. Catches anything that changes
   the dynamics: friction (stiction / bearing wear), a stuck joint, an unmodelled load or
   contact.
2. Tracking channel: e = q_cmd - q per joint. Catches commands that are executed late or
   wrongly. A faithfully executed but wrong command keeps the physics consistent, so the
   residual cannot see it (dev's A2, WORK_SPLIT.md); a command delay is the synthetic example.

Each channel is standardised per joint against a calibration window at the start of the run
(assumed fault-free): z = (x - mean) / max(std, floor). An alarm needs |z| > Z_THRESHOLD for
CONSECUTIVE steps in a row, and is suppressed where the pose fails the detector gate
(observers/confidence.py). The joint that alarms first is reported as the suspected joint.

Reporting rule (dev's decision A1): the friction fault class is always named
"friction (stiction / bearing wear)", never one of the two alone.

This is the threshold baseline the proposal asks for before any classifier. A payload picked
up mid-run would also raise the residual channel; telling the two apart is the classifier's job.

    python observers/detector.py data/synthetic/*.parquet
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent / "model")]
from confidence import FORCE_GAIN_MAX, conditioning  # noqa: E402
from pinocchio_model import load_panda  # noqa: E402
from residual import DEFAULT_CUTOFF_HZ, Residual, Run, compute_residual, load_run  # noqa: E402

FAULT_LABELS = {
    "none": "none",
    "friction": "friction (stiction / bearing wear)",
    "stuck_joint": "stuck joint",
    "delay": "command delay",
    "backlash": "backlash",
}

CALIBRATION_S = 3.0          # [CHOICE] fault-free window at the start of each run
Z_THRESHOLD = 8.0            # [CHOICE] standard deviations
CONSECUTIVE = 50             # [CHOICE] steps in a row (50 ms at 1 kHz)
FLOOR_RESIDUAL_NM = 0.02     # [CHOICE] minimum std, N*m, so a quiet joint cannot alarm on noise
FLOOR_TRACKING_RAD = 2e-4    # [CHOICE] minimum std, rad


@dataclass
class Detection:
    t: np.ndarray
    z_residual: np.ndarray       # (n, 7)
    z_tracking: np.ndarray       # (n, 7)
    alarm: np.ndarray            # (n,) bool, after the consecutive-steps rule and the gate
    gated: np.ndarray            # (n,) bool, pose failed the detector gate
    first_alarm_t: float | None
    first_alarm_channel: str | None
    first_alarm_joint: int | None    # 0-6

    def summary(self) -> str:
        if self.first_alarm_t is None:
            return "no alarm"
        return (f"alarm at t = {self.first_alarm_t:.3f} s on the {self.first_alarm_channel} channel, "
                f"joint {self.first_alarm_joint + 1}")


def _standardise(x: np.ndarray, calib: np.ndarray, floor: float) -> np.ndarray:
    mu = np.nanmean(x[calib], axis=0)
    sd = np.maximum(np.nanstd(x[calib], axis=0), floor)
    return (x - mu) / sd


def _run_length(over: np.ndarray) -> np.ndarray:
    """Number of consecutive True values ending at each row, per column."""
    out = np.zeros(over.shape, dtype=int)
    count = np.zeros(over.shape[1], dtype=int)
    for k in range(len(over)):
        count = np.where(over[k], count + 1, 0)
        out[k] = count
    return out


def detect(run: Run, res: Residual, calibration_s: float = CALIBRATION_S, z_threshold: float = Z_THRESHOLD,
           consecutive: int = CONSECUTIVE, force_gain_max: float = FORCE_GAIN_MAX) -> Detection:
    t = res.t
    calib = (t - t[0]) < calibration_s
    gated = np.array([conditioning(J)["force_noise_gain"] > force_gain_max for J in res.J])

    z_r = _standardise(res.r, calib & res.valid, FLOOR_RESIDUAL_NM)
    track = run.arr("q_cmd") - run.arr("q")
    z_e = _standardise(track, calib & np.all(np.isfinite(track), axis=1), FLOOR_TRACKING_RAD)

    over_r = np.nan_to_num(np.abs(z_r), nan=0.0) > z_threshold
    over_e = np.nan_to_num(np.abs(z_e), nan=0.0) > z_threshold
    over_r[gated] = False
    over_e[gated] = False
    over_r[calib] = False
    over_e[calib] = False
    hit_r = _run_length(over_r) >= consecutive
    hit_e = _run_length(over_e) >= consecutive
    alarm = hit_r.any(axis=1) | hit_e.any(axis=1)

    first_t = first_ch = first_j = None
    if alarm.any():
        k = int(np.argmax(alarm))
        first_t = float(t[k])
        if hit_r[k].any():
            first_ch, first_j = "residual", int(np.argmax(np.abs(z_r[k]) * hit_r[k]))
        else:
            first_ch, first_j = "tracking", int(np.argmax(np.abs(z_e[k]) * hit_e[k]))
    return Detection(t, z_r, z_e, alarm, gated, first_t, first_ch, first_j)


def evaluate(run: Run, det: Detection) -> dict:
    """Compare with the run's ground-truth labels."""
    df = run.df
    fault = str(df["fault_type"].iloc[0])
    active = df["fault_active"].to_numpy()
    onset = float(df["t"][active].iloc[0]) if active.any() else None
    false_alarm = bool(det.alarm[~active].any())
    detected = bool(det.alarm[active].any()) if active.any() else None
    delay = None
    if detected:
        delay = float(det.t[np.argmax(det.alarm & active)]) - onset
    true_joint = int(df["fault_joint"].iloc[0])
    return {"fault": FAULT_LABELS.get(fault, fault), "onset_s": onset, "detected": detected,
            "delay_s": delay, "false_alarm": false_alarm, "true_joint": true_joint + 1 if true_joint >= 0 else None,
            "suspected_joint": det.first_alarm_joint + 1 if det.first_alarm_joint is not None else None,
            "channel": det.first_alarm_channel}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("parquet", type=Path, nargs="+")
    ap.add_argument("--cutoff", type=float, default=DEFAULT_CUTOFF_HZ)
    ap.add_argument("--z", type=float, default=Z_THRESHOLD)
    ap.add_argument("--consecutive", type=int, default=CONSECUTIVE)
    args = ap.parse_args()

    model = load_panda()
    print(f"{'run':40s} {'fault':36s} {'detected':>8s} {'delay s':>8s} {'false':>6s} {'joint':>9s}  channel")
    for p in args.parquet:
        run = load_run(p)
        det = detect(run, compute_residual(run, model, cutoff_hz=args.cutoff), z_threshold=args.z,
                     consecutive=args.consecutive)
        e = evaluate(run, det)
        delay = f"{e['delay_s']:.3f}" if e["delay_s"] is not None else "-"
        joint = f"{e['suspected_joint'] or '-'}/{e['true_joint'] or '-'}"
        print(f"{run.name:40s} {e['fault']:36s} {str(e['detected']):>8s} {delay:>8s} "
              f"{str(e['false_alarm']):>6s} {joint:>9s}  {e['channel'] or '-'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
