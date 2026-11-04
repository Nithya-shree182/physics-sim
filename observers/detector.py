"""Fault and tamper detection (O5), baseline: thresholds on two signals.

1. Residual channel: r = tau_measured - tau_model per joint. Catches anything that changes
   the dynamics: friction (stiction / bearing wear), a stuck joint, an unmodelled load or
   contact.
2. Tracking channel: e = q_cmd - q per joint. Catches commands that are executed late or
   wrongly. A faithfully executed but wrong command keeps the physics consistent, so the
   residual cannot see it (dev's A2, WORK_SPLIT.md); a command delay is the synthetic example.

Each channel is standardised per joint against a calibration window at the start of the run
(assumed fault-free): z = (x - mean) / max(std, floor). An alarm needs |z| > Z_THRESHOLD for
CONSECUTIVE_S seconds in a row (counted in rows from the run's own row spacing, so 1 kHz and 250 Hz logs behave alike), and is
suppressed where the pose fails the detector's pose mask (observers/pose_mask.py). The joint that alarms first is reported as the suspected joint.

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
from pose_mask import FORCE_GAIN_MAX, force_noise_gain  # noqa: E402
from pinocchio_model import load_panda  # noqa: E402
from payload import estimate_payload, payload_torque_per_kg                                  # noqa: E402
from residual import DEFAULT_CUTOFF_HZ, Residual, Run, compute_residual, fault_torque, load_run  # noqa: E402

FAULT_LABELS = {
    "none": "none",
    "friction": "friction (stiction / bearing wear)",
    "stuck_joint": "stuck joint",
    "delay": "command delay",
    "backlash": "backlash",
}

CALIBRATION_S = 3.0          # [CHOICE] fault-free window at the start of each run
Z_THRESHOLD = 8.0            # [CHOICE] standard deviations
CONSECUTIVE_S = 0.050         # [CHOICE] how long the z-score must stay over the threshold, in seconds (was 50 rows, which is 50 ms only at 1 kHz)
FLOOR_RESIDUAL_NM = 0.02     # [CHOICE] minimum std, N*m, so a quiet joint cannot alarm on noise
FLOOR_TRACKING_RAD = 2e-4    # [CHOICE] minimum std, rad


@dataclass
class Detection:
    t: np.ndarray
    z_residual: np.ndarray       # (n, 7)
    z_tracking: np.ndarray       # (n, 7)
    alarm: np.ndarray            # (n,) bool, after the consecutive-time rule and the pose mask
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


BASELINES = ("none", "gain", "friction")
FRICTION_WIDTH = 0.05        # rad/s, width of the tanh(qd / width) sign feature; a plain choice, not tuned


def recalibrate_baseline(res: Residual, mode: str = "friction", calibration_s: float = CALIBRATION_S) -> Residual:
    """Subtract, per joint, what the calibration window says the healthy residual looks like.

    The healthy residual is not white: a mass mismatch makes a part proportional to the model torque, and Coulomb friction that the
    model only approximates makes a sign(qd) part. Both are fitted by least squares on the calibration window only (before any
    fault, so a later fault cannot leak into the fit) and removed, which narrows the z-score's denominator.

      'none'      the residual as it is
      'gain'      r - (a * tau_model + b)
      'friction'  r - (a * tau_model + b + c * tanh(qd / FRICTION_WIDTH))

    On our 40 faulty runs 'friction' finds friction down to x3 (x2 is still missed) with no false alarm; 'gain' alone changes
    nothing (docs/DETECTION_RESULTS.md).

    CONDITION: the fit extrapolates, so it is valid only when the calibration window covers the motion the run goes on to make.
    That holds for our repeating reference (a period of about 2 s fits in the 3 s window) and for a repeated pick-and-place cycle.
    It does NOT hold for motion that does not repeat: on the analysis side's synthetic slow_reference (autocorrelation -0.06 at
    3 s) it false-alarms on joint 5 with a 3 s window and is clean with 6 s or more. Hence it is opt-in.
    """
    import dataclasses
    if mode not in BASELINES:
        raise ValueError(f"baseline must be one of {BASELINES}")
    if mode == "none":
        return res
    calib = ((res.t - res.t[0]) < calibration_s) & res.valid
    r = res.r.copy()
    for j in range(res.r.shape[1]):
        cols = [res.tau_model[:, j], np.ones(len(res.t))]
        if mode == "friction":
            cols.append(np.tanh(res.qd[:, j] / FRICTION_WIDTH))
        X = np.stack(cols, axis=1)
        coef, *_ = np.linalg.lstsq(X[calib], res.r[calib, j], rcond=None)
        r[:, j] = res.r[:, j] - X @ coef
    return dataclasses.replace(res, r=r)


def compensate_payload(run: Run, model, res: Residual, cutoff_hz: float | None = DEFAULT_CUTOFF_HZ, tau_s: float = 0.3) -> Residual:
    """Remove the torque of a payload the model does not know about, so picking something up is not read as a fault.

    The model has no payload, so a pickup steps the applied-torque residual up by the payload's torque and the detector raises an alarm
    (measured: a healthy 1 kg pick false-alarms at the grasp). The analysis side's own estimator measures the payload from the NET torque,
    where friction and the other faults do not appear, so it cannot absorb a fault. Its running estimate (a short time constant, so the
    pickup is followed within ~0.3 s) times the torque one kilogram produces is subtracted from the residual before the z-score."""
    est = estimate_payload(compute_residual(run, model, torque="measured", cutoff_hz=cutoff_hz), model, tau_s=tau_s)
    import dataclasses
    m = np.where(np.isfinite(est.m_est), est.m_est, 0.0)
    r = res.r.copy()
    for k in np.flatnonzero(res.valid):
        r[k] -= m[k] * payload_torque_per_kg(model, res.q[k], res.qd[k], res.qdd[k])
    return dataclasses.replace(res, r=r)


def _smooth(x: np.ndarray, w: int) -> np.ndarray:
    """Moving average along axis 0 that holds the edge values (a zero-padded 'same' convolution decays to zero at both ends, which turns a
    constant offset into a huge false difference in the last w/2 samples)."""
    if w <= 1:
        return x
    pad = np.pad(x, ((w // 2, w - 1 - w // 2), (0, 0)), mode="edge")
    kernel = np.ones(w) / w
    return np.stack([np.convolve(pad[:, j], kernel, mode="valid") for j in range(x.shape[1])], axis=1)


def _fill_edges(x: np.ndarray) -> np.ndarray:
    """Replace NaN rows (the filter's edge effect, and rows without an acceleration) by the nearest valid row. Filling with zero instead
    drags a smoothed reference toward zero near the ends, which is wrong whenever the residual is not zero there."""
    x = x.copy()
    for j in range(x.shape[1]):
        ok = np.isfinite(x[:, j])
        if ok.any() and not ok.all():
            idx = np.flatnonzero(ok)
            x[:, j] = np.interp(np.arange(len(x)), idx, x[idx, j])          # holds the end values outside the valid range
    return x


def against_reference(res: Residual, reference: Residual, smooth_s: float = 0.2) -> Residual:
    """Subtract the residual of a known-healthy run of the SAME task, aligned in time, before detection.

    For a task that does not repeat (a pick-and-place), a 3 s calibration window never sees the carry or the place, so everything after it
    looks abnormal (measured: a healthy 1 kg pick false-alarms). A healthy reference of the same task carries the pose-dependent part of
    the residual (mass mismatch, friction hysteresis, the payload's weight), so the difference isolates what changed. The reference is
    smoothed over `smooth_s` so its own noise is not injected into the run being tested. Both runs must have the same timeline."""
    import dataclasses
    n = min(len(res.r), len(reference.r))
    w = max(1, int(round(smooth_s / float(np.median(np.diff(res.t))))))
    ref = _fill_edges(reference.r[:n])
    ref = _smooth(ref, w)
    r = res.r.copy()
    r[:n] -= ref
    r[n:] = np.nan
    return dataclasses.replace(res, r=r)


def consecutive_rows(t: np.ndarray, consecutive_s: float = CONSECUTIVE_S, consecutive: int | None = None) -> int:
    """Rows that make `consecutive_s` seconds at this log's row spacing; an explicit row count wins."""
    if consecutive is not None:
        return max(1, int(consecutive))
    return max(1, int(round(consecutive_s / float(np.median(np.diff(t))))))


def detect(run: Run, res: Residual, calibration_s: float = CALIBRATION_S, z_threshold: float = Z_THRESHOLD,
           consecutive_s: float = CONSECUTIVE_S, consecutive: int | None = None, force_gain_max: float = FORCE_GAIN_MAX, baseline: str = "none",
           reference_run: Run | None = None) -> Detection:
    res = recalibrate_baseline(res, baseline, calibration_s)
    t = res.t
    calib = (t - t[0]) < calibration_s
    gated = np.array([force_noise_gain(J) > force_gain_max for J in res.J])

    z_r = _standardise(res.r, calib & res.valid, FLOOR_RESIDUAL_NM)
    track = run.arr("q_cmd") - run.arr("q")
    if reference_run is not None:
        # the tracking channel needs the same reference: a loaded carry tracks worse than the unloaded first seconds, and that is not a fault
        ref = reference_run.arr("q_cmd") - reference_run.arr("q")
        n = min(len(track), len(ref))
        w = max(1, int(round(0.2 / float(np.median(np.diff(t))))))
        ref = _smooth(ref[:n], w)
        track = track.copy(); track[:n] -= ref; track[n:] = np.nan
    z_e = _standardise(track, calib & np.all(np.isfinite(track), axis=1), FLOOR_TRACKING_RAD)

    over_r = np.nan_to_num(np.abs(z_r), nan=0.0) > z_threshold
    over_e = np.nan_to_num(np.abs(z_e), nan=0.0) > z_threshold
    over_r[gated] = False
    over_e[gated] = False
    over_r[calib] = False
    over_e[calib] = False
    rows = consecutive_rows(t, consecutive_s, consecutive)
    hit_r = _run_length(over_r) >= rows
    hit_e = _run_length(over_e) >= rows
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
    ap.add_argument("--torque", choices=("auto", "measured", "applied"), default="auto",
                    help="auto (default): the applied torque on effort-mode logs, where friction and locked joints show; "
                         "measured otherwise")
    ap.add_argument("--baseline", choices=BASELINES, default="none",
                    help="calibration-window baseline removed before the z-score (default none, the original rule; 'friction' finds "
                         "smaller friction faults but needs a calibration window that covers the motion, see recalibrate_baseline)")
    ap.add_argument("--payload-aware", action="store_true",
                    help="subtract the estimated payload's torque first, so a pickup mid-run is not read as a fault")
    ap.add_argument("--reference", type=Path, default=None,
                    help="a known-healthy run of the same task (same timeline): calibrate against it instead of the first seconds")
    ap.add_argument("--z", type=float, default=Z_THRESHOLD)
    ap.add_argument("--consecutive-s", type=float, default=CONSECUTIVE_S, help="seconds the z-score must stay over the threshold")
    ap.add_argument("--consecutive", type=int, default=None, help="the same in rows (overrides --consecutive-s)")
    args = ap.parse_args()

    model = load_panda()
    print(f"{'run':40s} {'fault':36s} {'detected':>8s} {'delay s':>8s} {'false':>6s} {'joint':>9s}  channel")
    for p in args.parquet:
        run = load_run(p)
        torque = fault_torque(run) if args.torque == "auto" else args.torque
        res = compute_residual(run, model, torque=torque, cutoff_hz=args.cutoff)
        if args.payload_aware:
            res = compensate_payload(run, model, res, args.cutoff)
        if args.reference:
            ref_run = load_run(args.reference)
            res = against_reference(res, compute_residual(ref_run, model, torque=torque, cutoff_hz=args.cutoff))
        det = detect(run, res, z_threshold=args.z, consecutive_s=args.consecutive_s, consecutive=args.consecutive, baseline=args.baseline,
                     reference_run=ref_run if args.reference else None)
        e = evaluate(run, det)
        delay = f"{e['delay_s']:.3f}" if e["delay_s"] is not None else "-"
        joint = f"{e['suspected_joint'] or '-'}/{e['true_joint'] or '-'}"
        print(f"{run.name:40s} {e['fault']:36s} {str(e['detected']):>8s} {delay:>8s} "
              f"{str(e['false_alarm']):>6s} {joint:>9s}  {e['channel'] or '-'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
