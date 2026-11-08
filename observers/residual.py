"""Residual: the torque the model did not account for.

    r = tau_measured - tau_model(q, qd, qdd)
    tau_model = M(q) qdd + C(q, qd) qd + g(q) + tau_f(qd)      (Pinocchio, model/pinocchio_model.py)

and its equivalent external wrench at the hand,

    W_ext = -(J^T)^+ r

W_ext is the force and moment acting ON the robot at the panda_hand frame (world-aligned,
[fx, fy, fz, mx, my, mz]). Holding a 1 kg payload gives fz = -9.81 N: gravity pulls down. The
minus sign is because the joints must supply -J^T W to resist a wrench W.

qdd is not logged; it comes from differencing qd, matched to the row's torque by the timing
convention in the run manifest: forward difference for synthetic logs ("tau at row k moves
qd[k] to qd[k+1]"), backward difference for dev's Isaac Sim logs (row assembled after the
step, so tau[k] produced q[k]). Optional
zero-phase low-pass filtering (Butterworth, filtfilt) is applied to qd and tau together so the
two stay time-aligned. filtfilt is offline only; a live dashboard needs a causal filter.

Guard: tau_applied is identically zero in position mode (dev's finding F2), so asking for it
on a position-mode log raises instead of returning -tau_model.

    python observers/residual.py data/synthetic/<run>.parquet [--cutoff 20] [--torque measured]

Library use: load_run() or run_from_frame(), then compute_residual(run, cutoff_hz=DEFAULT_CUTOFF_HZ).
"""

from __future__ import annotations

import argparse
import json
import sys
import warnings
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "model"))
from pinocchio_model import PandaModel, load_panda, load_robot  # noqa: E402

EFFORT_MODES = {"torque"}
TIMING_FORWARD = "tau at row k moves qd[k] to qd[k+1]"

# [CHOICE] 20 Hz zero-phase low-pass. On synthetic excitation logs (qd noise 2e-3 rad/s) it takes
# the no-payload residual from 2-6% of the torque limits (unfiltered differentiation noise) to
# under 0.5%, while the trajectories themselves stay below 1 Hz. Re-check on real logs.
DEFAULT_CUTOFF_HZ = 20.0


def fault_torque(run) -> str:
    """Which torque the FAULT residual should use for this log.

    tau_measured is the NET joint torque, which obeys Newton's law whatever produced it, so friction, backlash and a
    locked joint cannot appear in it (measured: ~0.001 N*m with friction x8; docs/REVIEW.md F2). The torque that was
    APPLIED does show them, and exists only in effort mode (it is zero in position mode). So: applied on effort-mode
    logs, measured otherwise. Payload and wrench estimation keep tau_measured: an external load is in the net torque,
    and friction (which is not) cannot contaminate it.
    """
    return "applied" if run.drive_mode in EFFORT_MODES else "measured"


# ── loading ─────────────────────────────────────────────────────────────────────────────
@dataclass
class Run:
    df: pd.DataFrame
    manifest: dict
    name: str

    def arr(self, prefix: str, n: int | None = None) -> np.ndarray:
        """Columns `prefix_0..prefix_(n-1)`. With no n, take however many the log actually has,
        so a six-joint UR10e run and a seven-joint Panda run both read correctly."""
        if n is None:
            n = sum(1 for c in self.df.columns if c.startswith(f"{prefix}_")
                    and c[len(prefix) + 1:].isdigit())
            if n == 0:
                raise KeyError(f"no {prefix}_* columns in run {self.name!r}")
        return self.df[[f"{prefix}_{i}" for i in range(n)]].to_numpy()

    @property
    def dt(self) -> float:
        return float(self.df["dt"].iloc[0])

    @property
    def drive_mode(self) -> str:
        return str(self.manifest.get("drive_mode", self.df["drive_mode"].iloc[0]))


def load_run(parquet: str | Path, manifest: str | Path | None = None) -> Run:
    parquet = Path(parquet)
    mpath = Path(manifest) if manifest else parquet.with_suffix(".json")
    man = json.loads(mpath.read_text(encoding="utf-8")) if mpath.is_file() else {}
    return Run(pd.read_parquet(parquet), man, parquet.stem)


def run_from_frame(df: pd.DataFrame, manifest: dict) -> Run:
    """Wrap an in-memory frame, e.g. straight from model/synth_log.generate()."""
    return Run(df, manifest, manifest.get("run_id", "in-memory"))


# ── pieces ──────────────────────────────────────────────────────────────────────────────
def lowpass(x: np.ndarray, dt: float, cutoff_hz: float | None, order: int = 2) -> np.ndarray:
    """Zero-phase Butterworth low-pass along axis 0. None means no filtering."""
    if not cutoff_hz:
        return x
    b, a = butter(order, cutoff_hz / (0.5 / dt))
    return filtfilt(b, a, x, axis=0)


def joint_acceleration(qd: np.ndarray, dt: float, timing: str = "forward") -> np.ndarray:
    """qdd per row, matched to the torque logged on that row.
    'forward'  (qd[k+1]-qd[k])/dt, last row NaN: tau at row k moves qd[k] to qd[k+1] (synthetic logs)
    'backward' (qd[k]-qd[k-1])/dt, first row NaN: tau at row k produced row k (Isaac Sim logs)
    'central'  (qd[k+1]-qd[k-1])/2dt, both ends NaN: fallback when the convention is unknown"""
    qdd = np.full_like(qd, np.nan)
    if timing == "forward":
        qdd[:-1] = np.diff(qd, axis=0) / dt
    elif timing == "backward":
        qdd[1:] = np.diff(qd, axis=0) / dt
    elif timing == "central":
        qdd[1:-1] = (qd[2:] - qd[:-2]) / (2 * dt)
    else:
        raise ValueError("timing must be 'forward', 'backward' or 'central'")
    return qdd


def external_wrench(J: np.ndarray, r: np.ndarray) -> np.ndarray:
    """W_ext = -(J^T)^+ r for one 6x7 Jacobian and one 7-vector residual."""
    return -np.linalg.pinv(J.T) @ r


def timing_from_manifest(manifest: dict) -> str:
    """Synthetic logs declare `timing`; dev's Isaac Sim writer declares `timing_convention`
    ("row assembled after world.step() ... tau_measured[k] is the effort during the step that
    produced q[k]"), which pairs row k's torque with (qd[k] - qd[k-1]): a backward difference."""
    if manifest.get("timing") == TIMING_FORWARD:
        return "forward"
    conv = str(manifest.get("timing_convention", ""))
    if "after world.step()" in conv or "produced q[k]" in conv:
        return "backward"
    warnings.warn("run manifest states no known timing convention; using a central difference")
    return "central"


# ── main computation ────────────────────────────────────────────────────────────────────
@dataclass
class Residual:
    t: np.ndarray            # (n,)
    r: np.ndarray            # (n, 7)  tau_measured - tau_model; NaN where qdd is undefined
                             #         and, when filtering, in the first/last 3 filter time constants
    tau_model: np.ndarray    # (n, 7)
    tau: np.ndarray          # (n, 7)  the (filtered) measured or applied torque used
    qdd: np.ndarray          # (n, 7)
    wrench: np.ndarray | None  # (n, 6) W_ext at panda_hand, world-aligned
    J: np.ndarray            # (n, 6, 7) the Jacobian used
    q: np.ndarray            # (n, 7) (filtered) joint positions used
    qd: np.ndarray           # (n, 7)
    settings: dict

    @property
    def valid(self) -> np.ndarray:
        return np.all(np.isfinite(self.r), axis=1)


def compute_residual(run: Run, model: PandaModel | None = None, torque: str = "measured",
                     cutoff_hz: float | None = None, timing: str | None = None,
                     jacobian: str = "model", wrench: bool = True) -> Residual:
    """r = tau - tau_model for every row of a run.

    torque   'measured' (default) or 'applied'. 'applied' is refused on non-effort logs.
    jacobian 'model' (Pinocchio, independent; default) or 'logged' (the simulator's J columns).
    """
    if torque not in ("measured", "applied"):
        raise ValueError("torque must be 'measured' or 'applied'")
    if torque == "applied" and run.drive_mode not in EFFORT_MODES:
        raise ValueError(f"tau_applied is zero in position mode; refusing a '{run.drive_mode}' log. "
                         "Use torque='measured'.")
    model = model or load_robot()
    dt = run.dt
    timing = timing or timing_from_manifest(run.manifest)

    q = lowpass(run.arr("q"), dt, cutoff_hz)
    qd = lowpass(run.arr("qd"), dt, cutoff_hz)
    tau = lowpass(run.arr(f"tau_{torque}"), dt, cutoff_hz)
    qdd = joint_acceleration(qd, dt, timing)
    n, dof = len(q), q.shape[1]
    tau_model = np.full((n, dof), np.nan)
    J = np.zeros((n, 6, dof))
    J_logged = (run.df[[f"J_{a}_{b}" for a in range(6) for b in range(dof)]].to_numpy().reshape(n, 6, dof)
                if jacobian == "logged" else None)
    # Friction belongs in the model only when the torque it is compared with contains friction. The APPLIED (commanded) torque
    # does: it has to overcome friction. The simulator's tau_measured is the NET joint torque and does not (measured,
    # docs/REVIEW.md F2). Logs say which in the manifest; the default (True) is what the synthetic logs need, where
    # tau_measured = tau_applied + noise.
    include_friction = True if torque == "applied" else bool(run.manifest.get("tau_measured_includes_friction", True))
    for k in range(n):
        if np.all(np.isfinite(qdd[k])):
            tau_model[k] = model.tau_model(q[k], qd[k], qdd[k], include_friction=include_friction)
        J[k] = J_logged[k] if J_logged is not None else model.jacobian(q[k])
    r = tau - tau_model
    if cutoff_hz:
        # filtfilt distorts the first and last few time constants of a record (the first
        # payload estimate came out at 0.74 of the true mass). Drop 3 time constants each end.
        edge = int(np.ceil(3.0 / (cutoff_hz * dt)))
        r[:edge] = np.nan
        r[-edge:] = np.nan

    W = None
    if wrench:
        W = np.full((n, 6), np.nan)
        ok = np.all(np.isfinite(r), axis=1)
        for k in np.flatnonzero(ok):
            W[k] = external_wrench(J[k], r[k])

    settings = {"torque": torque, "cutoff_hz": cutoff_hz, "timing": timing, "jacobian": jacobian,
                "model_friction": include_friction,
                "ee_frame": model.ee_frame, "drive_mode": run.drive_mode}
    return Residual(run.df["t"].to_numpy(), r, tau_model, tau, qdd, W, J, q, qd, settings)


def summarize(res: Residual, effort_limit: np.ndarray) -> dict:
    ok = res.valid
    r = res.r[ok]
    rms = np.sqrt(np.mean(r**2, axis=0))
    out = {"rows_used": int(ok.sum()), "rms_nm": rms.tolist(), "max_abs_nm": np.abs(r).max(axis=0).tolist(),
           "rms_pct_limit": (100 * rms / effort_limit).tolist()}
    if res.wrench is not None:
        last = ok & (res.t >= res.t[ok].max() - 1.0)
        out["wrench_mean_last_1s"] = np.nanmean(res.wrench[last], axis=0).tolist()
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("parquet", type=Path)
    ap.add_argument("--torque", choices=("measured", "applied"), default="measured")
    ap.add_argument("--cutoff", type=float, default=DEFAULT_CUTOFF_HZ,
                    help=f"low-pass cutoff in Hz, 0 for none (default {DEFAULT_CUTOFF_HZ:g})")
    ap.add_argument("--timing", choices=("forward", "backward", "central"), default=None,
                    help="qdd differencing (default: from the manifest)")
    ap.add_argument("--jacobian", choices=("model", "logged"), default="model")
    ap.add_argument("--out", type=Path, default=None, help="write the residual and wrench as parquet")
    args = ap.parse_args()

    run = load_run(args.parquet)
    model = load_panda()
    res = compute_residual(run, model, args.torque, args.cutoff, args.timing, args.jacobian)
    s = summarize(res, model.effort_limit)
    np.set_printoptions(precision=4, suppress=True)
    print(f"{run.name}: {s['rows_used']} rows, {res.settings}")
    print("  RMS residual (N*m)   :", np.array(s["rms_nm"]))
    print("  RMS, % of limit      :", np.array(s["rms_pct_limit"]))
    print("  max |residual| (N*m) :", np.array(s["max_abs_nm"]))
    if "wrench_mean_last_1s" in s:
        print("  W_ext, last 1 s mean :", np.array(s["wrench_mean_last_1s"]), " [fx fy fz mx my mz]")
    if args.out:
        cols = {"t": res.t} | {f"r_{i}": res.r[:, i] for i in range(res.r.shape[1])}
        if res.wrench is not None:
            cols |= {f"w_{a}": res.wrench[:, i] for i, a in enumerate(["fx", "fy", "fz", "mx", "my", "mz"])}
        args.out.parent.mkdir(parents=True, exist_ok=True)
        pd.DataFrame(cols).to_parquet(args.out, index=False)
        print(f"  written {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
