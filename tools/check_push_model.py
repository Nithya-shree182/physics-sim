#!/usr/bin/env python3
"""Validation push (proposal 7.7) scored with the analysis side's independent model, which includes friction.

    .venv-pal/bin/python tools/check_push_model.py out/push/push_*.parquet      (not the push_nofric_* control)

tools/check_push.py scores the same runs with the simulator's own M, C, g, which contain no friction, so static friction biases
its estimate low (docs/VALIDATION.md). Here the force comes from observers/residual.py: r = tau_applied - tau_model with the
Pinocchio model (Gaz friction included, 20 Hz filter), then W_ext = -(J^T)^+ r at the gripper. Same window, same force columns,
same pre-registered rule as check_push.py: magnitude within 15%, off-axis within 1.5 N, speed under 50% of the limit.
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "observers"), str(ROOT / "model")]
from pinocchio_model import load_panda          # noqa: E402
from residual import DEFAULT_CUTOFF_HZ, compute_residual, load_run  # noqa: E402

REL_TOL, OFFAXIS_TOL_N, SPEED_LIMIT = 0.15, 1.5, 0.5
QD_MAX = np.array([2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61])


def main(paths):
    model = load_panda()
    print(f"  {'run':<34}{'applied N':>22}{'estimated N':>26}{'err %':>8}{'off-axis N':>12}")
    worst_rel = worst_off = worst_speed = 0.0
    n = 0
    for p in map(Path, paths):
        run = load_run(p)
        if str(run.df["traj_type"].iloc[0]) != "push" or "nofric" in p.name:
            continue
        hz = 1.0 / run.dt
        pf = run.df[[f"push_force_{a}" for a in "xyz"]].to_numpy()
        on = np.where(np.linalg.norm(pf, axis=1) > 0)[0]
        lo, hi = int(on[0] + 0.6 * hz), int(on[-1] - 0.05 * hz)
        res = compute_residual(run, model, torque="applied", cutoff_hz=DEFAULT_CUTOFF_HZ)
        est = np.nanmean(res.wrench[lo:hi, :3], axis=0)
        app = pf[lo:hi].mean(axis=0)
        mag = np.linalg.norm(app); axis = app / mag
        rel = abs(est @ axis - mag) / mag
        off = np.linalg.norm(est - (est @ axis) * axis)
        speed = float(np.max(np.abs(run.arr("qd")[lo:hi]) / QD_MAX))
        worst_rel, worst_off, worst_speed, n = max(worst_rel, rel), max(worst_off, off), max(worst_speed, speed), n + 1
        print(f"  {run.name:<34}{str(np.round(app, 1).tolist()):>22}{str(np.round(est, 2).tolist()):>26}"
              f"{100 * rel:>7.1f}%{off:>12.2f}")
    ok = n > 0 and worst_rel <= REL_TOL and worst_off <= OFFAXIS_TOL_N and worst_speed <= SPEED_LIMIT
    print(f"\n  {n} runs | worst magnitude error {100 * worst_rel:.1f}% (limit {100 * REL_TOL:.0f}%) | worst off-axis "
          f"{worst_off:.2f} N (limit {OFFAXIS_TOL_N} N) | peak speed {worst_speed:.0%} of limit")
    print("PUSH VALIDATION (independent model, friction included):", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
