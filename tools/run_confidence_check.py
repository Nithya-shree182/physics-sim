#!/usr/bin/env python3
"""O3 on simulator data: does the pose-confidence gate close at the near-singular poses, and does the payload estimate pause there?

    .venv-pal/bin/python tools/run_confidence_check.py [out/o3]

For each probe pose (out/o3 holds the arm still there at 0 and 1 kg) the analysis side's own conditioning is computed from the model
Jacobian at the logged pose: the confidence c, the force-noise gain (the detector's gate, limit 10) and the vertical-noise gain (the payload
gate, limit 10). Then the payload estimate is read with the gate on (what the proposal specifies: paused where confidence is low) and with
the gate off, so the gate's effect is visible. The proposal's O3 pass condition: estimates visibly suppressed, not reported as valid, near
singular poses.

The pass through the extended singular pose (o3_pass_singular_*) is scored the same way over time: the fraction of rows the gate closes.
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "observers"), str(ROOT / "model")]
from confidence import FORCE_GAIN_MAX, VERTICAL_GAIN_MAX, conditioning, pose_confidence   # noqa: E402
from payload import estimate_payload                                                        # noqa: E402
from pinocchio_model import load_panda                                                      # noqa: E402
from residual import compute_residual, load_run                                             # noqa: E402

POSES = ["home", "horizontal_reach", "folded", "near_singular_wrist", "near_singular_extended"]


def main(out):
    out = Path(out)
    model = load_panda()
    print(f"{'pose':<24}{'c':>7}{'force gain':>12}{'vert gain':>11}{'detector gate':>15}{'payload gate':>14}"
          f"{'1 kg est, gate on':>19}{'gate off':>10}")
    results = {}
    for pose in POSES:
        runs = {kg: load_run(out / f"o3_hold_{pose}_{kg}_1000hz.parquet") for kg in ("0kg", "1kg")}
        q = np.array([runs["1kg"].df[f"q_{i}"].iloc[-1] for i in range(7)])
        J = model.jacobian(q)
        cnd = conditioning(J)
        pc = pose_confidence(model, q)
        det_ok = cnd["force_noise_gain"] <= FORCE_GAIN_MAX
        pay_ok = cnd["vertical_noise_gain"] <= VERTICAL_GAIN_MAX
        est = {}
        for kg, run in runs.items():
            res = compute_residual(run, model)
            e = estimate_payload(res, model)
            valid = ~e.gated
            est[kg] = (float(np.nanmean(e.m_raw[valid])) if valid.any() else float("nan"), float(np.nanmean(e.m_raw)), float(np.mean(e.gated)))
        gated_est = f"{est['1kg'][0] - est['0kg'][0]:.3f}" if pay_ok else "paused"
        ungated = est["1kg"][1] - est["0kg"][1]
        results[pose] = (pay_ok, det_ok, ungated)
        print(f"{pose:<24}{pc.get('c', cnd.get('c', float('nan'))):>7.3f}{cnd['force_noise_gain']:>12.1f}{cnd['vertical_noise_gain']:>11.1f}"
              f"{'open' if det_ok else 'CLOSED':>15}{'open' if pay_ok else 'CLOSED':>14}{gated_est:>19}{ungated:>10.3f}")
    print("\nPass through the extended singular pose, 1 kg carried (fraction of rows the gate closes):")
    for kg in ("0kg", "1kg"):
        p = out / f"o3_pass_singular_{kg}_1000hz.parquet"
        if not p.exists():
            continue
        run = load_run(p)
        res = compute_residual(run, model)
        e = estimate_payload(res, model)
        det_closed = np.mean([conditioning(J)["force_noise_gain"] > FORCE_GAIN_MAX for J in res.J[::20]])
        print(f"  {kg}: payload gate closed on {np.mean(e.gated):.0%} of rows, detector gate on {det_closed:.0%}")
    closed = [p for p, (pay_ok, det_ok, _) in results.items() if not pay_ok or not det_ok]
    open_ = [p for p in POSES if p not in closed]
    ok = "near_singular_extended" in closed and "home" in open_
    print(f"\n  gates closed at: {closed or 'none'}; open at: {open_}")
    print("O3 (gate closes at the singular pose, open at home):", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "out/o3"))
