#!/usr/bin/env python3
"""Score the analysis side's real code on the pick-and-place runs (out/pick for the modelled payload, out/pickphys for a real cube):
payload estimate through the carry, and fault detection.

    .venv-pal/bin/python tools/run_pick_analysis.py [out/pick | out/pickphys] [physics rate in Hz, default 1000]

PAYLOAD (O4). The grasp is a mid-run event: before it the arm is empty, after it the known mass is at the fingertip centre. The tare is the
estimator's own reading on the empty stretch (it carries the plant's 5% mass mismatch, which looks like a phantom payload). The carried
estimate is the mean of the per-row estimates in the window from 1 s after the grasp to 0.3 s before the release, minus the tare, and is
compared with the known mass: within 3% or 0.03 kg, whichever is larger. Also reported: how long after the grasp the running estimate is
within that band.

FAULTS (O5). The detector (applied torque, their 20 Hz filter), with the fault starting just after the grasp, against the healthy twin that
shares its noise seed.
"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "observers"), str(ROOT / "model")]
from detector import against_reference, compensate_payload, detect, evaluate       # noqa: E402
from payload import estimate_payload                            # noqa: E402
from pinocchio_model import load_panda                          # noqa: E402
from residual import DEFAULT_CUTOFF_HZ, compute_residual, fault_torque, load_run  # noqa: E402


def known_mass(run, t):
    kg = run.df["payload_kg"].to_numpy(dtype=float)
    return float(kg[np.searchsorted(run.df["t"].to_numpy(), t)])


def main(out, hz=1000):
    suf = f"_{hz}hz"                      # score one physics rate at a time: runs of different rates are never paired
    out = Path(out)
    model = load_panda()
    print(f"{'PAYLOAD':<34}{'known kg':>9}{'tare kg':>9}{'carried est':>12}{'error kg':>10}{'error %':>9}{'settles in s':>14}")
    ok = True
    n = 0
    for p in sorted(out.glob(f"pick*_*{suf}.parquet")) + sorted(out.glob(f"ellipse_*{suf}.parquet")):
        run = load_run(p)
        if run.manifest.get("fault", {}).get("type", "none") != "none" or "fault" in p.name:
            continue
        task = run.manifest.get("task")
        res = compute_residual(run, model)                                   # measured torque: payload is an external load
        est = estimate_payload(res, model)
        t = est.t
        if task:
            ta = task["t_attach_s"]
            tr = task.get("t_open_s") or task["t_release_s"]       # a physical cube stops loading the arm when the fingers open
            known = known_mass(run, 0.5 * (ta + tr))
            empty = (t > 1.0) & (t < ta - 0.3)
            carry = (t > ta + 1.0) & (t < tr - 0.3)
        else:                                                                # ellipse: the payload is held throughout
            known = known_mass(run, t[len(t) // 2])
            empty = np.zeros_like(t, bool)
            carry = (t > 3.0) & (t < t[-1] - 3.0)
        if not task and known == 0.0:                                        # the empty ellipse IS the tare for the loaded one
            print(f"{run.name.replace(suf, ''):<34}{'0':>9}   (reference: the tare for the loaded ellipse)")
            continue
        raw = est.m_raw
        if empty.any():
            tare_use = float(np.nanmean(raw[empty]))
        elif not task and known > 0:                                         # ellipse: the 0 kg run on the same motion is the tare
            ref = load_run(out / f"ellipse_0kg{suf}.parquet")
            tare_use = float(np.nanmean(estimate_payload(compute_residual(ref, model), model).m_raw[carry]))
        else:
            tare_use = 0.0
        m = float(np.nanmean(raw[carry])) - tare_use
        err = m - known
        good = abs(err) <= max(0.03 * known, 0.03)
        settle = ""
        if task and known > 0:
            run_est = est.m_est - tare_use
            after = (t >= ta) & (np.abs(run_est - known) <= max(0.03 * known, 0.03))
            first = np.argmax(after) if after.any() else None
            settle = f"{t[first] - ta:.1f}" if first is not None else "never"
        ok &= good
        n += 1
        print(f"{run.name.replace(suf, ''):<34}{known:>9g}{tare_use:>9.3f}{m + tare_use:>12.3f}{err:>+10.3f}"
              f"{(100 * err / known) if known else 0:>8.1f}%{settle:>14}{'' if good else '   <-- outside tolerance'}")
    print(f"\n  {n} runs scored | PAYLOAD (picks and ellipse)", "PASS" if ok and n else "FAIL")

    print(f"\n{'FAULT (applied torque)':<38}{'calibration':>24}{'detected':>9}{'delay s':>9}{'false alarm':>13}{'joint':>7}  channel")
    healthy = out / f"pick_s_curve_1kg{suf}.parquet"
    healthy = healthy if healthy.exists() else out / f"pickphys_s_curve_1kg{suf}.parquet"
    ref_path = out / healthy.name.replace(f"_1kg{suf}", f"_1kg_ref{suf}")
    ref_run = load_run(ref_path) if ref_path.exists() else None
    ref_res = compute_residual(ref_run, model, torque="applied", cutoff_hz=DEFAULT_CUTOFF_HZ) if ref_run is not None else None
    for p in sorted(out.glob(f"pick*_fault_*{suf}.parquet")) + ([healthy] if healthy.exists() else []):
        run = load_run(p)
        base = compute_residual(run, model, torque=fault_torque(run), cutoff_hz=DEFAULT_CUTOFF_HZ)
        modes = [("first 3 s (the default)", base, None), ("payload-aware", compensate_payload(run, model, base), None)]
        if ref_res is not None:
            modes.append(("healthy reference run", against_reference(base, ref_res), ref_run))
        for label, res, rr in modes:
            e = evaluate(run, detect(run, res, reference_run=rr, consecutive=max(1, round(0.050 * hz))))     # 50 ms, whatever the row rate
            print(f"{run.name.replace(suf, ''):<38}{label:>24}{str(e['detected']):>9}"
                  f"{(format(e['delay_s'], '.2f') if e['delay_s'] is not None else '-'):>9}{str(e['false_alarm']):>13}"
                  f"{str(e['suspected_joint'] or '-'):>7}  {e['channel'] or '-'}")
    return 0 if ok and n else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "out/pick", int(sys.argv[2]) if len(sys.argv) > 2 else 1000))
