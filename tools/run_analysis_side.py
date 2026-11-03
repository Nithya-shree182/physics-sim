#!/usr/bin/env python3
"""Run the analysis side's real code (nithya branch: Pinocchio model, residual, detector) on our runs.

    .venv-pal/bin/python tools/run_analysis_side.py --nithya <checkout of origin/nithya> \
        [--torque applied|measured] out/o5/*.parquet

Their modules are imported unmodified from a checkout (e.g. `git worktree add --detach <dir> origin/nithya`);
nothing here edits their branch. `.venv-pal` is a Python 3.12 environment with `pin` (Pinocchio), numpy, scipy,
pandas, pyarrow, pyyaml (see README). Their detector's own command line defaults to the measured torque; this
driver passes the torque explicitly so the two signals can be compared on the same runs.

Prints, per run: fault, detected, delay after onset, false alarm before onset, first alarming joint and channel,
and then totals. The pose-confidence gate is on (their default).
"""
import argparse
import sys
from pathlib import Path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--nithya", type=Path, required=True)
    ap.add_argument("--torque", default="applied", choices=("applied", "measured"))
    ap.add_argument("--z", type=float, default=None)
    ap.add_argument("--no-gate", action="store_true")
    ap.add_argument("--qdd", default="estimated", choices=("estimated", "commanded"),
                    help="acceleration used in the model torque: their differenced qd (default) or our logged "
                         "analytic qdd_cmd (noise-free; close to the true value while tracking is good)")
    ap.add_argument("parquet", nargs="+", type=Path)
    a = ap.parse_args()
    sys.path[:0] = [str(a.nithya / "observers"), str(a.nithya / "model")]
    from detector import FORCE_GAIN_MAX, Z_THRESHOLD, detect, evaluate        # noqa: E402
    from pinocchio_model import load_panda                                     # noqa: E402
    from residual import compute_residual, load_run                            # noqa: E402

    model = load_panda()
    kw = {"z_threshold": a.z or Z_THRESHOLD}
    if a.no_gate:
        kw["force_gain_max"] = 1e18
    print(f"torque signal: tau_{a.torque}   qdd: {a.qdd}   gate: {'off' if a.no_gate else 'on'}   z: {kw['z_threshold']}\n")
    print(f"{'run':<34}{'fault':<36}{'detected':>9}{'delay s':>9}{'false':>7}  first alarm")
    rows = []
    for p in a.parquet:
        run = load_run(p)
        res = compute_residual(run, model, torque=a.torque)
        if a.qdd == "commanded":
            import dataclasses
            import numpy as np
            qdd_c = run.arr("qdd_cmd")
            tm = np.full_like(res.tau_model, np.nan)
            for k in np.flatnonzero(res.valid):
                tm[k] = model.tau_model(res.q[k], res.qd[k], qdd_c[k])
            r = res.tau - tm
            r[~res.valid] = np.nan
            res = dataclasses.replace(res, r=r, tau_model=tm, qdd=qdd_c)
        det = detect(run, res, **kw)
        e = evaluate(run, det)
        rows.append(e)
        first = f"{e['channel']} j{e['suspected_joint']}" if e["channel"] else "-"
        print(f"{run.name:<34}{e['fault']:<36}{str(e['detected']) if e['detected'] is not None else '-':>9}"
              f"{(format(e['delay_s'], '.2f') if e['delay_s'] is not None else '-'):>9}{str(e['false_alarm']):>7}  {first}")
    faulty = [r for r in rows if r["detected"] is not None]
    healthy = [r for r in rows if r["detected"] is None]
    print(f"\nfaulty runs detected: {sum(bool(r['detected']) for r in faulty)}/{len(faulty)} | "
          f"false alarms on healthy runs: {sum(r['false_alarm'] for r in healthy)}/{len(healthy)} | "
          f"before-onset false alarms on faulty runs: {sum(r['false_alarm'] for r in faulty)}/{len(faulty)}")


if __name__ == "__main__":
    main()
