#!/usr/bin/env python3
"""Compare two telemetry runs column by column: the regression test for "did this change alter the physics?".

    .venv-tools/bin/python tools/compare_runs.py A.parquet B.parquet [--tol 1e-9]
    .venv-tools/bin/python tools/compare_runs.py dirA dirB            # every run with the same name in both directories

Prints the largest absolute difference over every float column (and which column); exit 0 only if it is within the tolerance. Used to
show that a refactor leaves old data bit-identical, and that a windowed run reproduces the headless one.
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd


def compare(a, b):
    da, db = pd.read_parquet(a), pd.read_parquet(b)
    if len(da) != len(db):
        return float("inf"), f"row counts differ ({len(da)} vs {len(db)})"
    worst, where = 0.0, ""
    for c in da.columns:
        if da[c].dtype.kind == "f" and c in db.columns:
            x, y = da[c].to_numpy(), db[c].to_numpy()
            ok = np.isfinite(x) & np.isfinite(y)
            if (np.isfinite(x) != np.isfinite(y)).any():
                return float("inf"), f"NaN pattern differs in {c}"
            d = float(np.max(np.abs(x[ok] - y[ok]))) if ok.any() else 0.0
            if d > worst:
                worst, where = d, c
    return worst, where


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("a", type=Path); ap.add_argument("b", type=Path); ap.add_argument("--tol", type=float, default=1e-9)
    args = ap.parse_args()
    pairs = [(args.a, args.b)] if args.a.is_file() else [(p, args.b / p.name) for p in sorted(args.a.glob("*.parquet"))
                                                         if (args.b / p.name).exists()]
    bad = 0
    for pa, pb in pairs:
        w, col = compare(pa, pb)
        flag = "ok " if w <= args.tol else "DIFF"
        bad += w > args.tol
        print(f"  {flag} {pa.name:<48} max |diff| {w:.3g}  ({col})")
    print(f"\n{len(pairs)} run(s) compared, {bad} outside tolerance {args.tol:g}")
    sys.exit(1 if bad or not pairs else 0)


if __name__ == "__main__":
    main()
