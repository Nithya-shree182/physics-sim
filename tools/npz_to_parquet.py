#!/usr/bin/env python3
"""Convert a captured .npz telemetry run to Parquet.

The Isaac Sim container ships neither pandas nor pyarrow, so pal_telemetry.py writes
compressed .npz with numpy alone. This runs on the host (.venv-tools) and produces the
Parquet file the analysis side's schema calls for. The JSON manifest is written by the
capture itself and is left alone.

    .venv-tools/bin/python tools/npz_to_parquet.py data/<run_id>.npz
    .venv-tools/bin/python tools/npz_to_parquet.py data/*.npz
"""
import sys
from pathlib import Path

import numpy as np
import pandas as pd

INT_COLS = {"step"}
INT8_COLS = {"fault_joint"}
BOOL_COLS = {"payload_attached", "fault_active", "attack_active", "mj_fresh"}
STR_COLS = {"run_id", "traj_type", "drive_mode", "fault_type", "attack_type"}


def convert(npz_path: Path) -> Path:
    z = np.load(npz_path, allow_pickle=True)
    cols = [str(c) for c in z["__columns__"]]
    out = {}
    for c in cols:
        v = z[c]
        if c in STR_COLS:
            out[c] = v.astype(str)
        elif c in BOOL_COLS:
            out[c] = v.astype(bool)
        elif c in INT8_COLS:
            out[c] = v.astype(np.int8)
        elif c in INT_COLS:
            out[c] = v.astype(np.int64)
        else:
            out[c] = v.astype(np.float64)
    df = pd.DataFrame(out, columns=cols)
    pq = npz_path.with_suffix(".parquet")
    df.to_parquet(pq, index=False)
    print(f"  {npz_path.name} -> {pq.name}   {len(df)} rows x {len(cols)} cols "
          f"({pq.stat().st_size/1e6:.2f} MB)")
    return pq


if __name__ == "__main__":
    args = sys.argv[1:]
    if not args:
        print(__doc__); sys.exit(2)
    for a in args:
        convert(Path(a))
