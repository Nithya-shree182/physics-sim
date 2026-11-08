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

# Which columns are not float64. Everything else is, including the energy and conditioning
# channels. Named explicitly rather than inferred from the array dtype, because npz round-trips
# a bool array as bool but a column that happened to hold only 0.0 and 1.0 would otherwise come
# back as float and fail the schema's dtype check. A few label arrays are stored as object arrays
# of Python strings (for example `fault_type` and `attack_type` when they are the sentinel
# literal "none"), so make the string detection resilient instead of assuming a fixed column set.
INT_COLS = {"step"}
INT8_COLS = {"fault_joint", "box_index"}
BOOL_COLS = {"payload_attached", "fault_active", "grip_active", "mj_fresh", "attack_active"}
STR_COLS = {"run_id", "traj_type", "drive_mode", "fault_type", "phase", "attack_type"}


def _is_text_like(v: np.ndarray) -> bool:
    if v.dtype.kind in ("U", "S"):
        return True
    if v.dtype.kind != "O":
        return False
    sample = np.asarray(v).ravel()
    if sample.size == 0:
        return False
    sample = sample[: min(64, sample.size)]
    return all(isinstance(x, (str, bytes, np.str_, np.bytes_)) or x is None for x in sample)


def convert(npz_path: Path) -> Path:
    z = np.load(npz_path, allow_pickle=True)
    cols = [str(c) for c in z["__columns__"]]
    out = {}
    for c in cols:
        v = z[c]
        if c in STR_COLS or _is_text_like(v):
            out[c] = np.asarray(v, dtype=object).astype(str)
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
