"""Validate a telemetry log (parquet + JSON manifest) against schema/telemetry_v0.yaml.

    python schema/validate.py data/<run_id>.parquet
    python schema/validate.py data/<run_id>.parquet --manifest other/place.json

Checks:
  - every required column is present, with the right type
  - no missing values except in columns marked nullable
  - label columns only use their allowed values
  - step goes up by exactly 1, t increases, dt is positive and matches the t spacing
  - M is symmetric with a positive diagonal, ee_quat is a unit quaternion
  - fault_joint is -1 exactly when fault_type is none
  - the manifest has every required field and its physics_hz matches dt

Exit code 0 means no errors. Warnings (for example unknown extra columns) do not fail.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from itertools import product
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

SCHEMA_PATH = Path(__file__).resolve().parent / "telemetry_v0.yaml"

# placeholder in a field name -> key in the field spec that gives its values
_EXPANDERS = {"i": "range", "r": "rows", "c": "cols", "a": "axes", "b": "bodies", "k": "components"}


def load_schema(path: Path = SCHEMA_PATH) -> dict:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def _values(spec: dict, key: str) -> list:
    v = spec[key]
    if key == "axes":
        return list(v)
    lo, hi = v
    return list(range(lo, hi + 1))


def expand_field(spec: dict) -> list[str]:
    """'q_{i}' with range [0, 6] -> ['q_0', ..., 'q_6']."""
    name = spec["name"]
    holders = re.findall(r"\{(\w)\}", name)
    if not holders:
        return [name]
    pools = [_values(spec, _EXPANDERS[h]) for h in holders]
    out = []
    for combo in product(*pools):
        out.append(name.format(**dict(zip(holders, combo))))
    return out


def required_fields(schema: dict) -> dict[str, dict]:
    """Expanded column name -> its field spec, for every required column."""
    out = {}
    for group in schema["columns"]:
        for spec in group["fields"]:
            for col in expand_field(spec):
                out[col] = spec
    return out


def optional_fields(schema: dict) -> set[str]:
    return {col for g in schema.get("optional_columns", []) for s in g["fields"] for col in expand_field(s)}


def _dtype_ok(series: pd.Series, dtype: str) -> bool:
    if dtype.startswith("float"):
        return pd.api.types.is_float_dtype(series)
    if dtype.startswith("int"):
        return pd.api.types.is_integer_dtype(series)
    if dtype == "bool":
        return pd.api.types.is_bool_dtype(series)
    if dtype == "string":
        return pd.api.types.is_string_dtype(series) or pd.api.types.is_object_dtype(series)
    return False


def validate_frame(df: pd.DataFrame, schema: dict) -> tuple[list[str], list[str]]:
    errors, warnings = [], []
    fields = required_fields(schema)

    missing = [c for c in fields if c not in df.columns]
    if missing:
        errors.append(f"{len(missing)} required columns missing, e.g. {missing[:8]}")
    extra = set(df.columns) - set(fields) - optional_fields(schema)
    if extra:
        warnings.append(f"columns not in the schema: {sorted(extra)[:8]}")

    for col, spec in fields.items():
        if col not in df.columns:
            continue
        s = df[col]
        if not _dtype_ok(s, spec["dtype"]):
            errors.append(f"{col}: expected {spec['dtype']}, got {s.dtype}")
        if not spec.get("nullable", False) and s.isna().any():
            errors.append(f"{col}: {int(s.isna().sum())} missing values in a non-nullable column")
        if "allowed" in spec:
            bad = set(s.dropna().unique()) - set(spec["allowed"])
            if bad:
                errors.append(f"{col}: values not allowed {sorted(map(str, bad))}")

    if len(df) == 0:
        return errors + ["file has no rows"], warnings

    # Each consistency check runs only if its columns are present and correctly typed, so one
    # missing column does not hide unrelated problems such as dropped frames.
    bad_cols = {c for c in fields if c not in df.columns or not _dtype_ok(df[c], fields[c]["dtype"])}

    def have(cols) -> bool:
        return not (set(cols) & bad_cols)

    if have(["step"]) and np.any(np.diff(df["step"].to_numpy()) != 1):
        errors.append("step does not go up by exactly 1 every row (dropped or repeated frames)")
    if have(["t", "dt"]):
        t, dt = df["t"].to_numpy(), df["dt"].to_numpy()
        if np.any(np.diff(t) <= 0):
            errors.append("t is not strictly increasing")
        if np.any(dt <= 0):
            errors.append("dt has non-positive values")
        if len(df) > 1 and not np.allclose(np.diff(t), dt[1:], rtol=1e-6, atol=1e-9):
            errors.append("dt does not match the spacing of t")

    m_cols = [f"M_{r}_{c}" for r in range(7) for c in range(7)]
    if have(m_cols):
        M = df[m_cols].to_numpy().reshape(-1, 7, 7)
        asym = np.max(np.abs(M - M.transpose(0, 2, 1)))
        if asym > 1e-6:
            errors.append(f"M is not symmetric (worst asymmetry {asym:.2e})")
        if np.any(np.diagonal(M, axis1=1, axis2=2) <= 0):
            errors.append("M has a non-positive diagonal entry")

    q_cols = [f"ee_quat_{a}" for a in "wxyz"]
    if have(q_cols):
        norm_err = np.max(np.abs(np.linalg.norm(df[q_cols].to_numpy(), axis=1) - 1.0))
        if norm_err > 1e-4:
            errors.append(f"ee_quat is not unit length (worst error {norm_err:.2e})")

    if have(["fault_type", "fault_joint"]):
        no_fault = df["fault_type"] == "none"
        if np.any(no_fault != (df["fault_joint"] == -1)):
            errors.append("fault_joint must be -1 exactly when fault_type is none")

    return errors, warnings


def validate_manifest(manifest: dict, schema: dict, df: pd.DataFrame | None = None) -> list[str]:
    errors = [f"manifest missing '{k}'" for k in schema["manifest"]["required"] if k not in manifest]
    if manifest.get("schema_version") not in (None, schema["version"]):
        errors.append(f"manifest schema_version {manifest['schema_version']} != {schema['version']}")
    if df is not None and "physics_hz" in manifest and "dt" in df and len(df):
        expected = 1.0 / float(manifest["physics_hz"])
        if not np.allclose(df["dt"].to_numpy(), expected, rtol=1e-6):
            errors.append(f"dt does not equal 1/physics_hz = {expected:.6g} s")
    return errors


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("parquet", type=Path)
    ap.add_argument("--manifest", type=Path, default=None, help="defaults to <parquet stem>.json beside it")
    ap.add_argument("--schema", type=Path, default=SCHEMA_PATH)
    args = ap.parse_args()

    schema = load_schema(args.schema)
    df = pd.read_parquet(args.parquet)
    errors, warnings = validate_frame(df, schema)

    manifest_path = args.manifest or args.parquet.with_suffix(".json")
    if manifest_path.is_file():
        errors += validate_manifest(json.loads(manifest_path.read_text(encoding="utf-8")), schema, df)
    else:
        errors.append(f"manifest not found: {manifest_path}")

    print(f"{args.parquet.name}: {len(df)} rows, {len(df.columns)} columns, schema v{schema['version']}")
    for w in warnings:
        print(f"  WARNING  {w}")
    for e in errors:
        print(f"  ERROR    {e}")
    print("  VALID" if not errors else f"  INVALID ({len(errors)} errors)")
    return 0 if not errors else 1


if __name__ == "__main__":
    sys.exit(main())
