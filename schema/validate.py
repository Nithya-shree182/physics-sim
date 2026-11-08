"""Validate a telemetry log (parquet + JSON manifest) against schema/telemetry_v1.yaml.

    python schema/validate.py data/<run_id>.parquet
    python schema/validate.py data/<run_id>.parquet --manifest other/place.json

Checks:
  - every required column is present, with the right type
  - no missing values except in columns marked nullable
  - label columns only use their allowed values
  - step goes up by exactly 1, t increases, dt is positive and matches the t spacing
  - M is symmetric with a positive diagonal, ee_quat is a unit quaternion
  - fault_joint is -1 exactly when fault_type is none
  - the torque decomposition adds up: g + inertial + coriolis + friction + unexplained == applied
  - the energy columns are consistent: E_tot_rel == E_kin + E_pot_rel
  - the aggregates match their parts: P_mech == sum(P_joint), tau_frac_max == max(tau_frac_i)
  - tau_frac_i really is |tau_measured_i| / tau_max_i from the manifest
  - a held box has an index: grip_active implies box_index >= 0
  - the manifest has every required field and its physics_hz matches dt
  - the manifest states a timing convention; mj_every > 1 comes with an mj_fresh column;
    a stated row count matches the file

Exit code 0 means no errors. Warnings (for example unknown extra columns) do not fail.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

from itertools import product

import numpy as np
import pandas as pd
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "workspace"))
import pal_robot  # noqa: E402

SPEC = pal_robot.active()
DOF = SPEC.dof
SCHEMA_PATH = Path(__file__).resolve().parent / "telemetry_v1.yaml"

# placeholder in a field name -> key in the field spec that gives its values
_EXPANDERS = {"i": "range", "r": "rows", "c": "cols", "a": "axes", "b": "bodies", "k": "components"}


def load_schema(path: Path = SCHEMA_PATH) -> dict:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def _values(spec: dict, key: str) -> list:
    """The values a placeholder expands over.

    A per-joint column says `per_joint: true` rather than a literal range, so the schema does not
    carry the joint count: it comes from the robot spec, and the same schema validates any arm.
    That is the whole point of the change from v0, where [0, 6] was written out seven times.
    """
    if key in ("range", "cols") and spec.get("per_joint" if key == "range" else "per_joint_cols"):
        return list(range(DOF))
    if spec.get("per_joint_matrix") and key in ("rows", "cols"):
        return list(range(DOF))
    v = spec[key]
    if key == "axes":
        return list(v)
    lo, hi = v
    return list(range(lo, hi + 1))


def expand_field(spec: dict) -> list[str]:
    """'q_{i}' with per_joint -> ['q_0', ..., 'q_{dof-1}']."""
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


NOISE_SIGMAS = 6.0      # an identity may differ by this many torque-noise sigmas when the sensor is noisy


def torque_noise_sigma(manifest: dict | None) -> float:
    """The torque sigma the run's sensor model injected (0 for ideal sensing or no record). The logger
    derives the decomposition and tau_frac from clean values and THEN corrupts tau_measured and
    tau_applied, so on a noisy run those exact identities hold only to within the noise."""
    try:
        return float(((manifest or {}).get("sensor_noise") or {}).get("sigma", {}).get("tau", 0.0) or 0.0)
    except (TypeError, ValueError):
        return 0.0


def validate_frame(df: pd.DataFrame, schema: dict, tau_sigma: float = 0.0) -> tuple[list[str], list[str]]:
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

    def joint_cols(prefix):
        return [f"{prefix}_{i}" for i in range(DOF)]

    m_cols = [f"M_{r}_{c}" for r in range(DOF) for c in range(DOF)]
    if have(m_cols):
        M = df[m_cols].to_numpy().reshape(-1, DOF, DOF)
        asym = np.max(np.abs(M - M.transpose(0, 2, 1)))
        if asym > 1e-6:
            errors.append(f"M is not symmetric (worst asymmetry {asym:.2e})")
        if np.any(np.diagonal(M, axis1=1, axis2=2) <= 0):
            errors.append("M has a non-positive diagonal entry")

    # The decomposition is an identity by construction, so any disagreement means a column was
    # written from different state than its neighbours — a real and otherwise invisible bug.
    parts = ["g", "coriolis", "tau_inertial", "tau_friction", "tau_unexplained"]
    if have([c for p in parts for c in joint_cols(p)] + joint_cols("tau_applied")):
        total = sum(df[joint_cols(p)].to_numpy() for p in parts)
        worst = float(np.nanmax(np.abs(total - df[joint_cols("tau_applied")].to_numpy())))
        if worst > 1e-6 + NOISE_SIGMAS * tau_sigma:
            errors.append(f"the torque decomposition does not add up to tau_applied "
                          f"(worst {worst:.2e} N*m)")

    if have(["E_kin", "E_pot_rel", "E_tot_rel"]):
        worst = float(np.nanmax(np.abs(df["E_kin"] + df["E_pot_rel"] - df["E_tot_rel"])))
        if worst > 1e-9:
            errors.append(f"E_tot_rel != E_kin + E_pot_rel (worst {worst:.2e} J)")
        if float(df["E_kin"].min()) < -1e-12:
            errors.append("E_kin is negative, which a positive-definite mass matrix forbids")
        if abs(float(df["E_pot_rel"].iloc[0])) > 1e-12:
            errors.append("E_pot_rel is relative to row 0 and must start at zero")

    if have(["P_mech"] + joint_cols("P_joint")):
        worst = float(np.nanmax(np.abs(df[joint_cols("P_joint")].to_numpy().sum(axis=1) - df["P_mech"])))
        if worst > 1e-6:
            errors.append(f"P_mech != sum(P_joint) (worst {worst:.2e} W)")

    if have(["tau_frac_max"] + joint_cols("tau_frac")):
        worst = float(np.nanmax(np.abs(df[joint_cols("tau_frac")].to_numpy().max(axis=1)
                                       - df["tau_frac_max"])))
        if worst > 1e-9:
            errors.append(f"tau_frac_max != max(tau_frac_i) (worst {worst:.2e})")

    if have(["grip_active", "box_index"]):
        held = df["grip_active"].to_numpy().astype(bool)
        if np.any(held & (df["box_index"].to_numpy() < 0)):
            errors.append("grip_active is true on rows where box_index is -1")

    if have(["sigma_min", "sigma_max"]):
        if np.any(df["sigma_min"].to_numpy() > df["sigma_max"].to_numpy() + 1e-12):
            errors.append("sigma_min exceeds sigma_max")

    q_cols = [f"ee_quat_{a}" for a in "wxyz"]
    if have(q_cols):
        quat = df[q_cols].to_numpy()
        rows = np.all(np.isfinite(quat), axis=1)        # the pose is nullable; check what is there
        if rows.any():
            norm_err = np.max(np.abs(np.linalg.norm(quat[rows], axis=1) - 1.0))
            if norm_err > 1e-4:
                errors.append(f"ee_quat is not unit length (worst error {norm_err:.2e})")
        if not rows.all():
            warnings.append(f"ee_quat/ee_pos absent on {int((~rows).sum())} rows "
                            f"(a capture with no USD stage)")

    if have(["fault_type", "fault_joint"]):
        no_fault = df["fault_type"] == "none"
        if np.any(no_fault != (df["fault_joint"] == -1)):
            errors.append("fault_joint must be -1 exactly when fault_type is none")

    return errors, warnings


def validate_manifest(manifest: dict, schema: dict, df: pd.DataFrame | None = None) -> list[str]:
    tau_sigma = torque_noise_sigma(manifest)
    errors = [f"manifest missing '{k}'" for k in schema["manifest"]["required"] if k not in manifest]
    if manifest.get("schema_version") not in (None, schema["version"]):
        errors.append(f"manifest schema_version {manifest['schema_version']} != {schema['version']}")
    if df is not None and "physics_hz" in manifest and "dt" in df and len(df):
        expected = 1.0 / float(manifest["physics_hz"])
        if not np.allclose(df["dt"].to_numpy(), expected, rtol=1e-6):
            errors.append(f"dt does not equal 1/physics_hz = {expected:.6g} s")
    if df is not None and int(manifest.get("mj_every", 1) or 1) > 1 and "mj_fresh" not in df:
        errors.append("manifest mj_every > 1 but the mj_fresh column is missing")
    if df is not None and "rows" in manifest and int(manifest["rows"]) != len(df):
        errors.append(f"manifest says {manifest['rows']} rows, file has {len(df)}")
    if df is not None and "dof" in manifest and int(manifest["dof"]) != DOF:
        errors.append(f"manifest says {manifest['dof']} joints, this build's arm "
                      f"({SPEC.name}) has {DOF}; validate with the matching PAL_ROBOT")
    # tau_frac is only re-derivable if the limits it was divided by are on record.
    if df is not None and "tau_max" in manifest:
        tm = np.asarray(manifest["tau_max"], dtype=float)
        cols = [f"tau_frac_{i}" for i in range(DOF)]
        meas = [f"tau_measured_{i}" for i in range(DOF)]
        if tm.shape == (DOF,) and all(c in df for c in cols + meas):
            want = np.abs(df[meas].to_numpy()) / tm
            worst = float(np.nanmax(np.abs(want - df[cols].to_numpy())))
            if worst > 1e-6 + NOISE_SIGMAS * tau_sigma / float(np.min(tm)):
                errors.append(f"tau_frac does not equal |tau_measured| / manifest tau_max "
                              f"(worst {worst:.2e})")
    if "timing" not in manifest and "timing_convention" not in manifest:
        errors.append("manifest states no timing convention ('timing' or 'timing_convention')")
    return errors


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("parquet", type=Path)
    ap.add_argument("--manifest", type=Path, default=None, help="defaults to <parquet stem>.json beside it")
    ap.add_argument("--schema", type=Path, default=SCHEMA_PATH)
    args = ap.parse_args()

    schema = load_schema(args.schema)
    df = pd.read_parquet(args.parquet)
    manifest_path = args.manifest or args.parquet.with_suffix(".json")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.is_file() else None
    errors, warnings = validate_frame(df, schema, torque_noise_sigma(manifest))

    if manifest is not None:
        errors += validate_manifest(manifest, schema, df)
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
