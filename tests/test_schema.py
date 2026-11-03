import numpy as np

from synth_log import generate
from validate import load_schema, validate_frame, validate_manifest

SCHEMA = load_schema()


def _errors(df, man):
    errors, _ = validate_frame(df, SCHEMA)
    return errors + validate_manifest(man, SCHEMA, df)


def test_synthetic_run_is_valid():
    assert _errors(*generate("hold", 0.2, noise=False)) == []


def test_dev_style_manifest_with_decimation_is_valid():
    df, man = generate("hold", 0.2, noise=False)
    man = {k: v for k, v in man.items() if k != "timing"}
    man |= {"timing_convention": "row assembled after world.step(); ...", "mj_every": 5,
            "rows": len(df), "payload_attachment": {"body": "panda_hand", "offset_m": [0, 0, 0.1034]}}
    df["mj_fresh"] = (np.arange(len(df)) % 5) == 0
    errors, warnings = validate_frame(df, SCHEMA)
    assert errors == [] and warnings == []             # mj_fresh is a known optional column
    assert validate_manifest(man, SCHEMA, df) == []


def test_missing_timing_and_missing_mj_fresh_are_errors():
    df, man = generate("hold", 0.2, noise=False)
    man = {k: v for k, v in man.items() if k != "timing"} | {"mj_every": 5, "rows": len(df) + 1}
    errors = validate_manifest(man, SCHEMA, df)
    assert any("timing" in e for e in errors)
    assert any("mj_fresh" in e for e in errors)
    assert any("rows" in e for e in errors)
