import numpy as np

from synth_log import generate
from validate import load_schema, validate_frame, validate_manifest

SCHEMA = load_schema()


def _errors(df, man):
    errors, _ = validate_frame(df, SCHEMA)
    return errors + validate_manifest(man, SCHEMA, df)


def test_synthetic_run_is_valid():
    assert _errors(*generate("hold", 0.2, noise=False)) == []


def test_sim_style_manifest_with_decimation_is_valid():
    import pal_robot
    spec = pal_robot.active()
    df, man = generate("hold", 0.2, noise=False)
    man = {k: v for k, v in man.items() if k != "timing"}
    man |= {"timing_convention": "row assembled after world.step(); ...", "mj_every": 5,
            "rows": len(df), "payload_attachment": {"body": spec.tool_frame,
                                                    "offset_m": list(spec.tool_offset)}}
    df["mj_fresh"] = (np.arange(len(df)) % 5) == 0
    errors, warnings = validate_frame(df, SCHEMA)
    assert errors == [] and warnings == []             # mj_fresh is a known optional column
    assert validate_manifest(man, SCHEMA, df) == []


def test_missing_timing_and_missing_mj_fresh_are_errors():
    """A log that does not say which step's torque goes with which row cannot be used for a
    residual at all, so its absence has to be an error rather than a warning. Both spellings are
    stripped here: `timing` is the synthetic form and `timing_convention` the simulator's."""
    df, man = generate("hold", 0.2, noise=False)
    man = {k: v for k, v in man.items() if k not in ("timing", "timing_convention")}
    man |= {"mj_every": 5, "rows": len(df) + 1}
    errors = validate_manifest(man, SCHEMA, df)
    assert any("timing" in e for e in errors)
    assert any("mj_fresh" in e for e in errors)
    assert any("rows" in e for e in errors)
