import numpy as np
import pytest

from residual import DEFAULT_CUTOFF_HZ, compute_residual, run_from_frame, summarize
from synth_log import generate


def test_clean_plant_gives_zero_residual(twin):
    run = run_from_frame(*generate("excitation", 2.0, mismatch=False, noise=False))
    res = compute_residual(run, twin)
    assert np.nanmax(np.abs(res.r)) < 1e-9
    assert res.valid.sum() == len(run.df) - 1          # forward difference drops the last row


def test_payload_appears_as_its_weight(twin):
    run = run_from_frame(*generate("hold", 2.0, payload_kg=1.0, mismatch=False, noise=False))
    res = compute_residual(run, twin)
    fz = summarize(res, twin.effort_limit)["wrench_mean_last_1s"][2]
    assert fz == pytest.approx(-9.81, abs=1e-3)        # W_ext acts on the robot: gravity pulls down


def test_residual_sign_is_measured_minus_model(twin):
    """With an unmodelled payload, r equals the payload's own joint torque, same sign."""
    run = run_from_frame(*generate("hold", 1.0, payload_kg=1.0, mismatch=False, noise=False))
    res = compute_residual(run, twin)
    k = len(run.df) - 2
    g_plant = run.arr("g")[k]                         # simulator-side g(q), includes the payload
    # not exact: the arm is still settling by ~1e-5 rad, so a little M*qdd and friction remain
    assert np.allclose(res.r[k], g_plant - twin.g(res.q[k]), atol=1e-3)


def test_filter_edges_are_dropped(twin):
    run = run_from_frame(*generate("hold", 1.0, mismatch=False, noise=False))
    res = compute_residual(run, twin, cutoff_hz=DEFAULT_CUTOFF_HZ)
    edge = int(np.ceil(3.0 / (DEFAULT_CUTOFF_HZ * run.dt)))
    assert not res.valid[:edge].any() and not res.valid[-edge:].any()
    assert res.valid[edge:-edge].all()


def test_filter_tames_differentiation_noise(twin):
    run = run_from_frame(*generate("excitation", 4.0, seed=1))
    raw = summarize(compute_residual(run, twin, cutoff_hz=None), twin.effort_limit)
    filt = summarize(compute_residual(run, twin, cutoff_hz=DEFAULT_CUTOFF_HZ), twin.effort_limit)
    assert max(filt["rms_pct_limit"]) < 1.0
    assert max(filt["rms_pct_limit"]) < 0.3 * max(raw["rms_pct_limit"])


DEV_TIMING = ("row assembled after world.step(); all quantities are post-step values; "
              "tau_measured[k] is the effort during the step that produced q[k].")


def _as_dev_layout(df, man):
    """Rewrite a synthetic log (tau[k] moves row k to k+1) in dev's layout (tau[k] produced row k)."""
    df = df.copy()
    for name in ("tau_measured", "tau_applied"):
        cols = [f"{name}_{i}" for i in range(7)]
        df[cols] = df[cols].shift(1)
    df = df.iloc[1:].reset_index(drop=True)
    man = {k: v for k, v in man.items() if k != "timing"} | {"timing_convention": DEV_TIMING}
    return run_from_frame(df, man)


def test_dev_timing_convention_is_read_and_used(twin):
    from residual import timing_from_manifest
    run = _as_dev_layout(*generate("excitation", 2.0, mismatch=False, noise=False))
    assert timing_from_manifest(run.manifest) == "backward"
    res = compute_residual(run, twin)                        # backward, from the manifest
    assert res.settings["timing"] == "backward"
    assert not res.valid[0] and res.valid[-1]               # backward: first row has no qdd
    # Not exactly zero: the model is evaluated at the post-step state while the plant stepped
    # from the pre-step state, an O(dt) difference. On smooth motion a one-step pairing error
    # is just as small (dev measured pre/post/midpoint as indistinguishable), so this test
    # checks the convention is read and gives a near-zero residual, not that the other is bad.
    assert 100 * np.nanmax(np.abs(res.r) / twin.effort_limit) < 0.1


def test_unknown_timing_warns_and_falls_back_to_central(twin):
    from residual import timing_from_manifest
    with pytest.warns(UserWarning, match="timing"):
        assert timing_from_manifest({}) == "central"


def test_tau_applied_refused_in_position_mode(twin):
    df, man = generate("hold", 0.2, mismatch=False, noise=False)
    man["drive_mode"] = "position_stiff"
    with pytest.raises(ValueError, match="position mode"):
        compute_residual(run_from_frame(df, man), twin, torque="applied")
    compute_residual(run_from_frame(df, man), twin, torque="measured")   # measured stays allowed


def test_logged_and_model_jacobian_agree_on_synthetic(twin):
    run = run_from_frame(*generate("slow_reference", 0.5, mismatch=False, noise=False))
    a = compute_residual(run, twin, jacobian="model")
    b = compute_residual(run, twin, jacobian="logged")
    assert np.allclose(a.J, b.J, atol=1e-12)
