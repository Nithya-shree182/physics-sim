"""The calibration-window baseline in observers/detector.py."""
import numpy as np
import pytest

from conftest import REPO_ROOT
from detector import BASELINES, detect, evaluate, recalibrate_baseline
from residual import DEFAULT_CUTOFF_HZ, compute_residual, load_run, run_from_frame
from synth_log import generate


def synth(**kw):
    return run_from_frame(*generate(traj="slow_reference", duration=8.0, seed=3, **kw))


def test_rejects_unknown_mode(twin):
    res = compute_residual(synth(), twin, cutoff_hz=DEFAULT_CUTOFF_HZ)
    with pytest.raises(ValueError):
        recalibrate_baseline(res, "magic")


def test_none_changes_nothing(twin):
    res = compute_residual(synth(), twin, cutoff_hz=DEFAULT_CUTOFF_HZ)
    assert recalibrate_baseline(res, "none") is res


def test_fit_uses_only_the_calibration_window(twin):
    """Changing the residual AFTER the calibration window must not change the fit: a later fault cannot leak into it."""
    res = compute_residual(synth(), twin, cutoff_hz=DEFAULT_CUTOFF_HZ)
    a = recalibrate_baseline(res, "friction")
    r2 = res.r.copy()
    late = (res.t - res.t[0]) > 3.5
    r2[late] += 5.0
    import dataclasses
    b = recalibrate_baseline(dataclasses.replace(res, r=r2), "friction")
    early = (res.t - res.t[0]) < 3.0
    assert np.allclose(a.r[early], b.r[early], equal_nan=True)
    both = np.isfinite(a.r[late]) & np.isfinite(b.r[late])
    assert np.allclose((b.r[late] - a.r[late])[both], 5.0)


def test_baseline_narrows_the_healthy_residual(twin):
    """On a mismatched plant (masses and friction off) the healthy residual after the baseline is smaller in the calibration window."""
    res = compute_residual(synth(mismatch=True), twin, cutoff_hz=DEFAULT_CUTOFF_HZ)
    calib = ((res.t - res.t[0]) < 3.0) & res.valid
    before = np.sqrt(np.mean(res.r[calib] ** 2, axis=0))
    after = np.sqrt(np.mean(recalibrate_baseline(res, "friction").r[calib] ** 2, axis=0))
    assert np.all(after <= before + 1e-9) and after.sum() < before.sum()


def test_no_false_alarm_on_a_healthy_synthetic_run_without_the_friction_baseline(twin):
    run = synth()
    res = compute_residual(run, twin, torque="measured", cutoff_hz=DEFAULT_CUTOFF_HZ)
    for mode in ("none", "gain"):
        assert not evaluate(run, detect(run, res, baseline=mode))["false_alarm"]


@pytest.mark.parametrize("seed", [3, 4, 5])
def test_friction_baseline_needs_a_calibration_window_that_covers_the_motion(twin, seed):
    """Documented limitation: the synthetic reference does not repeat within 3 s, so the friction baseline fitted on 3 s
    extrapolates badly (false alarm on joint 5); a 6 s window covers the motion and is clean. See recalibrate_baseline."""
    run = run_from_frame(*generate(traj="slow_reference", duration=14.0, seed=seed))
    res = compute_residual(run, twin, torque="measured", cutoff_hz=DEFAULT_CUTOFF_HZ)
    assert not evaluate(run, detect(run, res, calibration_s=6.0, baseline="friction"))["false_alarm"]


C4 = REPO_ROOT / "out" / "extra"


@pytest.mark.skipif(not (C4 / "c4_friction_j2_3_0kg_s1000_1000hz.parquet").exists(), reason="out/extra runs not present")
@pytest.mark.parametrize("name", ["c4_friction_j2_3_0kg_s1000", "c4_friction_j2_3_0kg_s2001", "c4_friction_j2_3_0kg_s2002",
                                  "c4_friction_j5_3_0kg_s1000", "c4_friction_j5_3_0kg_s2001", "c4_friction_j5_3_0kg_s2002"])
def test_friction_x3_is_found_only_with_the_baseline(twin, name):
    """The friction x3 faults the analysis side asked for: missed by the original rule, found with the friction baseline."""
    run = load_run(C4 / f"{name}_1000hz.parquet")
    res = compute_residual(run, twin, torque="applied", cutoff_hz=DEFAULT_CUTOFF_HZ)
    assert not evaluate(run, detect(run, res, baseline="none"))["detected"]
    e = evaluate(run, detect(run, res, baseline="friction"))
    assert e["detected"] and not e["false_alarm"] and e["suspected_joint"] == e["true_joint"]


# ---- reference-run calibration and payload awareness --------------------------------------------------------------------------------
from detector import _smooth, against_reference, compensate_payload  # noqa: E402


def test_smoothing_holds_a_constant_at_the_edges():
    """Regression: a zero-padded 'same' convolution decayed to zero at both ends, so a constant 0.1 rad tracking offset became a huge
    false difference in the last samples and raised an alarm at the very end of a healthy run."""
    x = np.full((500, 3), 0.1)
    np.testing.assert_allclose(_smooth(x, 200), x, atol=1e-12)


def test_smoothing_is_a_moving_average_in_the_interior():
    x = np.arange(100.0)[:, None]
    np.testing.assert_allclose(_smooth(x, 5)[50], 50.0)


def test_a_run_against_itself_leaves_nothing_to_alarm_on(twin):
    run = synth()
    res = compute_residual(run, twin, torque="measured", cutoff_hz=DEFAULT_CUTOFF_HZ)
    out = against_reference(res, res)
    assert np.nanmax(np.abs(out.r[res.valid & (np.arange(len(res.t)) < len(res.t) - 200)])) < 1e-9 + 0.5 * np.nanmax(np.abs(res.r))
    assert not evaluate(run, detect(run, out))["false_alarm"]


def test_a_reference_removes_what_calibration_never_saw(twin):
    """A step that appears after the calibration window (a pickup, a new phase) alarms on its own and not against a reference that has it."""
    run = synth()
    res = compute_residual(run, twin, torque="measured", cutoff_hz=DEFAULT_CUTOFF_HZ)
    import dataclasses
    step = res.r.copy()
    ramp = np.clip((res.t - res.t[0] - 3.5) / 2.0, 0.0, 1.0)         # a slow ramp: the reference is smoothed over 0.2 s, so a sharper feature leaves corner error
    step[:, 2] += 3.0 * ramp                                         # 3 N*m on joint 3 from 4 s on
    stepped = dataclasses.replace(res, r=step)
    assert evaluate(run, detect(run, stepped))["false_alarm"]
    assert not evaluate(run, detect(run, against_reference(stepped, stepped)))["false_alarm"]


PICK = REPO_ROOT / "out" / "pick"


@pytest.mark.skipif(not (PICK / "pick_s_curve_1kg_ref_1000hz.parquet").exists(), reason="out/pick runs not present")
def test_the_healthy_pick_needs_a_reference_and_every_fault_in_it_is_found(twin):
    """The finding on real runs: calibrating on the first 3 s false-alarms on a healthy pick; calibrating on a healthy run of the same task
    (a different noise seed) does not, and the faults are still found."""
    from residual import fault_torque
    ref_run = load_run(PICK / "pick_s_curve_1kg_ref_1000hz.parquet")
    ref = compute_residual(ref_run, twin, torque="applied", cutoff_hz=DEFAULT_CUTOFF_HZ)

    def verdict(name, use_ref):
        run = load_run(PICK / f"{name}_1000hz.parquet")
        res = compute_residual(run, twin, torque=fault_torque(run), cutoff_hz=DEFAULT_CUTOFF_HZ)
        if use_ref:
            res = against_reference(res, ref)
        return evaluate(run, detect(run, res, reference_run=ref_run if use_ref else None))
    assert verdict("pick_s_curve_1kg", False)["false_alarm"]            # the problem
    assert not verdict("pick_s_curve_1kg", True)["false_alarm"]         # the fix
    for name in ("pick_fault_delay_all_50ms_1kg", "pick_fault_friction_j5_8_1kg", "pick_fault_friction_j3_8_1kg",
                 "pick_fault_stuck_joint_j3_200_1kg"):
        e = verdict(name, True)
        assert e["detected"] and not e["false_alarm"], name
