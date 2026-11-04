"""The O5 findings on real simulator runs, as regression tests (skipped when the runs are not present).

The runs live in out/ (git-ignored, ~4 MB each); regenerate with tools/pal_suite.sh o5.
Each test states one finding from docs/DETECTION_RESULTS.md and docs/REVIEW.md.
"""
from pathlib import Path

import pytest

from conftest import REPO_ROOT
from detector import detect, evaluate
from residual import DEFAULT_CUTOFF_HZ, compute_residual, fault_torque, load_run

O5 = REPO_ROOT / "out" / "o5"
pytestmark = pytest.mark.skipif(not (O5 / "o5_healthy_s0_1000hz.parquet").exists(), reason="out/o5 runs not present")


def verdict(twin, name, torque):
    run = load_run(O5 / f"{name}_1000hz.parquet")
    res = compute_residual(run, twin, torque=torque, cutoff_hz=DEFAULT_CUTOFF_HZ)
    return evaluate(run, detect(run, res))


@pytest.mark.parametrize("name", ["o5_friction_j1_8", "o5_friction_j2_8", "o5_friction_j3_8", "o5_friction_j5_8",
                                  "o5_friction_j3_4", "o5_friction_j5_4"])
def test_friction_is_found_in_the_applied_torque(twin, name):
    e = verdict(twin, name, "applied")
    assert e["detected"] and e["channel"] == "residual" and e["suspected_joint"] == e["true_joint"]


@pytest.mark.parametrize("name", ["o5_friction_j1_8", "o5_friction_j2_8", "o5_friction_j3_8", "o5_friction_j5_8"])
def test_friction_is_invisible_in_the_measured_torque(twin, name):
    """tau_measured is the net joint torque: friction cannot appear in it (REVIEW F2)."""
    assert not verdict(twin, name, "measured")["detected"]


def test_locked_joint_is_found_on_the_residual_with_the_applied_torque(twin):
    e = verdict(twin, "o5_stuck_joint_j3_200", "applied")
    assert e["detected"] and e["channel"] == "residual" and e["delay_s"] < 0.1


def test_no_false_alarms_on_healthy_runs(twin):
    for s in range(6):
        e = verdict(twin, f"o5_healthy_s{s}", "applied")
        assert not e["false_alarm"]


def test_known_misses_stay_documented(twin):
    """Backlash and short delays are invisible by construction at 20% speed (REVIEW F10); if one is ever found, the docs are stale."""
    for name in ("o5_backlash_j0_0.12", "o5_delay_all_10ms", "o5_delay_all_30ms"):
        assert not verdict(twin, name, "applied")["detected"], name


def test_fault_torque_rule():
    run = load_run(O5 / "o5_healthy_s0_1000hz.parquet")
    assert fault_torque(run) == "applied"          # effort-mode log
