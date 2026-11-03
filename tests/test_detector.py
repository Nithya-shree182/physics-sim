import pytest

from detector import FAULT_LABELS, detect, evaluate
from residual import DEFAULT_CUTOFF_HZ, compute_residual, run_from_frame
from synth_log import generate


def _evaluate(twin, **kw):
    run = run_from_frame(*generate(**kw))
    return evaluate(run, detect(run, compute_residual(run, twin, cutoff_hz=DEFAULT_CUTOFF_HZ)))


@pytest.mark.parametrize("traj,payload", [("excitation", 0.0), ("hold", 1.0)])
def test_no_false_alarm_on_fault_free_runs(twin, traj, payload):
    e = _evaluate(twin, traj=traj, duration=8.0, payload_kg=payload, seed=7)
    assert e["false_alarm"] is False


def test_friction_fault_found_on_the_right_joint(twin):
    e = _evaluate(twin, traj="excitation", duration=10.0, fault="friction", fault_joint=2,
                  fault_onset=5.0, seed=7)
    assert e["detected"] and not e["false_alarm"]
    assert e["channel"] == "residual" and e["suspected_joint"] == 3
    assert e["fault"] == "friction (stiction / bearing wear)"


def test_stuck_joint_found_on_the_right_joint(twin):
    e = _evaluate(twin, traj="excitation", duration=10.0, fault="stuck_joint", fault_joint=3,
                  fault_onset=5.0, seed=7)
    assert e["detected"] and e["suspected_joint"] == 4 and e["delay_s"] < 0.5


def test_command_delay_is_caught_by_tracking_not_residual(twin):
    e = _evaluate(twin, traj="excitation", duration=10.0, fault="delay", fault_joint=0,
                  fault_onset=5.0, seed=7)
    assert e["detected"] and e["channel"] == "tracking"


def test_friction_label_always_names_both_classes():
    assert FAULT_LABELS["friction"] == "friction (stiction / bearing wear)"
