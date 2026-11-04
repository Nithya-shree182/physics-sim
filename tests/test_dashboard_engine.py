"""Smoke test for the dashboard's engine: the live stand-in plant plus the analysis on top.

Drives the stand-in the way the guided demo does and checks that the right alert appears and the
payload estimate follows. About 15 s. Stand-in numbers prove the pipeline works; they are not Isaac Sim's.
"""
import sys

import pytest

from conftest import REPO_ROOT

sys.path.insert(0, str(REPO_ROOT / "dashboard"))
from engine import Analyzer, LiveSource  # noqa: E402


def _run(src, an, seconds, **controls):
    for k, v in controls.items():
        setattr(src.c, k, v)
    snap = {}
    for _ in range(int(seconds / 0.05)):
        snap = an.feed(src.next_block(), src.c, src.t)
    return snap


@pytest.fixture(scope="module")
def session():
    src = LiveSource()
    an = Analyzer(src.twin, "forward")
    snap = _run(src, an, 6.0)                     # calibration (3 s) plus a healthy stretch
    return src, an, snap


def test_healthy_run_is_calibrated_and_quiet(session):
    _, an, snap = session
    assert an.cal["ready"] and not an.alerts
    assert abs(snap["est"]) < 0.1


def test_pickup_raises_payload_alert_and_estimate_converges(session):
    src, an, _ = session
    snap = _run(src, an, 8.0, load=1.2)
    assert snap["est"] == pytest.approx(1.2, abs=0.1)
    assert "payload" in an.alerts and not any(a.startswith(("fric", "stuck", "delay")) for a in an.alerts)


def test_friction_fault_is_named_on_the_right_joint(session):
    src, an, _ = session
    _run(src, an, 4.0, load=0.0)
    _run(src, an, 8.0, fault="friction", fault_joint=2)
    assert "fric2" in an.alerts and an.alerts["fric2"]["title"] == "friction (stiction / bearing wear)"


def test_command_delay_is_caught_by_the_tracking_channel(session):
    src, an, _ = session
    _run(src, an, 4.0, fault="none")
    _run(src, an, 8.0, fault="delay")
    assert "delay" in an.alerts and not any(a.startswith("fric") for a in an.alerts)


def test_speed_is_capped_at_the_trusted_band():
    from types import SimpleNamespace
    from engine import BAND, Controls, Engine
    stub = SimpleNamespace(controls=Controls(), an=SimpleNamespace(snapshot={}), source=SimpleNamespace(kind="isaac"))
    Engine.set_controls(stub, speeds=[10.0, 50.0, 70.0, 100.0, 5.0, 0.0, 49.0])
    assert max(stub.controls.speeds) <= BAND * 100
    assert stub.controls.speeds[2] == stub.controls.speeds[3] == BAND * 100
