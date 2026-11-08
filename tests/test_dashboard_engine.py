"""Smoke test for the dashboard's engine: the live stand-in plant plus the analysis on top.

Drives the stand-in the way the guided demo does and checks that the right alert appears and the
payload estimate follows. About 15 s. Stand-in numbers prove the pipeline works; they are not Isaac Sim's.
"""
import sys

import numpy as np

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


def test_a_healthy_pick_is_recorded_as_the_reference_and_the_same_pick_matches_it():
    from engine import Controls
    key = ("s_curve", 1.0, 0.6, (0.45, -0.25))
    an = Analyzer(LiveSource().twin, "forward", picks=True)
    ctl = Controls()
    def run(src):
        an.reset_window(); an.recalibrate(); an.begin_run(key, 0.0, "none")
        for _ in range(int(6.0 / 0.05)):
            blk = src.next_block()
            blk.truth = dict(blk.truth, fault="none")
            an.feed(blk, ctl, 0.0)
        mode = (an.recording, an.cur_ref is not None)
        an.end_run()
        return mode
    assert run(LiveSource(seed=1)) == (True, False)             # first time: recorded
    assert key in an.refs and len(an.refs[key]["t"]) > 20
    assert run(LiveSource(seed=1)) == (False, True)             # second time: compared with it
    assert float(np.abs(an.last_ue[0]).max()) < 1e-9            # the same pick leaves nothing


def test_narrator_is_rule_based_and_states_only_known_numbers():
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "dashboard"))
    import views as V
    snap = {"summary": {"picks": True, "running": False, "ref": "compared", "peak": (2, 0.4), "alerts": [],
                        "info": {"kg": 2.0, "boxes": 3, "style": "s_curve", "fault": "none"},
                        "boxes": [{"est": 2.01, "n": 80}, {"est": 1.99, "n": 80}, {"est": 2.0, "n": 80}]}}
    text = " ".join(V.narrate(snap, {"last": {"ok": True, "aborted": False, "boxes_placed": 3, "boxes": 3}}))
    assert "all 3 on the belt" in text and "within tolerance" in text and "No alarm" in text
    snap["summary"]["info"]["fault"] = "friction"
    snap["summary"]["info"]["joint"] = 3
    text = " ".join(V.narrate(snap, None))
    assert "not reliable" in text and "joint 4" in text
    assert V.narrate({"summary": {"picks": False, "info": {}}}, None)[0].startswith("No cycle yet")


def test_style_explanation_reads_its_numbers_from_the_benchmark():
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "dashboard"))
    import views as V
    mk = lambda st, kg, t, p: {"style": st, "payload_kg": kg, "cycle_time_s": t, "peak_torque_frac": p, "energy_drawn_J": 100.0, "track_err_max_rad": 0.01}
    d = {"runs": [mk("trapezoidal", 0, 5.0, 0.4), mk("trapezoidal", 2, 5.5, 0.42), mk("s_curve", 0, 6.0, 0.35), mk("torque_aware", 0, 9.0, 0.3)]}
    text = " ".join(t for _, t in V.explain_styles(d))
    assert "5.0 s" in text and "6.0 s" in text and "9.0 s" in text
