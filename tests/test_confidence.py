import numpy as np
import pytest

from confidence import C_REF, conditioning, gates, pose_confidence, raw_conditioning
from poses import PROBE_POSES


def test_raw_home_matches_dev_simulator(twin):
    # dev's table (PINOCCHIO_HANDOFF.md section 8): home sigma_min 0.22485, sigma_max 1.80571, c 0.1245.
    # Dev's arm sags slightly in position mode, so allow a small difference.
    raw = raw_conditioning(twin.jacobian(PROBE_POSES["home"]))
    assert raw["sigma_min"] == pytest.approx(0.22485, rel=0.02)
    assert raw["sigma_max"] == pytest.approx(1.80571, rel=0.02)
    assert raw["c"] == pytest.approx(0.1245, rel=0.02)


def test_elbow_singular_pose_scores_lowest(twin):
    scores = {n: pose_confidence(twin, q) for n, q in PROBE_POSES.items()}
    worst = min(scores, key=lambda n: scores[n]["c"])
    assert worst == "near_singular_extended"
    assert scores["home"]["c_norm"] > 0.9


def test_vertical_score_bounded_by_c_and_one(twin):
    rng = np.random.default_rng(3)
    for _ in range(200):
        s = conditioning(twin.jacobian(rng.uniform(-1.5, 1.5, 7)))
        assert s["c"] - 1e-12 <= s["c_vertical"] <= 1 + 1e-12


def test_gates_on_named_poses(twin):
    g_home = gates(pose_confidence(twin, PROBE_POSES["home"]))
    g_ext = gates(pose_confidence(twin, PROBE_POSES["near_singular_extended"]))
    assert g_home == {"detector_ok": True, "payload_ok": True}
    assert g_ext == {"detector_ok": False, "payload_ok": False}


def test_display_score_is_clipped_to_one(twin):
    assert pose_confidence(twin, PROBE_POSES["home"])["c"] / C_REF > 0.9
    rng = np.random.default_rng(4)
    assert all(conditioning(twin.jacobian(rng.uniform(-1.5, 1.5, 7)))["c_norm"] <= 1.0 for _ in range(100))
