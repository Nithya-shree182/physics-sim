"""The proposal's evaluation table (section 9), one test per row where it can be checked now.

| Objective | Pass condition (proposal)                                 | Here |
| O1        | no dropped frames across a full run                       | real + synthetic logs |
| O2        | slow unloaded run within 5 percent per joint              | real log |
| O3        | estimates visibly suppressed near singular poses          | confidence, detector |
| O4        | error against known masses; unobservable case flagged     | synthetic ladder |
| O5        | detection and false-positive rate, with and without gate  | synthetic fault set |
| Validation| applied force against estimated force                     | test_physics_model.py |

Synthetic results prove the code works; they are optimistic because the plant is built from
the same URDF as the twin (mass mismatch and extra friction soften that, as the proposal
asks). Numbers to quote come from Isaac Sim logs.
"""
import json

import numpy as np
import pandas as pd
import pytest

from conftest import REPO_ROOT
from confidence import conditioning, gates
from detector import detect, evaluate
from payload import estimate_payload
from poses import PROBE_POSES
from residual import DEFAULT_CUTOFF_HZ, compute_residual, load_run, run_from_frame
from synth_log import generate

REAL = REPO_ROOT / "data"        # our tracked reference runs (the old data/real/ logs were on a wrong-friction plant)
REAL_RUNS = [REAL / "ref_matched_0kg_1000hz.parquet", REAL / "ref_matched_1kg_1000hz.parquet"]
have_real = pytest.mark.skipif(not all(p.exists() for p in REAL_RUNS), reason="data/ref_matched_* reference runs not present")
LIMITS = np.array([87, 87, 87, 87, 12, 12, 12.0])           # N*m, datasheet


# ── O1: no dropped frames ──────────────────────────────────────────────────────────────
REQUIRED = [f"{p}_{j}" for p in ("q", "qd", "tau_measured") for j in range(7)] + ["t"]


def _assert_complete(df, dt):
    assert not df[REQUIRED].isna().any().any(), "NaN in a required column"
    steps = np.diff(df["t"].to_numpy())
    assert np.allclose(steps, dt, atol=dt * 1e-3), "a frame was dropped or repeated"


@have_real
@pytest.mark.parametrize("path", REAL_RUNS)
def test_o1_real_logs_have_one_frame_per_physics_step(path):
    man = json.loads(path.with_suffix(".json").read_text())
    df = pd.read_parquet(path)
    assert len(df) == man["rows"]
    _assert_complete(df, 1.0 / 1000.0)


def test_o1_synthetic_log_has_one_frame_per_physics_step():
    df, man = generate(traj="slow_reference", duration=2.0, seed=1)
    _assert_complete(df, 1.0 / 1000.0)
    assert len(df) == 2000


# ── O2: model matches the simulator on a slow, unloaded run ────────────────────────────
@have_real
def test_o2_slow_unloaded_run_is_within_five_percent_per_joint(twin):
    """The proposal's calibration gate, measured as dev does: per-joint RMS residual as a
    fraction of that joint's torque limit."""
    res = compute_residual(load_run(REAL_RUNS[0]), twin)
    pct = 100 * np.sqrt(np.nanmean(res.r ** 2, axis=0)) / LIMITS
    assert np.all(pct < 5.0), f"per-joint residual, % of limit: {np.round(pct, 2)}"


@have_real
def test_o2_floor_stays_inside_dev_trusted_band(twin):
    """Dev's RESIDUAL_FLOOR.md: at or below 50% of the velocity limit the floor is under
    about 2% of a limit. The slow reference run is far inside that band."""
    run = load_run(REAL_RUNS[0])
    qd_max = np.array([2.175] * 4 + [2.61] * 3)
    assert (np.abs(run.arr("qd")).max(axis=0) / qd_max).max() < 0.5
    res = compute_residual(run, twin)
    assert (np.sqrt(np.nanmean(res.r ** 2, axis=0)) / LIMITS).max() < 0.02


def test_o2_calibration_gate_on_synthetic_matched_plant(twin):
    """Without mismatch and noise the identity is exact, so the gate is met trivially; it is
    here to catch a regression in the pipeline itself."""
    run = run_from_frame(*generate(traj="slow_reference", duration=6.0, mismatch=False, noise=False))
    res = compute_residual(run, twin, cutoff_hz=DEFAULT_CUTOFF_HZ)
    assert (np.sqrt(np.nanmean(res.r ** 2, axis=0)) / LIMITS).max() < 0.005


# ── O3: confidence ─────────────────────────────────────────────────────────────────────
def test_o3_confidence_is_a_zero_to_one_ratio_of_singular_values(twin):
    for name, q in PROBE_POSES.items():
        c = conditioning(twin.jacobian(q))
        assert 0.0 <= c["c"] <= 1.0, name
    # the proposal's definition: c = sigma_min / sigma_max
    s = np.linalg.svd(twin.jacobian(PROBE_POSES["home"]), compute_uv=False)
    assert s.min() / s.max() <= 1.0


def test_o3_singular_pose_is_gated_and_healthy_pose_is_not(twin):
    ok = gates(conditioning(twin.jacobian(PROBE_POSES["home"])))
    bad = gates(conditioning(twin.jacobian(PROBE_POSES["near_singular_extended"])))
    assert ok["detector_ok"] and ok["payload_ok"]
    assert not bad["detector_ok"] and not bad["payload_ok"]


def test_o3_detector_never_alarms_on_a_gated_row(twin):
    run = run_from_frame(*generate(traj="excitation", duration=8.0, fault="friction", fault_joint=2,
                                   fault_onset=4.0, seed=3))
    det = detect(run, compute_residual(run, twin, cutoff_hz=DEFAULT_CUTOFF_HZ))
    assert not (det.alarm & det.gated).any()


# ── O4: payload against known masses ───────────────────────────────────────────────────
@pytest.mark.parametrize("mass", [0.5, 1.0, 2.0, 3.0])
def test_o4_known_masses_are_recovered_after_tare(twin, mass):
    """Tare from the empty gripper on the same trajectory, as for the real logs. Tolerance
    stated up front: 5% of the mass or 30 g, whichever is larger."""
    kw = dict(traj="slow_reference", duration=8.0, seed=2)
    empty = estimate_payload(compute_residual(run_from_frame(*generate(payload_kg=0.0, **kw)), twin), twin)
    held = estimate_payload(compute_residual(run_from_frame(*generate(payload_kg=mass, **kw)), twin), twin,
                            tare_kg=empty.final[0])
    est, band = held.final
    assert abs(est - mass) <= max(0.05 * mass, 0.03), f"{mass} kg read as {est:.3f} +- {band:.3f}"


@have_real
def test_o4_real_one_kg_within_tolerance_after_tare(twin):
    m0, _ = estimate_payload(compute_residual(load_run(REAL_RUNS[0]), twin), twin).final
    m1, _ = estimate_payload(compute_residual(load_run(REAL_RUNS[1]), twin), twin, tare_kg=m0).final
    assert abs(m1 - 1.0) <= 0.03


def test_o4_unobservable_pose_is_flagged_not_reported(twin):
    """Near the elbow-singular pose the vertical force cannot be separated from torque
    noise: the payload gate must close so the estimator holds instead of reporting."""
    q = PROBE_POSES["near_singular_extended"]
    assert conditioning(twin.jacobian(q))["vertical_noise_gain"] > 10
    assert not gates(conditioning(twin.jacobian(q)))["payload_ok"]


# ── O5: detection and false-positive rate, with and without the confidence gate ────────
FAULT_SET = [("friction", 2), ("friction", 5), ("stuck_joint", 3), ("delay", 0)]
SEEDS = (11, 12)


@pytest.fixture(scope="module")
def o5_table(twin):
    """Rows: every fault x seed, plus a fault-free baseline per seed (same seed, A2 in
    NOTES_FOR_DEV round 4), each evaluated with the gate and without it."""
    rows = []
    for seed in SEEDS:
        runs = [("none", -1)] + FAULT_SET
        for fault, joint in runs:
            kw = {} if fault == "none" else dict(fault=fault, fault_joint=joint, fault_onset=6.0)
            run = run_from_frame(*generate(traj="excitation", duration=12.0, seed=seed, **kw))
            res = compute_residual(run, twin, cutoff_hz=DEFAULT_CUTOFF_HZ)
            for gated_on, limit in (("gate", 10.0), ("no gate", 1e9)):
                e = evaluate(run, detect(run, res, force_gain_max=limit))
                rows.append({**e, "kind": fault, "joint": joint, "seed": seed, "gate": gated_on})
    return pd.DataFrame(rows)


def test_o5_report_table(o5_table, capsys):
    """Prints the table the proposal asks for. Always passes; read it with -s."""
    f = o5_table[o5_table.kind != "none"]
    base = o5_table[o5_table.kind == "none"]
    with capsys.disabled():
        print()
        for g in ("gate", "no gate"):
            fg, bg = f[f.gate == g], base[base.gate == g]
            print(f"  O5 [{g:7s}] detection {fg.detected.mean():.0%} of {len(fg)} faults, "
                  f"false positives {bg.false_alarm.mean():.0%} of {len(bg)} healthy runs, "
                  f"median delay {fg.delay_s.median():.3f} s")


def test_o5_every_injected_fault_is_detected_with_the_gate(o5_table):
    f = o5_table[(o5_table.kind != "none") & (o5_table.gate == "gate")]
    assert f.detected.all(), f[~f.detected][["kind", "joint", "seed"]].to_dict("records")


def test_o5_no_false_positives_on_matched_healthy_runs_with_the_gate(o5_table):
    h = o5_table[(o5_table.kind == "none") & (o5_table.gate == "gate")]
    assert not h.false_alarm.any()


def test_o5_gate_never_makes_detection_worse_on_healthy_runs(o5_table):
    """The gate's job is fewer false alarms: healthy-run false positives with the gate must
    not exceed those without it."""
    h = o5_table[o5_table.kind == "none"]
    assert h[h.gate == "gate"].false_alarm.sum() <= h[h.gate == "no gate"].false_alarm.sum()


def test_o5_detection_is_prompt_and_on_the_right_joint(o5_table):
    f = o5_table[(o5_table.kind.isin(["stuck_joint", "friction"])) & (o5_table.gate == "gate")]
    assert (f.delay_s < 2.0).all()
    assert (f.suspected_joint == f.true_joint).mean() >= 0.75
    d = o5_table[(o5_table.kind == "delay") & (o5_table.gate == "gate")]
    assert (d.channel == "tracking").all()                 # the residual cannot see a pure delay
