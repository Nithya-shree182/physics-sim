"""The telemetry writer, end to end, without Isaac Sim.

Isaac is not importable on the host, so the articulation is stood in for by a stub whose
dynamics come from the project's own Pinocchio model. That is enough to prove the parts that
actually go wrong: the column set, the units, the identities between columns, the decimation
bookkeeping, and that a written run validates against the schema. It cannot prove what Isaac
returns — that is what tests/test_ur10e.py's in-simulator run is for.

Why these identities are worth testing. Every one of them is true by construction, which is
exactly why a violation is informative: it means two columns in the same row were computed from
different state. That is the failure mode a per-step writer has, and it is invisible in a plot.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
for sub in ("workspace", "model", "schema"):
    sys.path.insert(0, str(ROOT / sub))

import pal_config as cfg            # noqa: E402
import pal_telemetry as PT          # noqa: E402
from pinocchio_model import load_robot   # noqa: E402

DOF = cfg.DOF


class StubArticulation:
    """The slice of Isaac's Articulation API that TelemetryWriter uses.

    The dynamics are real — they come from Pinocchio on the same URDF — so the identities the
    writer asserts between its columns are being checked against genuine physics and not against
    zeros, which would pass anything.
    """

    def __init__(self, model, extra_joints=0):
        self.m = model
        self.n = DOF + extra_joints
        self.body_names = ["base_link"] + [f"link_{i}" for i in range(DOF)] + [cfg.TOOL_FRAME]
        self.q = np.array(cfg.HOME_JOINTS, dtype=float)
        self.qd = np.zeros(DOF)
        self.tau = np.zeros(DOF)

    def advance(self, dt, step):
        """A smooth, non-trivial motion: every joint moving, none of them at a constant speed."""
        w = 0.6 + 0.1 * np.arange(DOF)
        t = step * dt
        self.q = np.array(cfg.HOME_JOINTS) + 0.15 * np.sin(w * t)
        self.qd = 0.15 * w * np.cos(w * t)
        qdd = -0.15 * w * w * np.sin(w * t)
        self.tau = self.m.tau_model(self.q, self.qd, qdd)

    def _w(self, v):
        out = np.zeros(self.n)
        out[:DOF] = v
        return out.reshape(1, -1)

    def get_joint_positions(self):
        return self._w(self.q)

    def get_joint_velocities(self):
        return self._w(self.qd)

    def get_measured_joint_efforts(self):
        return self._w(self.tau)

    def get_applied_joint_efforts(self):
        return self._w(self.tau)

    def get_generalized_gravity_forces(self):
        return self._w(self.m.g(self.q))

    def get_coriolis_and_centrifugal_forces(self):
        return self._w(self.m.coriolis(self.q, self.qd))

    def get_mass_matrices(self):
        M = np.zeros((self.n, self.n))
        M[:DOF, :DOF] = self.m.M(self.q)
        return M.reshape(1, self.n, self.n)

    def get_jacobians(self):
        J = np.zeros((1, len(self.body_names) - 1, 6, self.n))
        J[0, -1, :, :DOF] = self.m.jacobian(self.q)
        return J


@pytest.fixture(scope="module")
def model():
    return load_robot()


def _capture(tmp_path, model, steps=200, hz=250.0, mj_every=1, extra_joints=0, noise=None):
    art = StubArticulation(model, extra_joints=extra_joints)
    w = PT.TelemetryWriter(art, "stub_run", out_dir=str(tmp_path), mj_every=mj_every,
                           noise=noise, ee_prim=None,
                           labels={"traj_type": "cycle", "drive_mode": "torque",
                                   "payload_kg": 1.5, "payload_attached": True,
                                   "fault_type": "none", "fault_joint": -1,
                                   "fault_param": float("nan")},
                           manifest={"git_commit": "test", "machine": "test",
                                     "isaac_sim_version": "stub", "physics_hz": hz,
                                     "solver": {"type": cfg.SOLVER_TYPE,
                                                "position_iterations": cfg.POSITION_ITERATIONS,
                                                "velocity_iterations": cfg.VELOCITY_ITERATIONS},
                                     "asset": {"usd": cfg.ROBOT_USD_SUBPATH,
                                               "urdf": cfg.URDF_PATH},
                                     "armature": cfg.ARMATURE.tolist(),
                                     "friction_source": cfg.FRICTION_SOURCE,
                                     "plant_friction_scale": cfg.PLANT_FRICTION_SCALE,
                                     "friction_values": cfg.ROBOT.friction_3col().tolist(),
                                     "gravity": [0.0, 0.0, cfg.GRAVITY], "seed": 0})
    dt = 1.0 / hz
    for k in range(steps):
        art.advance(dt, k)
        w.capture(k, (k + 1) * dt, dt, q_cmd=art.q, qd_cmd=art.qd, qdd_cmd=np.zeros(DOF),
                  grip_active=k > 50, box_index=0 if k > 50 else -1, phase="traverse")
    return w


def _frame(w):
    import pandas as pd
    rows = w.rows
    return pd.DataFrame(rows)


# ── the column set ──────────────────────────────────────────────────────────────────────
def test_the_writer_emits_exactly_the_schema_columns(tmp_path, model):
    import validate as V
    w = _capture(tmp_path, model, steps=5)
    required = set(V.required_fields(V.load_schema()))
    got = set(w.rows[0])
    assert not required - got, f"schema columns never written: {sorted(required - got)}"
    assert not got - required, f"columns written that the schema does not define: {sorted(got - required)}"


def test_the_tool_jacobian_row_is_resolved_from_the_body_name_not_guessed(tmp_path, model):
    """A wrong body index is silently wrong: it returns the neighbouring link's Jacobian. The
    writer must find the tool by name, and must still find it when the articulation carries
    joints past the arm's own."""
    art = StubArticulation(model, extra_joints=2)
    w = PT.TelemetryWriter(art, "r", out_dir=str(tmp_path), ee_prim=None)
    assert w.ee_body_index == len(art.body_names) - 2      # body list index minus the fixed base
    assert w.ee_body_name == cfg.TOOL_FRAME


# ── identities between columns ──────────────────────────────────────────────────────────
def test_the_torque_decomposition_adds_up_to_the_applied_torque(tmp_path, model):
    df = _frame(_capture(tmp_path, model))
    parts = ["g", "coriolis", "tau_inertial", "tau_friction", "tau_unexplained"]
    total = sum(df[[f"{p}_{i}" for i in range(DOF)]].to_numpy() for p in parts)
    applied = df[[f"tau_applied_{i}" for i in range(DOF)]].to_numpy()
    assert np.abs(total - applied).max() < 1e-9


def test_unexplained_torque_is_small_when_the_stub_obeys_the_same_model(tmp_path, model):
    """The stub's torque IS rnea plus the sigmoid, so what the row cannot account for is only the
    difference between the logged qdd (a backward difference of qd) and the true one. That is the
    discretisation floor, and seeing it be small is what says the two are wired up consistently."""
    df = _frame(_capture(tmp_path, model, steps=400, hz=1000.0))
    unexp = df[[f"tau_unexplained_{i}" for i in range(DOF)]].to_numpy()[2:]
    worst = np.abs(unexp).max(axis=0) / np.asarray(cfg.TAU_MAX)
    assert worst.max() < 0.01, worst.tolist()


def test_kinetic_energy_matches_the_model_and_is_never_negative(tmp_path, model):
    df = _frame(_capture(tmp_path, model))
    q = df[[f"q_{i}" for i in range(DOF)]].to_numpy()
    qd = df[[f"qd_{i}" for i in range(DOF)]].to_numpy()
    want = np.array([model.kinetic_energy(q[k], qd[k]) for k in range(len(df))])
    assert np.abs(want - df["E_kin"].to_numpy()).max() < 1e-9
    assert df["E_kin"].min() >= 0.0


def test_relative_potential_energy_tracks_the_models_absolute_one(tmp_path, model):
    """E_pot_rel is integrated from dV/dt = g' qd rather than computed from link centres of mass,
    because the articulation does not hand those back cheaply. The integral must therefore follow
    the model's absolute potential, up to the integration error of a first-order sum."""
    w = _capture(tmp_path, model, steps=400, hz=1000.0)
    df = _frame(w)
    q = df[[f"q_{i}" for i in range(DOF)]].to_numpy()
    absolute = np.array([model.potential_energy(q[k]) for k in range(len(df))])
    want = absolute - absolute[0]
    got = df["E_pot_rel"].to_numpy()
    assert abs(got[0]) < 1e-12, "E_pot_rel must start at zero"
    swing = want.max() - want.min()
    assert np.abs(got - want).max() < 0.02 * max(swing, 1e-6), (np.abs(got - want).max(), swing)


def test_power_and_work_are_consistent(tmp_path, model):
    df = _frame(_capture(tmp_path, model))
    pj = df[[f"P_joint_{i}" for i in range(DOF)]].to_numpy()
    assert np.abs(pj.sum(axis=1) - df["P_mech"].to_numpy()).max() < 1e-9
    dt = float(df["dt"].iloc[0])
    assert abs(df["W_mech"].iloc[-1] - df["P_mech"].to_numpy().sum() * dt) < 1e-9


def test_headroom_is_the_torque_over_the_limit(tmp_path, model):
    df = _frame(_capture(tmp_path, model))
    tm = df[[f"tau_measured_{i}" for i in range(DOF)]].to_numpy()
    tf = df[[f"tau_frac_{i}" for i in range(DOF)]].to_numpy()
    assert np.abs(np.abs(tm) / np.asarray(cfg.TAU_MAX) - tf).max() < 1e-12
    assert np.abs(tf.max(axis=1) - df["tau_frac_max"].to_numpy()).max() < 1e-12


def test_conditioning_matches_the_model_and_uses_the_linear_block_only(tmp_path, model):
    """The full 6xn Jacobian mixes m/s rows with rad/s rows, so its singular values have no single
    unit. The columns must come from the linear block, which is a physical quantity."""
    df = _frame(_capture(tmp_path, model, steps=20))
    q = df[[f"q_{i}" for i in range(DOF)]].to_numpy()
    for k in (0, 7, 19):
        sv = model.singular_values(q[k])
        assert abs(df["sigma_min"].iloc[k] - sv.min()) < 1e-10
        assert abs(df["sigma_max"].iloc[k] - sv.max()) < 1e-10
        assert abs(df["manip"].iloc[k] - model.manipulability(q[k])) < 1e-10


def test_tool_velocity_is_the_jacobian_times_the_joint_speeds(tmp_path, model):
    df = _frame(_capture(tmp_path, model, steps=30))
    qd = df[[f"qd_{i}" for i in range(DOF)]].to_numpy()
    for k in (5, 17, 29):
        J = df[[f"J_{r}_{c}" for r in range(6) for c in range(DOF)]].iloc[k].to_numpy().reshape(6, DOF)
        want = J @ qd[k]
        got = np.concatenate([df[[f"ee_vel_{a}" for a in "xyz"]].iloc[k].to_numpy(),
                              df[[f"ee_omega_{a}" for a in "xyz"]].iloc[k].to_numpy()])
        assert np.abs(want - got).max() < 1e-12


# ── decimation ──────────────────────────────────────────────────────────────────────────
def test_decimation_holds_M_and_everything_derived_from_it(tmp_path, model):
    """Holding M but recomputing the energy from a fresh qd would make E_kin inconsistent with the
    M in its own row. Both must be held together."""
    df = _frame(_capture(tmp_path, model, steps=60, mj_every=10))
    assert df["mj_fresh"].sum() == 6
    fresh = df.index[df["mj_fresh"]].tolist()
    assert fresh == [0, 10, 20, 30, 40, 50]
    for a, b in zip(fresh, fresh[1:]):
        block = df.iloc[a:b]
        assert block["M_0_0"].nunique() == 1, "M changed on a non-fresh row"
        assert block["sigma_min"].nunique() == 1, "a J-derived scalar changed on a non-fresh row"
    # E_kin uses the held M with the fresh qd, which is the point of decimation, so it does move.
    assert df["E_kin"].nunique() > 6


# ── the written run ─────────────────────────────────────────────────────────────────────
def test_a_written_run_validates_against_the_schema(tmp_path, model):
    import pandas as pd
    import validate as V
    w = _capture(tmp_path, model, steps=120, hz=250.0, mj_every=10)
    out = w.close()
    assert out is not None and out.exists()

    sys.path.insert(0, str(ROOT / "tools"))
    import npz_to_parquet as NP
    pq = NP.convert(out)
    df = pd.read_parquet(pq)
    schema = V.load_schema()
    errors, _ = V.validate_frame(df, schema)
    manifest = json.loads((tmp_path / "stub_run.json").read_text())
    errors += V.validate_manifest(manifest, schema, df)
    assert not errors, errors


def test_the_manifest_records_the_sensor_model_and_the_conventions(tmp_path, model):
    from pal_sensors import SensorNoise
    w = _capture(tmp_path, model, steps=40, noise=SensorNoise(0, scale=0.0))
    w.close()
    man = json.loads((tmp_path / "stub_run.json").read_text())
    assert man["schema_version"] == 1
    assert man["robot"] == cfg.ROBOT_NAME and man["dof"] == DOF
    assert man["sensor_noise"]["ideal"] is True and man["sensor_noise"]["applied_to"] == []
    assert "timing_convention" in man and "energy_convention" in man
    assert man["tau_max"] == list(np.asarray(cfg.TAU_MAX))
    s = man["summary"]
    assert s["rows"] == 40 and s["energy"]["E_kin_max_J"] > 0
    assert 0 <= s["headroom"]["tau_frac_max"] <= 1
    assert 0 <= s["headroom"]["worst_joint"] < DOF


def test_noise_is_off_by_default_so_a_repeat_is_identical(tmp_path, model):
    """With ideal sensing the plant is deterministic and two runs must agree exactly. This is the
    property that makes an energy or torque channel readable, and it is also the reason a
    false-alarm RATE needs noise turned on — stated so neither fact is a surprise later."""
    a = _frame(_capture(tmp_path, model, steps=50))
    b = _frame(_capture(tmp_path, model, steps=50))
    cols = [f"tau_measured_{i}" for i in range(DOF)] + ["E_kin", "W_mech"]
    assert (a[cols].to_numpy() == b[cols].to_numpy()).all()


def test_switching_noise_on_moves_the_logged_channels_but_not_the_decomposition(tmp_path, model):
    from pal_sensors import SensorNoise
    clean = _frame(_capture(tmp_path, model, steps=50))
    noisy = _frame(_capture(tmp_path, model, steps=50, noise=SensorNoise(1, scale=1.0)))
    tm = [f"tau_measured_{i}" for i in range(DOF)]
    assert not np.allclose(clean[tm].to_numpy(), noisy[tm].to_numpy())
    # g, C and M are a cross-check, not a sensor reading, so noise must leave them alone.
    keep = [f"g_{i}" for i in range(DOF)] + [f"M_{r}_{c}" for r in range(DOF) for c in range(DOF)]
    assert (clean[keep].to_numpy() == noisy[keep].to_numpy()).all()
