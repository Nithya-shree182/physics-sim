"""Physics-based checks on the independent Pinocchio model (proposal section 6 and 7.7).

None of these need the simulator: they test properties the equations must have whatever the
parameters are, so a failure means a modelling bug and not a data problem.
"""
import numpy as np
import pal_robot

DOF = pal_robot.active().dof
import pytest

from conftest import REPO_ROOT
from pinocchio_model import friction_sigmoid, load_robot
from poses import HOME_JOINTS, PROBE_POSES, Q_MAX, Q_MIN
from residual import external_wrench

G = 9.81
RNG = np.random.default_rng(0)


def _random_poses(n=6):
    """Poses inside the joint limits, with a margin."""
    lo, hi = Q_MIN + 0.2, Q_MAX - 0.2
    return [lo + RNG.random(DOF) * (hi - lo) for _ in range(n)]


@pytest.fixture(scope="module")
def rigid():
    """Frictionless twin: conservative dynamics, so energy identities hold exactly."""
    return load_robot(friction=False)


# ── structure of the rigid-body equations ──────────────────────────────────────────────
def test_mass_matrix_symmetric_positive_definite_everywhere(twin):
    for q in _random_poses(20):
        M = twin.M(q)
        assert np.allclose(M, M.T, atol=1e-12)
        assert np.linalg.eigvalsh(M).min() > 0


def test_mdot_minus_2c_is_skew_symmetric(rigid):
    """qd^T (Mdot - 2C) qd = 0: the property that makes C a Coriolis matrix. A wrong sign
    or a missing term in C(q, qd) qd breaks it."""
    eps = 1e-6
    for q in _random_poses():
        qd = RNG.normal(size=DOF)
        Mdot = (rigid.M(q + eps * qd) - rigid.M(q - eps * qd)) / (2 * eps)
        assert qd @ Mdot @ qd == pytest.approx(2 * qd @ rigid.coriolis(q, qd), abs=1e-6)


def test_coriolis_is_quadratic_in_velocity(twin):
    q, qd = _random_poses(1)[0], RNG.normal(size=DOF)
    assert np.allclose(twin.coriolis(q, 3 * qd), 9 * twin.coriolis(q, qd), atol=1e-9)
    assert np.allclose(twin.coriolis(q, np.zeros(DOF)), 0, atol=1e-12)


def test_torque_is_linear_in_acceleration(twin):
    q, qd = _random_poses(1)[0], RNG.normal(size=DOF)
    a, b = RNG.normal(size=DOF), RNG.normal(size=DOF)
    base = twin.tau_model(q, qd, np.zeros(DOF))
    lhs = twin.tau_model(q, qd, a + 2 * b) - base
    rhs = (twin.tau_model(q, qd, a) - base) + 2 * (twin.tau_model(q, qd, b) - base)
    assert np.allclose(lhs, rhs, atol=1e-9)


def test_base_rotation_about_vertical_changes_nothing(twin):
    """Joint 1 is the vertical axis: M and g cannot depend on q1, and g_1 is zero."""
    q = _random_poses(1)[0]
    q2 = q.copy()
    q2[0] += 1.234
    assert np.allclose(twin.g(q), twin.g(q2), atol=1e-10)
    assert np.allclose(twin.M(q), twin.M(q2), atol=1e-10)
    assert twin.g(q)[0] == pytest.approx(0.0, abs=1e-10)


def test_gravity_scales_with_g_and_vanishes_in_zero_gravity():
    half, zero = load_robot(gravity=(0, 0, -G / 2)), load_robot(gravity=(0, 0, 0))
    full = load_robot()
    q = HOME_JOINTS
    assert np.allclose(half.g(q), full.g(q) / 2, atol=1e-10)
    assert np.allclose(zero.g(q), 0, atol=1e-12)


# ── energy ─────────────────────────────────────────────────────────────────────────────
def _rk4(model, q, qd, tau, dt):
    def f(q, qd):
        return qd, model.forward_dynamics(q, qd, tau)
    k1 = f(q, qd)
    k2 = f(q + dt / 2 * k1[0], qd + dt / 2 * k1[1])
    k3 = f(q + dt / 2 * k2[0], qd + dt / 2 * k2[1])
    k4 = f(q + dt * k3[0], qd + dt * k3[1])
    return (q + dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]),
            qd + dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]))


def _energy(model, q, qd):
    return 0.5 * qd @ model.M(q) @ qd + model.potential_energy(q)


def test_energy_is_conserved_when_the_arm_falls_freely(rigid):
    """No torque, no friction: kinetic + potential energy stays constant. Ties M, C and g
    together (a wrong C or g drifts)."""
    q, qd = PROBE_POSES["home"].copy(), np.zeros(DOF)
    e0, dt = _energy(rigid, q, qd), 1e-3
    drift = 0.0
    for _ in range(600):
        q, qd = _rk4(rigid, q, qd, np.zeros(DOF), dt)
        drift = max(drift, abs(_energy(rigid, q, qd) - e0))
    assert drift < 1e-4 * max(1.0, abs(e0))
    assert 0.5 * qd @ rigid.M(q) @ qd > 1.0           # it did move: the test is not vacuous


def test_work_energy_theorem_with_applied_torque(rigid):
    """d(E)/dt = tau . qd: the energy gained equals the work the joints did."""
    q, qd, dt, work = HOME_JOINTS.copy(), np.zeros(DOF), 1e-3, 0.0
    e0 = _energy(rigid, q, qd)
    for k in range(800):
        tau = rigid.g(q) + 3.0 * np.sin(2 * np.pi * 0.7 * k * dt + np.arange(DOF))
        q1, qd1 = _rk4(rigid, q, qd, tau, dt)
        work += 0.5 * (tau @ qd + tau @ qd1) * dt
        q, qd = q1, qd1
    assert _energy(rigid, q, qd) - e0 == pytest.approx(work, rel=2e-3, abs=2e-3)


def test_friction_only_removes_energy(twin):
    """Passive friction: for the Gaz sigmoid, friction power tau_f . qd stays >= -tiny. The
    sigmoid has a small offset (phi3), so allow a few mW."""
    for _ in range(200):
        qd = RNG.normal(scale=1.5, size=DOF)
        assert (twin.friction(qd) * qd).min() > -5e-3
    assert np.allclose(friction_sigmoid(np.zeros(DOF)), 0, atol=1e-3)


def test_forward_and_inverse_dynamics_agree(twin):
    q, qd, tau = _random_poses(1)[0], RNG.normal(size=DOF), RNG.normal(size=DOF)
    qdd = twin.forward_dynamics(q, qd, tau)
    assert np.allclose(twin.tau_model(q, qd, qdd), tau, atol=1e-9)


# ── kinematics, Jacobian, wrench ───────────────────────────────────────────────────────
def test_jacobian_matches_finite_difference_of_forward_kinematics(twin):
    eps = 1e-6
    for q in _random_poses():
        J = twin.jacobian(q)[:3]
        num = np.column_stack([(twin.ee_position(q + eps * e) - twin.ee_position(q - eps * e)) / (2 * eps)
                               for e in np.eye(DOF)])
        assert np.allclose(J, num, atol=1e-6)


def test_jacobian_angular_rows_match_finite_difference_of_orientation(twin):
    import pinocchio as pin
    fid = twin.model.getFrameId(twin.ee_frame)
    eps = 1e-6
    q = _random_poses(1)[0]

    def R(qq):
        twin.ee_pose(qq)
        return twin.data.oMf[fid].rotation.copy()

    J = twin.jacobian(q)[3:]
    for i in range(DOF):
        e = np.eye(DOF)[i]
        omega = pin.log3(R(q).T @ R(q + eps * e)) / eps          # body-frame, then to world
        assert np.allclose(R(q) @ omega, J[:, i], atol=1e-5)


@pytest.mark.parametrize("pose", list(PROBE_POSES))   # the arm's named poses
def test_known_push_is_recovered_from_the_residual(twin, pose):
    """Proposal 7.7 (independent validation) on the maths alone. A force F acting on the
    robot at the hand adds J^T F to the measured torque, so the residual is J^T F and the
    wrench conversion must give -F back in the convention 'acts on the robot'."""
    q = PROBE_POSES[pose]
    J = twin.jacobian(q)
    for F in (np.array([0, 0, -9.81, 0, 0, 0]), np.array([5, -3, 8, 0, 0, 0]),
              np.array([0, 0, 0, 0.4, -0.2, 0.1])):
        r = J.T @ F
        assert np.allclose(-external_wrench(J, r), F, atol=1e-8)


def test_push_recovery_degrades_gracefully_with_torque_noise(twin):
    """With 0.05 N*m of torque noise (the synthetic sensor level) the force error stays
    within the gate: at most 10 N per N*m of torque error at a gated-in pose."""
    q, J = PROBE_POSES["home"], twin.jacobian(PROBE_POSES["home"])
    F = np.array([0, 0, -9.81, 0, 0, 0])
    errs = []
    for _ in range(300):
        r = J.T @ F + RNG.normal(scale=0.05, size=DOF)
        errs.append(np.linalg.norm(-external_wrench(J, r)[:3] - F[:3]))
    assert np.sqrt(np.mean(np.square(errs))) < 10 * 0.05 * np.sqrt(7)


# ── payload physics ────────────────────────────────────────────────────────────────────
def _payload_torque_per_kg(twin, q, offset=None):
    """-dU/dq of a 1 kg point mass `offset` m along the tool frame's z, by finite differences of
    forward kinematics, independent of how the plant or the estimator models the payload.

    `offset` defaults to the arm's own tool_offset, so the number this returns is about the TCP
    the rest of the project uses rather than a figure fixed when the arm was something else."""
    offset = float(twin.spec.tool_offset[2]) if offset is None else float(offset)
    import pinocchio as pin
    fid = twin.model.getFrameId(twin.ee_frame)

    def z(qq):
        twin.ee_pose(qq)
        oMf = twin.data.oMf[fid]
        return (oMf.translation + oMf.rotation @ np.array([0, 0, offset]))[2]

    eps = 1e-6
    return np.array([G * (z(q + eps * e) - z(q - eps * e)) / (2 * eps) for e in np.eye(DOF)])


def test_payload_torque_at_home_matches_a_hand_calculation(twin):
    """The payload torque at home, derived rather than quoted.

    For a point mass on a chain of PARALLEL horizontal joints, the torque each one carries is
    m g times the horizontal distance from its own axis to the load — the whole statics of it.
    This checks the model's finite-difference answer against that distance, joint by joint, so
    the test is a derivation and not a recorded constant. (It used to assert 4.6306 N*m/kg on
    joint 4, which was the Franka Panda's figure at its own fingertip TCP.)

    On the UR10e at the standard ready pose the upper arm stands vertically, which puts the
    shoulder-lift and elbow axes at the same x — so those two joints carry the SAME torque, and
    seeing them equal is correct rather than a copy-paste error.
    """
    import pinocchio as pin
    q = np.array(HOME_JOINTS, dtype=float)
    per_kg = _payload_torque_per_kg(twin, q)

    pin.forwardKinematics(twin.model, twin.data, q)
    pin.updateFramePlacements(twin.model, twin.data)
    fid = twin.model.getFrameId(twin.ee_frame)
    oMf = twin.data.oMf[fid]
    tcp = oMf.translation + oMf.rotation @ np.array([0, 0, float(twin.spec.tool_offset[2])])

    for jid in range(1, twin.model.njoints):
        i = jid - 1
        oMi = twin.data.oMi[jid]
        axis = oMi.rotation @ np.array([0.0, 0.0, 1.0])
        if abs(axis[2]) > 0.99:            # a vertical axis: gravity makes no torque about it
            assert per_kg[i] == pytest.approx(0.0, abs=1e-6)
            continue
        if abs(axis[1]) < 0.99:            # not parallel to y; the simple lever argument needs it
            continue
        lever = tcp[0] - oMi.translation[0]
        assert abs(per_kg[i]) == pytest.approx(G * abs(lever), abs=1e-3), (
            twin.model.names[jid], per_kg[i], G * lever)

    # And the same quantity through Pinocchio's own gravity term, which is the independent path.
    assert np.allclose(twin.with_payload(1.0, point_mass=True).g(q) - twin.g(q), per_kg, atol=1e-9)


def test_payload_torque_equals_jacobian_transpose_times_weight(twin):
    """Same quantity through the Jacobian: tau = J_p^T (0, 0, m g) for a point mass."""
    for q in _random_poses(4):
        twin.ee_pose(q)
        R = twin.data.oMf[twin.model.getFrameId(twin.ee_frame)].rotation
        J = twin.jacobian(q)
        # point velocity = v + omega x p  ->  Jp = Jv - [p]x Jw
        p = R @ np.array([0.0, 0.0, float(twin.spec.tool_offset[2])])
        skew = np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])
        Jp = J[:3] - skew @ J[3:]
        assert np.allclose(Jp.T @ np.array([0, 0, G]), _payload_torque_per_kg(twin, q), atol=1e-5)


def test_payload_torque_is_bounded_by_weight_times_reach(twin):
    """A payload can never load a joint by more than its weight times the distance from the
    joint axis to the payload, which caps what the residual can attribute to it."""
    for q in _random_poses(6):
        per_kg = _payload_torque_per_kg(twin, q)
        reach = np.linalg.norm(twin.ee_position(q)) + 0.1034 + 0.34     # base offset + hand
        assert np.abs(per_kg).max() <= G * reach


# ── what the simulator reports (real log) ──────────────────────────────────────────────
REAL_0 = REPO_ROOT / "data" / "real" / "slow_reference_0kg_1000hz.parquet"


@pytest.mark.skipif(not REAL_0.exists(), reason="data/real/ logs not present")
def test_simulator_mass_matrix_does_not_include_armature():
    """Lead from setup.md section 6, now checked. The simulator's logged M(q) matches our M
    built WITHOUT armature on the wrist joints (J7 diagonal differs by 0.0000), and differs
    from ours WITH armature by the armature itself (0.0061). Armature is still in the plant's
    dynamics (the identity only closes with it), so the residual is unaffected; but anything
    that uses the LOGGED M (--jacobian logged, a live M) must add 0.0061 to the diagonal."""
    import pandas as pd
    df = pd.read_parquet(REAL_0)
    q = df[[f"q_{j}" for j in range(DOF)]].to_numpy()
    with_arm, without = load_robot(), load_robot(armature=0.0)
    d_with, d_without = [], []
    for i in range(0, len(df), 500):
        Ml = np.array([[df[f"M_{a}_{b}"].iloc[i] for b in range(DOF)] for a in range(DOF)])
        d_with.append(np.diag(Ml) - np.diag(with_arm.M(q[i])))
        d_without.append(np.diag(Ml) - np.diag(without.M(q[i])))
    j7_with, j7_without = np.mean(d_with, axis=0)[6], np.mean(d_without, axis=0)[6]
    assert j7_without == pytest.approx(0.0, abs=5e-4)
    assert j7_with == pytest.approx(-0.0061, abs=5e-4)
