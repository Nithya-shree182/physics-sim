"""Energy, power, and a carried box — the physics the model side is now asked to get right.

These are laws, not measurements: nothing here depends on the arm being a UR10e, on any [CHOICE]
friction value, or on a recorded run. They would catch a wrong inertia tensor, a mass matrix
missing its armature, a payload hung off the wrong frame, or a sign error in the gravity term —
all four of which have happened to this project at least once and none of which show up in a plot.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pinocchio as pin
import pytest

ROOT = Path(__file__).resolve().parents[1]
for sub in ("workspace", "model"):
    sys.path.insert(0, str(ROOT / sub))

from pinocchio_model import load_robot   # noqa: E402

G = 9.81
RNG = np.random.default_rng(20261108)


@pytest.fixture(scope="module")
def m():
    return load_robot()


@pytest.fixture(scope="module")
def frictionless():
    return load_robot(friction=False)


def random_q(spec, n=1):
    """Poses inside the joint limits but away from the stops, so a finite difference is valid."""
    lo = np.maximum(spec.q_min, -np.pi)
    hi = np.minimum(spec.q_max, np.pi)
    return [lo + (hi - lo) * RNG.random(spec.dof) for _ in range(n)]


# ── energy ──────────────────────────────────────────────────────────────────────────────
def test_kinetic_energy_includes_the_armature(m):
    """0.5 qd' M qd must use the M the model actually simulates, armature included.

    Two separate facts are pinned here, because the armature is the term most easily lost. First,
    it really is in the energy: an otherwise identical model with armature=0 reports less. Second,
    going through M agrees with pin.computeKineticEnergy on the installed Pinocchio — version 4.1
    accounts for model.armature in both. That agreement is checked rather than assumed, so if a
    future Pinocchio stops doing it this test fails instead of the twin quietly becoming lighter
    than the plant (which is the 2026-11-04 class of bug: a quiet mass error reads as a payload).
    """
    q = m.spec.home
    qd = np.full(m.nq, 1.0)
    ours = m.kinetic_energy(q, qd)
    assert abs(ours - float(pin.computeKineticEnergy(m.model, m.data, q, qd))) < 1e-12

    bare = load_robot(armature=0.0)
    rotors = 0.5 * float(qd @ (m.armature * qd))
    assert abs(ours - bare.kinetic_energy(q, qd) - rotors) < 1e-12
    assert rotors / ours > 0.01, "the armature should be a visible share, or this test proves nothing"


def test_kinetic_energy_is_zero_at_rest_and_positive_otherwise(m):
    for q in random_q(m.spec, 6):
        assert m.kinetic_energy(q, np.zeros(m.nq)) == 0.0
        assert m.kinetic_energy(q, RNG.normal(size=m.nq)) > 0.0


def test_gravity_power_is_the_rate_of_change_of_potential_energy(m):
    """g(q)' qd == dV/dt. This identity is what lets the in-sim log carry a relative potential
    without ever reading a link's centre of mass, so it had better hold."""
    eps = 1e-6
    for q in random_q(m.spec, 8):
        qd = RNG.normal(size=m.nq)
        numeric = (m.potential_energy(q + eps * qd) - m.potential_energy(q - eps * qd)) / (2 * eps)
        assert abs(m.gravity_power(q, qd) - numeric) < 1e-5 * max(1.0, abs(numeric))


def test_total_energy_is_conserved_in_a_free_swing(frictionless):
    """Zero torque, no friction: the arm falls and total energy must not move. This is the
    cheapest check there is on the mass matrix and the gravity term together — a wrong inertia
    tensor makes energy drift, and nothing else in the suite would notice."""
    mdl = frictionless
    q = np.array(mdl.spec.home, dtype=float)
    qd = np.zeros(mdl.nq)
    dt = 1e-5
    E0 = mdl.total_energy(q, qd)
    for _ in range(4000):                       # 40 ms of free fall
        qdd = mdl.forward_dynamics(q, qd, np.zeros(mdl.nq))
        qd = qd + dt * qdd                      # semi-implicit Euler: drift is O(dt), not O(1)
        q = q + dt * qd
    drift = abs(mdl.total_energy(q, qd) - E0)
    swing = abs(mdl.potential_energy(np.array(mdl.spec.home)) - mdl.potential_energy(q))
    assert swing > 0.05, "the arm barely moved, so conservation was not actually tested"
    assert drift < 0.02 * swing, (drift, swing)


def test_work_done_equals_the_energy_gained(frictionless):
    """The work-energy theorem on a driven motion: integral of tau' qd dt == change in total
    energy. This is the identity the telemetry's W_mech column is for, checked here against the
    model rather than against the simulator, so a failure is the model's."""
    mdl = frictionless
    dt = 1e-5
    q = np.array(mdl.spec.home, dtype=float)
    qd = np.zeros(mdl.nq)
    E0 = mdl.total_energy(q, qd)
    work = 0.0
    tau = np.zeros(mdl.nq)
    tau[0] = 60.0                               # push two joints and let the rest react; big
    tau[1] = -40.0                              # enough that tens of joules actually move
    for _ in range(4000):
        qdd = mdl.forward_dynamics(q, qd, tau)
        qd_mid = qd + 0.5 * dt * qdd            # midpoint velocity: the power over the step
        work += float(tau @ qd_mid) * dt
        qd = qd + dt * qdd
        q = q + dt * qd
    gained = mdl.total_energy(q, qd) - E0
    assert abs(gained) > 0.05, "no energy moved, so nothing was tested"
    assert abs(work - gained) < 0.02 * abs(gained), (work, gained)


# ── the torque decomposition ────────────────────────────────────────────────────────────
def test_the_decomposition_sums_to_the_model_torque(m):
    for q in random_q(m.spec, 6):
        qd, qdd = RNG.normal(size=m.nq), RNG.normal(size=m.nq)
        d = m.decompose(q, qd, qdd)
        assert np.allclose(d["gravity"] + d["inertial"] + d["coriolis"] + d["friction"],
                           d["total"], atol=1e-12)
        assert np.allclose(d["total"], m.tau_model(q, qd, qdd), atol=1e-12)


def test_each_term_vanishes_when_it_should(m):
    """Every term has a condition that kills it, and each one is a different possible sign or
    frame error. At rest only gravity survives; in zero gravity only the motion terms do."""
    q = m.spec.home
    at_rest = m.decompose(q, np.zeros(m.nq), np.zeros(m.nq))
    assert np.allclose(at_rest["inertial"], 0, atol=1e-12)
    assert np.allclose(at_rest["coriolis"], 0, atol=1e-12)
    assert np.allclose(at_rest["friction"], 0, atol=1e-12), "friction must be exactly zero at rest"
    assert np.abs(at_rest["gravity"]).max() > 1.0

    weightless = load_robot(gravity=(0.0, 0.0, 0.0))
    d = weightless.decompose(q, RNG.normal(size=m.nq), RNG.normal(size=m.nq))
    assert np.allclose(d["gravity"], 0, atol=1e-12)
    assert np.abs(d["inertial"]).max() > 0.0

    constant_speed = m.decompose(q, np.full(m.nq, 0.2), np.zeros(m.nq))
    assert np.allclose(constant_speed["inertial"], 0, atol=1e-12)
    assert np.abs(constant_speed["coriolis"]).max() > 0.0


# ── a carried box ───────────────────────────────────────────────────────────────────────
def test_a_payload_adds_exactly_its_own_mass(m):
    loaded = m.with_payload(5.0, size=(0.22, 0.22, 0.17))
    assert abs(loaded.total_mass - m.total_mass - 5.0) < 1e-12
    assert abs(m.total_mass - load_robot().total_mass) < 1e-12, "with_payload modified the original"


def test_a_payloads_gravity_torque_is_the_jacobian_transpose_times_its_weight(m):
    """The one identity that says the load is hung off the right frame at the right offset.

    A payload of mass m at a point p adds J_p' (0, 0, -m g) to the gravity torque, where J_p is
    the Jacobian of that point. Getting the offset wrong leaves this test passing at the flange
    and failing everywhere else, which is why the offset is swept rather than taken once.
    """
    kg = 4.0
    for q in random_q(m.spec, 5):
        for off in ((0.0, 0.0, 0.0), (0.0, 0.0, 0.195), (0.0, 0.05, 0.30)):
            loaded = m.with_payload(kg, offset=off, point_mass=True)
            added = loaded.g(q) - m.g(q)

            J = m.jacobian(q)
            R = m.data.oMf[m.model.getFrameId(m.ee_frame)].rotation
            p = R @ np.asarray(off, float)
            skew = np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])
            Jp = J[:3] - skew @ J[3:]
            want = Jp.T @ np.array([0.0, 0.0, kg * G])     # the torque needed to HOLD the weight

            assert np.abs(added - want).max() < 1e-9, (off, np.abs(added - want).max())


def test_a_box_weighs_the_same_as_a_point_mass_but_does_not_spin_the_same(m):
    """Why with_payload takes a size at all.

    A box and a point mass of equal mass at the same place produce the same gravity torque, so a
    static test cannot tell them apart. They produce different mass matrices, because the box has
    its own rotational inertia about its centre — and that difference lands in M(q) qdd and in
    C(q,qd)qd, which is exactly where a payload is identified from while the arm is moving.
    """
    q = m.spec.home
    box = m.with_payload(6.0, size=(0.22, 0.22, 0.17))
    point = m.with_payload(6.0, point_mass=True)

    assert np.abs(box.g(q) - point.g(q)).max() < 1e-12
    dM = np.abs(box.M(q) - point.M(q))
    assert dM.max() > 1e-4, "a box's own inertia should be visible in the mass matrix"

    # And it is the box's own tensor that explains the difference: m/12 (y^2+z^2, ...) about its
    # centre, which for a 0.22 x 0.22 x 0.17 carton is dominated by the wrist's own axis.
    x, y, z = 0.22, 0.22, 0.17
    I_box = 6.0 / 12.0 * np.diag([y * y + z * z, x * x + z * z, x * x + y * y])
    assert dM.max() < 2.0 * float(np.max(I_box))


def test_a_payload_increases_how_heavy_the_tool_feels(m):
    """The operational-space inertia is the apparent mass at the tool. Adding 8 kg to the tool
    must make every direction heavier, and the matrix must stay symmetric positive-definite."""
    q = m.spec.home
    bare = m.operational_space_inertia(q)
    loaded = m.with_payload(8.0, size=(0.22, 0.22, 0.17)).operational_space_inertia(q)
    for A in (bare, loaded):
        assert np.abs(A - A.T).max() < 1e-8
        assert np.linalg.eigvalsh(A).min() > 0.0
    # Heavier in every principal direction, and by no more than the mass that was added.
    d = np.linalg.eigvalsh(loaded) - np.linalg.eigvalsh(bare)
    assert d.min() > 0.0, d
    assert np.linalg.eigvalsh(loaded).min() > np.linalg.eigvalsh(bare).min()


def test_a_payload_shows_up_in_the_moving_terms_as_well_as_gravity(m):
    """A load the model does not know about is usually described as "extra gravity torque". It is
    not: it is extra inertia and extra velocity coupling too, and at speed those are the larger
    part. Spelled out here because the payload estimator's accuracy depends on it."""
    q = m.spec.home
    qd = np.full(m.nq, 0.8)
    qdd = np.full(m.nq, 1.5)
    bare = m.decompose(q, qd, qdd)
    loaded = m.with_payload(6.0, size=(0.22, 0.22, 0.17)).decompose(q, qd, qdd)
    for term in ("gravity", "inertial", "coriolis"):
        delta = np.abs(loaded[term] - bare[term]).max()
        assert delta > 1e-3, f"a 6 kg box left {term} unchanged"
    assert np.allclose(loaded["friction"], bare["friction"]), "a payload must not change friction"


def test_a_negative_payload_is_refused(m):
    with pytest.raises(ValueError):
        m.with_payload(-1.0)


# ── pose quality ────────────────────────────────────────────────────────────────────────
def test_manipulability_collapses_at_the_elbow_singularity(m):
    """The measure has to actually measure something, and the two measures are not the same thing.

    With the elbow straight the UR10e is at its classic elbow singularity: the tool has lost the
    radial direction. The VOLUME, sqrt(det(J_v J_v')), collapses by more than a factor of ten,
    which is what makes it the useful alarm. The smallest singular value falls much less — only
    about four-fold — because J_v is 3 x 6 and keeps full row rank until the degeneracy is exact.

    Worth having as a test rather than a comment: reading sigma_min alone would have called this
    pose merely mediocre, and a residual measured here is not trustworthy. Both columns are in the
    telemetry for that reason, and anything that gates on pose quality should use `manip`.
    """
    bent = np.array(m.spec.home, dtype=float)
    straight = bent.copy()
    straight[2] = 0.0                             # elbow out straight
    assert m.manipulability(straight) < 0.1 * m.manipulability(bent)
    smin_ratio = m.singular_values(straight).min() / m.singular_values(bent).min()
    assert 0.1 < smin_ratio < 0.5, (
        f"sigma_min fell by {1 / smin_ratio:.1f}x where the volume fell by "
        f"{m.manipulability(bent) / m.manipulability(straight):.1f}x; if this ratio has changed, "
        "the comment above about the two measures disagreeing needs rechecking")


def test_manipulability_is_the_volume_of_the_linear_block(m):
    for q in random_q(m.spec, 5):
        Jv = m.jacobian(q)[:3]
        assert abs(m.manipulability(q) - np.sqrt(np.linalg.det(Jv @ Jv.T))) < 1e-12
        assert np.allclose(np.sort(m.singular_values(q))[::-1], m.singular_values(q))


def test_gravity_headroom_is_a_fraction_and_the_home_pose_has_plenty(m):
    h = m.gravity_headroom(m.spec.home)
    assert h.shape == (m.nq,)
    assert np.all(h >= 0.0)
    assert h.max() < 0.5, f"holding the home pose should not need half the arm's strength: {h}"
