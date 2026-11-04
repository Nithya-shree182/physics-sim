import pytest

from payload import estimate_payload
from residual import DEFAULT_CUTOFF_HZ, compute_residual, run_from_frame
from synth_log import generate


def _estimate(twin, **kw):
    run = run_from_frame(*generate(**kw))
    return estimate_payload(compute_residual(run, twin, cutoff_hz=DEFAULT_CUTOFF_HZ), twin)


@pytest.mark.parametrize("mass", [0.5, 1.0, 3.0])
def test_exact_mass_at_rest_without_mismatch(twin, mass):
    m, _ = _estimate(twin, traj="hold", duration=2.0, payload_kg=mass, mismatch=False, noise=False).final
    assert m == pytest.approx(mass, rel=0.002)


def test_mass_during_fast_motion_without_mismatch(twin):
    """The acceleration term keeps the estimate unbiased while the arm moves."""
    m, _ = _estimate(twin, traj="excitation", duration=6.0, payload_kg=1.0, mismatch=False, noise=False).final
    assert m == pytest.approx(1.0, rel=0.01)


def test_empty_arm_reads_near_zero_with_mismatch(twin):
    m, _ = _estimate(twin, traj="hold", duration=2.0, payload_kg=0.0).final
    assert abs(m) < 0.05          # the +-3% link-mass mismatch shows up as ~25 g of phantom mass


def test_tare_removes_the_phantom_mass(twin):
    empty, _ = _estimate(twin, traj="hold", duration=2.0, payload_kg=0.0, seed=5).final
    run = run_from_frame(*generate("hold", 2.0, payload_kg=1.0, seed=5))
    est = estimate_payload(compute_residual(run, twin, cutoff_hz=DEFAULT_CUTOFF_HZ), twin, tare_kg=empty)
    assert est.final[0] == pytest.approx(1.0, abs=0.01)
