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
    """With no payload the estimate must sit near zero, but not AT zero: the synthetic plant's
    link masses differ from the twin's by +-3% (synth_log.MASS_MISMATCH), and that error has to
    land somewhere. It lands in the estimate as a phantom mass.

    The tolerance is a fraction of the arm's own moving mass rather than an absolute number of
    kilograms, because that is what the quantity actually scales with: 3% of a 29 kg arm, seen
    through the Jacobian at one pose, is naturally larger than 3% of an 18 kg one. The previous
    absolute 50 g bound was 0.3% of the Panda's mass and failed here at 0.24% of this arm's —
    tighter in real terms, which is how an un-scaled tolerance misleads."""
    m, _ = _estimate(twin, traj="hold", duration=2.0, payload_kg=0.0).final
    bound = 0.004 * twin.total_mass          # 0.4% of the arm's moving mass
    assert abs(m) < bound, f"phantom mass {m:.4f} kg exceeds {bound:.4f} kg"


def test_tare_removes_the_phantom_mass(twin):
    empty, _ = _estimate(twin, traj="hold", duration=2.0, payload_kg=0.0, seed=5).final
    run = run_from_frame(*generate("hold", 2.0, payload_kg=1.0, seed=5))
    est = estimate_payload(compute_residual(run, twin, cutoff_hz=DEFAULT_CUTOFF_HZ), twin, tare_kg=empty)
    assert est.final[0] == pytest.approx(1.0, abs=0.01)
