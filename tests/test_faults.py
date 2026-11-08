"""Unit tests for the fault injectors and sensor noise (no simulator needed)."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "workspace"))
from pal_faults import FaultInjector, FaultSpec  # noqa: E402
import pal_config as cfg  # noqa: E402
from pal_sensors import REFERENCE_SIGMA, SensorNoise  # noqa: E402


class StubPlant:
    base_friction = np.tile(np.array([0.2, 0.17, 0.05]), (7, 1))

    def __init__(self):
        self.written = []

    def set_friction(self, f):
        self.written.append(np.array(f).copy())


class Spec(unittest.TestCase):
    def test_schema_rule_joint_minus_one_only_for_none(self):
        FaultSpec("none")
        with self.assertRaises(ValueError):
            FaultSpec("delay", -1, 0.01)
        with self.assertRaises(ValueError):
            FaultSpec("friction", 9, 2.0)
        with self.assertRaises(ValueError):
            FaultSpec("attack", 0, 1.0)            # no attack type exists: out of scope by decision

    def test_tags(self):
        self.assertEqual(FaultSpec("none").tag, "healthy")
        self.assertEqual(FaultSpec("friction", 3, 4.0).tag, "friction_j3_4")
        self.assertEqual(FaultSpec("delay", 0, 0.025).tag, "delay_all_25ms")


class Friction(unittest.TestCase):
    def test_only_the_named_joint_changes_and_clear_restores(self):
        p = StubPlant(); inj = FaultInjector(p, 1000)
        inj.set_spec(FaultSpec("friction", 3, 4.0, onset_s=1.0))
        self.assertFalse(inj.update(0.5)); self.assertEqual(p.written, [])
        self.assertTrue(inj.update(1.0))
        f = p.written[-1]
        np.testing.assert_allclose(f[3], 4.0 * p.base_friction[3])
        np.testing.assert_allclose(np.delete(f, 3, axis=0), np.delete(p.base_friction, 3, axis=0))
        inj.clear()
        np.testing.assert_allclose(p.written[-1], p.base_friction)

    def test_stuck_joint_raises_static_and_dynamic_only(self):
        p = StubPlant(); inj = FaultInjector(p, 1000)
        inj.set_spec(FaultSpec("stuck_joint", 3, 200.0, onset_s=0.0)); inj.update(0.0)
        np.testing.assert_allclose(p.written[-1][3], [200.0, 200.0, 0.05])

    def test_applied_once(self):
        p = StubPlant(); inj = FaultInjector(p, 1000)
        inj.set_spec(FaultSpec("friction", 1, 2.0, onset_s=0.0))
        for k in range(5):
            inj.update(k * 1e-3)
        self.assertEqual(len(p.written), 1)


class Delay(unittest.TestCase):
    def test_exact_delay_in_steps(self):
        inj = FaultInjector(StubPlant(), 1000)
        inj.set_spec(FaultSpec("delay", 0, 0.003, onset_s=0.002))
        out = []
        for k in range(10):
            inj.update(k * 1e-3)
            out.append(inj.command(np.full(7, float(k)), np.zeros(7))[0])
        # active from step 2; command k appears at step k+3; before that the last healthy one repeats
        self.assertEqual(out[:2], [0.0, 1.0])
        self.assertEqual(out[2:5], [1.0, 1.0, 1.0])
        self.assertEqual(out[5:], [2.0, 3.0, 4.0, 5.0, 6.0])


class Backlash(unittest.TestCase):
    def test_gap_blocks_torque_then_engages(self):
        inj = FaultInjector(StubPlant(), 1000)
        inj.set_spec(FaultSpec("backlash", 3, 0.01, onset_s=0.0)); inj.update(0.0)
        tau = np.zeros(7); tau[3] = 5.0
        seq = [inj.command(tau, np.zeros(7))[3] for _ in range(30)]
        first = next(k for k, v in enumerate(seq) if v != 0.0)
        self.assertGreater(first, 0)                        # play: nothing for the first few steps
        self.assertTrue(all(v == 5.0 for v in seq[first:]))
        self.assertTrue(all(v == 0.0 for v in seq[:first]))

    def test_other_joints_untouched(self):
        inj = FaultInjector(StubPlant(), 1000)
        inj.set_spec(FaultSpec("backlash", 3, 0.01, onset_s=0.0)); inj.update(0.0)
        tau = np.arange(7, dtype=float) + 1
        out = inj.command(tau, np.zeros(7))
        np.testing.assert_allclose(np.delete(out, 3), np.delete(tau, 3))

    def test_wider_play_takes_longer(self):
        def steps(width):
            inj = FaultInjector(StubPlant(), 1000)
            inj.set_spec(FaultSpec("backlash", 3, width, onset_s=0.0)); inj.update(0.0)
            tau = np.zeros(7); tau[3] = 5.0
            return next(k for k in range(200) if inj.command(tau, np.zeros(7))[3] != 0.0)
        self.assertGreater(steps(0.04), steps(0.01))


class Noise(unittest.TestCase):
    """The sensor model. Noise is OFF by default since 2026-11-08, so every test that wants it
    asks for it explicitly with scale=1.0; `test_the_default_is_ideal` pins the default itself."""

    def row(self):
        r = {}
        for i in range(cfg.DOF):
            r.update({f"q_{i}": 0.0, f"qd_{i}": 0.0,
                      f"tau_measured_{i}": 0.0, f"tau_applied_{i}": 0.0})
        return r

    def test_the_default_is_ideal_sensing(self):
        n = SensorNoise(5)
        self.assertTrue(n.ideal)
        self.assertFalse(bool(n))
        r = self.row(); n.apply(r)
        self.assertEqual(r, self.row())
        self.assertEqual(n.manifest()["applied_to"], [])

    def test_same_seed_same_noise_different_seed_differs(self):
        a, b, c = self.row(), self.row(), self.row()
        SensorNoise(5, scale=1.0).apply(a)
        SensorNoise(5, scale=1.0).apply(b)
        SensorNoise(6, scale=1.0).apply(c)
        self.assertEqual(a, b); self.assertNotEqual(a, c)

    def test_sigma_is_what_was_asked(self):
        n = SensorNoise(1, scale=1.0)
        vals = []
        for _ in range(4000):
            r = self.row(); n.apply(r); vals.append(r["tau_applied_0"])
        want = REFERENCE_SIGMA["tau"]
        self.assertAlmostEqual(float(np.std(vals)), want, delta=0.1 * want)

    def test_scaling_scales_the_sigma(self):
        n = SensorNoise(3, scale=0.25)
        vals = []
        for _ in range(4000):
            r = self.row(); n.apply(r); vals.append(r["tau_measured_0"])
        want = 0.25 * REFERENCE_SIGMA["tau"]
        self.assertAlmostEqual(float(np.std(vals)), want, delta=0.1 * want)

    def test_unrelated_columns_untouched(self):
        """g, C, M and J are the simulator's own cross-check, not sensor readings, so noise must
        never reach them however it is configured."""
        r = self.row(); r["g_0"] = 1.234
        SensorNoise(2, scale=1.0).apply(r); self.assertEqual(r["g_0"], 1.234)

    def test_a_row_missing_a_field_is_not_an_error(self):
        """The live server passes a partial row (no commands, no M). Noise must take what is
        there rather than raising on the first absent column."""
        r = {f"tau_measured_{i}": 0.0 for i in range(cfg.DOF)}
        SensorNoise(4, scale=1.0).apply(r)
        self.assertTrue(all(v != 0.0 for v in r.values()))


if __name__ == "__main__":
    unittest.main()
