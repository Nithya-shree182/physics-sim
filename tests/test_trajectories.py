"""Unit tests for workspace/pal_trajectories.py (numpy only). Run: python3 -m unittest discover tests"""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "workspace"))
import pal_config as cfg  # noqa: E402
import pal_trajectories as T  # noqa: E402

Q0 = cfg.HOME_JOINTS
Q1 = cfg.HOME_JOINTS + np.array([0.6, -0.3, 0.3, 0.4, 0.3, -0.4, 0.3])


def sample(tr, n=4000):
    ts = np.linspace(0, tr.duration, n)
    out = [tr(t) for t in ts]
    return ts, np.array([o[0] for o in out]), np.array([o[1] for o in out]), np.array([o[2] for o in out])


class Cycle(unittest.TestCase):
    def kinds(self):
        return [T.cycle(Q0, Q1, "trapezoidal"), T.cycle(Q0, Q1, "s_curve"),
                T.cycle(Q0, Q1, "torque_aware", T=1.5)]

    def test_returns_to_start_and_reaches_goal(self):
        for tr in self.kinds():
            ts, q, qd, qdd = sample(tr)
            np.testing.assert_allclose(q[0], Q0, atol=1e-9)
            np.testing.assert_allclose(q[-1], Q0, atol=1e-6)
            far = np.abs(q - Q1).max(axis=1).min()
            self.assertLess(far, 1e-3, "never reaches the far waypoint")

    def test_rest_to_rest(self):
        for tr in self.kinds():
            _, _, qd, _ = sample(tr)
            self.assertLess(np.abs(qd[0]).max(), 1e-9)
            self.assertLess(np.abs(qd[-1]).max(), 1e-6)

    def test_velocity_limits_respected(self):
        for tr in self.kinds()[:2]:
            _, _, qd, _ = sample(tr)
            self.assertTrue((np.abs(qd).max(axis=0) <= cfg.QD_MAX * 1.001).all())

    def test_acceleration_limits_respected(self):
        for tr in self.kinds()[:2]:
            _, _, _, qdd = sample(tr)
            self.assertTrue((np.abs(qdd).max(axis=0) <= T.A_MAX * 1.01).all())

    def test_scurve_is_jerk_limited_and_trapezoid_is_not(self):
        _, _, _, a_s = sample(T.cycle(Q0, Q1, "s_curve"), 20000)
        ts = np.linspace(0, T.cycle(Q0, Q1, "s_curve").duration, 20000)
        jerk_s = np.abs(np.diff(a_s, axis=0) / np.diff(ts)[:, None]).max()
        _, _, _, a_t = sample(T.cycle(Q0, Q1, "trapezoidal"), 20000)
        tt = np.linspace(0, T.cycle(Q0, Q1, "trapezoidal").duration, 20000)
        jerk_t = np.abs(np.diff(a_t, axis=0) / np.diff(tt)[:, None]).max()
        self.assertLessEqual(jerk_s, 1.01 * T.J_MAX.max())   # the contract: joint jerk <= J_MAX
        self.assertGreater(jerk_t, 5 * jerk_s)

    def test_scurve_slower_than_trapezoid(self):
        self.assertGreater(T.cycle(Q0, Q1, "s_curve").duration,
                           T.cycle(Q0, Q1, "trapezoidal").duration)

    def test_derivatives_consistent(self):
        tr = T.cycle(Q0, Q1, "torque_aware", T=1.5)
        ts, q, qd, qdd = sample(tr, 8000)
        dt = ts[1] - ts[0]
        np.testing.assert_allclose(np.diff(q, axis=0) / dt, qd[:-1], atol=2e-2)
        np.testing.assert_allclose(np.diff(qd, axis=0) / dt, qdd[:-1], atol=2.0)


class Reference(unittest.TestCase):
    def test_multisine_inside_velocity_limit_fraction(self):
        tr = T.multisine(0.20)
        ts = np.linspace(0, 20, 20000)
        qd = np.array([tr(t)[1] for t in ts])
        self.assertTrue((np.abs(qd).max(axis=0) <= 0.20 * cfg.QD_MAX + 1e-9).all())

    def test_multisine_starts_at_home_with_matching_derivatives(self):
        tr = T.multisine(0.20)
        np.testing.assert_allclose(tr(0.0)[0], cfg.HOME_JOINTS, atol=1e-12)
        dt = 1e-6
        qd_num = (tr(1.0 + dt)[0] - tr(1.0 - dt)[0]) / (2 * dt)
        np.testing.assert_allclose(qd_num, tr(1.0)[1], atol=1e-6)

    def test_hold(self):
        q, qd, qdd = T.hold(Q0)(3.0)
        np.testing.assert_allclose(q, Q0); self.assertEqual(np.abs(qd).max(), 0.0)


if __name__ == "__main__":
    unittest.main()
