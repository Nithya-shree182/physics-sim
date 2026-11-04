"""IK and the task trajectories built on it (numpy only, no simulator)."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "workspace"))
import pal_config as cfg  # noqa: E402
import pal_ik  # noqa: E402
import pal_tasks as K  # noqa: E402

U = K._urdf()


def tool(q):
    R, p, _ = U.tool_jacobian(q)
    return R, p


class IK(unittest.TestCase):
    def test_reaches_targets_with_the_gripper_down(self):
        R, _ = pal_ik.tool_target_from_home(U)
        for p in ([0.45, -0.25, 0.025], [0.45, 0.25, 0.22], [0.35, 0.0, 0.40], [0.5, 0.1, 0.1]):
            q, ok, _ = pal_ik.solve(U, np.array(p), R, cfg.HOME_JOINTS)
            self.assertTrue(ok, p)
            Rq, pq = tool(q)
            np.testing.assert_allclose(pq, p, atol=1e-7)
            np.testing.assert_allclose(Rq[:, 2], [0, 0, -1], atol=1e-6)

    def test_respects_joint_limits(self):
        R, _ = pal_ik.tool_target_from_home(U)
        q, ok, _ = pal_ik.solve(U, np.array([0.45, -0.25, 0.025]), R, cfg.HOME_JOINTS)
        self.assertTrue(np.all(q > cfg.Q_MIN) and np.all(q < cfg.Q_MAX))

    def test_unreachable_target_is_reported_not_faked(self):
        R, _ = pal_ik.tool_target_from_home(U)
        q, ok, err = pal_ik.solve(U, np.array([1.5, 0.0, 0.3]), R, cfg.HOME_JOINTS)
        self.assertFalse(ok)
        self.assertGreater(err[0], 0.1)

    def test_jacobian_matches_finite_differences(self):
        q = np.array([0.1, -0.5, 0.2, -2.0, 0.3, 1.5, 0.4])
        J = U.tool_jacobian(q)[2]
        for k in range(7):
            e = np.zeros(7); e[k] = 1e-6
            num = (tool(q + e)[1] - tool(q - e)[1]) / 2e-6
            np.testing.assert_allclose(J[:3, k], num, atol=1e-7)


class PickPlace(unittest.TestCase):
    def test_waypoints_are_where_the_cube_is(self):
        wp = K.waypoints(U)
        np.testing.assert_allclose(tool(wp["pick"])[1], [*K.PICK_XY, K.CUBE / 2], atol=1e-6)
        np.testing.assert_allclose(tool(wp["pre_pick"])[1][:2], K.PICK_XY, atol=1e-6)
        np.testing.assert_allclose(tool(wp["pre_place"])[1][:2], K.PLACE_XY, atol=1e-6)

    def test_no_branch_flip_between_waypoints(self):
        wp = K.waypoints(U)
        self.assertLess(np.abs(wp["pick"] - wp["pre_pick"]).max(), 1.0)       # a vertical move of 0.2 m is a small joint move

    def test_every_style_starts_and_ends_at_home_at_rest(self):
        for style in ("trapezoidal", "s_curve", "torque_aware"):
            t = K.pick_place(style)
            q0, qd0, _ = t.traj(0.0)
            qe, qde, _ = t.traj(t.duration)
            np.testing.assert_allclose(q0, cfg.HOME_JOINTS, atol=1e-9)
            np.testing.assert_allclose(qe, cfg.HOME_JOINTS, atol=1e-6)
            self.assertLess(np.abs(qd0).max() + np.abs(qde).max(), 1e-6, style)

    def test_grasp_and_release_happen_in_order_with_the_arm_at_the_cube(self):
        for style in ("trapezoidal", "s_curve", "torque_aware"):
            t = K.pick_place(style)
            self.assertTrue(0 < t.t_attach < t.t_release < t.duration)
            np.testing.assert_allclose(tool(t.traj(t.t_attach - 1e-3)[0])[1][:2], K.PICK_XY, atol=1e-4)
            np.testing.assert_allclose(tool(t.traj(t.t_release - 1e-3)[0])[1][:2], K.PLACE_XY, atol=1e-4)

    def test_default_speeds_stay_inside_the_trusted_regime(self):
        for style in ("trapezoidal", "s_curve", "torque_aware"):
            t = K.pick_place(style)
            qd = np.array([t.traj(x)[1] for x in np.linspace(0, t.duration, 4000)])
            self.assertLess((np.abs(qd).max(axis=0) / cfg.QD_MAX).max(), 0.42, style)

    def test_styles_differ_in_time_not_in_where_they_go(self):
        a, b = K.pick_place("trapezoidal"), K.pick_place("s_curve")
        self.assertNotAlmostEqual(a.duration, b.duration, places=1)
        np.testing.assert_allclose(a.traj(a.t_attach - 1e-3)[0], b.traj(b.t_attach - 1e-3)[0], atol=1e-4)

    def test_trajectory_is_continuous(self):
        t = K.pick_place("s_curve")
        q = np.array([t.traj(x)[0] for x in np.arange(0, t.duration, 1e-3)])
        self.assertLess(np.abs(np.diff(q, axis=0)).max(), 0.01)               # no jump larger than 0.01 rad per millisecond


class Ellipse(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.e = K.ellipse(duration=6.0)

    def test_the_fingertips_trace_the_ellipse(self):
        # the middle of the traj is the ellipse; check radial consistency of the tool path against the semi-axes
        t0 = self.e.traj.starts[1]
        pts = np.array([tool(self.e.traj(t0 + x)[0])[1] for x in np.linspace(0, 6.0, 600)])
        c = np.array([0.42, 0.0, 0.30])
        r = ((pts[:, 0] - c[0]) / 0.12) ** 2 + ((pts[:, 1] - c[1]) / 0.08) ** 2
        np.testing.assert_allclose(r, 1.0, atol=1e-6)
        np.testing.assert_allclose(pts[:, 2], 0.30, atol=1e-6)

    def test_gripper_stays_vertical(self):
        t0 = self.e.traj.starts[1]
        for x in np.linspace(0, 6.0, 20):
            np.testing.assert_allclose(tool(self.e.traj(t0 + x)[0])[0][:, 2], [0, 0, -1], atol=1e-6)

    def test_starts_and_ends_at_home(self):
        np.testing.assert_allclose(self.e.traj(0.0)[0], cfg.HOME_JOINTS, atol=1e-9)
        np.testing.assert_allclose(self.e.traj(self.e.duration)[0], cfg.HOME_JOINTS, atol=1e-6)


if __name__ == "__main__":
    unittest.main()
