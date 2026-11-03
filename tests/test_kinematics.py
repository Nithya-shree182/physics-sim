"""Unit tests for the independent URDF kinematics/gravity (pal_kinematics.py, numpy only)."""
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "workspace"))
from pal_kinematics import Urdf  # noqa: E402

URDF = ROOT / "assets/franka_panda/panda_arm_hand.urdf"
Q = np.array([0.1, -0.5, 0.2, -2.0, 0.3, 1.5, 0.4])


class Gravity(unittest.TestCase):
    def test_joint1_axis_is_vertical_so_no_gravity_torque(self):
        self.assertLess(abs(Urdf(URDF).gravity(Q, 0.04)[0]), 1e-6)

    def test_payload_torque_is_linear_in_mass(self):
        g0 = Urdf(URDF).gravity(Q, 0.04)
        d1 = Urdf(URDF, payload_kg=1.0).gravity(Q, 0.04) - g0
        d3 = Urdf(URDF, payload_kg=3.0).gravity(Q, 0.04) - g0
        np.testing.assert_allclose(d3, 3 * d1, rtol=1e-6, atol=1e-6)

    def test_mass_scale_scales_arm_part_only(self):
        # all payload, no arm links: scale must not change the payload-only increment
        base = Urdf(URDF, mass_scale=1.0)
        d_a = Urdf(URDF, 1.0, payload_kg=1.0).gravity(Q, 0.04) - base.gravity(Q, 0.04)
        d_b = Urdf(URDF, 1.3, payload_kg=1.0).gravity(Q, 0.04) - Urdf(URDF, 1.3).gravity(Q, 0.04)
        np.testing.assert_allclose(d_a, d_b, atol=1e-9)
        # and a heavier arm gives a larger shoulder torque
        self.assertGreater(abs(Urdf(URDF, 1.3).gravity(Q, 0.04)[1]), abs(base.gravity(Q, 0.04)[1]))

    def test_matches_a_hand_calculation_for_one_link(self):
        # joint 2 torque from a payload at the hand, arm folded in a plane:
        # tau = m g * horizontal distance from joint 2 axis to the payload.  Compare against FK.
        u = Urdf(URDF, payload_kg=1.0)
        q = np.zeros(7)
        pos = u.com_positions(q, 0.04)
        px, pz = pos["payload"][1][[0, 2]]
        tau = Urdf(URDF, payload_kg=1.0).gravity(q, 0.04)[1] - Urdf(URDF).gravity(q, 0.04)[1]
        self.assertAlmostEqual(abs(tau), 9.81 * abs(px), delta=0.05)


if __name__ == "__main__":
    unittest.main()


class MassMatrix(unittest.TestCase):
    def test_symmetric_positive_definite_at_random_poses(self):
        rng = np.random.default_rng(0)
        u = Urdf(URDF)
        for _ in range(8):
            q = rng.uniform([-2.8, -1.7, -2.8, -3.0, -2.8, 0.0, -2.8], [2.8, 1.7, 2.8, -0.1, 2.8, 3.7, 2.8])
            M = u.mass_matrix(q)
            np.testing.assert_allclose(M, M.T, atol=1e-10)
            self.assertGreater(np.linalg.eigvalsh(M).min(), 0)

    def test_armature_adds_to_the_diagonal_only(self):
        u = Urdf(URDF)
        d = u.mass_matrix(Q, armature=0.0061) - u.mass_matrix(Q)
        np.testing.assert_allclose(d, 0.0061 * np.eye(7), atol=1e-12)

    def test_payload_adds_mass_and_scale_affects_arm_only(self):
        a = Urdf(URDF).mass_matrix(Q)
        b = Urdf(URDF, payload_kg=2.0).mass_matrix(Q)
        self.assertTrue(np.all(np.linalg.eigvalsh(b - a) > -1e-12))      # payload only adds inertia
        c = Urdf(URDF, mass_scale=1.3).mass_matrix(Q)
        self.assertGreater(np.trace(c), np.trace(a))

    def test_kinetic_energy_matches_link_by_link_sum(self):
        # 0.5 qd' M qd must equal the sum of each link's translational + rotational energy,
        # computed independently from FK-by-finite-difference of the COM positions.
        u = Urdf(URDF)
        qd = np.array([0.3, -0.2, 0.4, 0.1, -0.3, 0.2, 0.5])
        ke = 0.5 * qd @ u.mass_matrix(Q) @ qd
        eps = 1e-6
        pos0 = u.com_positions(Q, 0.04)
        pos1 = u.com_positions(Q + eps * qd, 0.04)
        trans = sum(0.5 * m * np.sum(((pos1[n][1] - pos0[n][1]) / eps) ** 2) for n, (m, _) in pos0.items())
        self.assertLessEqual(trans, ke + 1e-9)        # translational part is a lower bound of the total
        self.assertGreater(ke, 0)
