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
