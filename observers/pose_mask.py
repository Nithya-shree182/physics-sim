"""The detector's pose mask: no alarms where the arm is stretched so far that its joint torques say little about a force at the hand.

This is all that is left of the old confidence system (DECISIONS.md D6): no 0-to-1 score, no payload gate, no panel.

At a pose with Jacobian J (6x7, world frame, linear rows first), the angular rows are scaled by a lever arm so every row is in
metres, J_s = diag(1, 1, 1, L, L, L) J. A torque error of 1 N*m can then move the force estimate at the hand by at most
1 / sigma_min(J_s) newtons. The detector raises no alarm where that exceeds FORCE_GAIN_MAX.

Measured (DECISIONS.md D4): the fixed demo spot and the random close cubes stay far below the limit; a cube 0.78 m from the base
(arm stretched) goes above it, and without the mask the detector false-alarms on joint 2 there.
"""

import numpy as np

LENGTH_SCALE_M = 0.472      # dev's measured lever arm at the TCP, home pose
FORCE_GAIN_MAX = 10.0       # [CHOICE] N of force error per N*m of torque error; home is 4.8, the cube at 0.78 m is about 16


def force_noise_gain(J: np.ndarray, length: float = LENGTH_SCALE_M) -> float:
    """Worst-case N of force at the hand per N*m of joint torque error at this pose."""
    s = np.linalg.svd(np.diag([1.0, 1.0, 1.0, length, length, length]) @ J, compute_uv=False)
    return float(1.0 / s[-1])
