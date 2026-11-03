"""Named joint poses (rad), shared by the gravity check and the confidence-score work.

Copied from dev's workspace/pal_config.py (HOME_JOINTS, PROBE_POSES) so both sides test the
same poses. Keep in sync if dev changes them.
"""

import numpy as np

HOME_JOINTS = np.array([0.00, -0.785, 0.00, -2.356, 0.00, 1.571, 0.785])

PROBE_POSES = {
    "home": HOME_JOINTS,
    "near_singular_extended": np.array([0.0, 0.30, 0.0, -0.20, 0.0, 1.00, 0.785]),
    "near_singular_wrist": np.array([0.0, -0.785, 0.0, -2.356, 0.0, 0.00, 0.785]),
    "folded": np.array([0.0, -1.60, 0.0, -2.80, 0.0, 1.40, 0.785]),
    "horizontal_reach": np.array([0.0, 0.00, 0.0, -1.57, 0.0, 1.57, 0.785]),
}

# Joint limits from the datasheet, as used in dev's pal_config.py (Q_MIN, Q_MAX).
Q_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
Q_MAX = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
