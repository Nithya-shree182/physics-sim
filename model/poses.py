"""Named joint poses (rad), shared by the gravity check and the analysis side.

Everything here follows the active robot (pal_robot). The UR10e has only a home pose: the others
were Franka Panda configurations chosen for the confidence score that D6 removed, and the Panda
itself went on 2026-11-08.
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "workspace"))
import pal_robot  # noqa: E402

_ACTIVE = pal_robot.active()

HOME_JOINTS = _ACTIVE.home
PROBE_POSES = {"home": _ACTIVE.home}

# Joint limits, from the robot spec (datasheet values).
Q_MIN, Q_MAX = _ACTIVE.q_min, _ACTIVE.q_max
