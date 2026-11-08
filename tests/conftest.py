"""Shared setup: make model/, observers/ and schema/ importable, and provide a twin model.

Every test in this directory was written for the Franka Panda and cites Panda numbers, so the arm
is pinned here rather than left to follow `pal_robot.DEFAULT_ROBOT` (which became the UR10e on
2026-11-08). That keeps the whole suite as the regression net it was built to be: if a change to
the shared code alters a Panda result, these tests still catch it. The UR10e has its own tests,
which set `PAL_ROBOT` themselves.

This must run before anything imports `pal_config`, because that module reads the variable once at
import time. A conftest is loaded before the test modules it sits beside, so this is that place.
"""
import os
import sys
from pathlib import Path

os.environ.setdefault("PAL_ROBOT", "panda")

import pytest  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[1]
for sub in ("model", "observers", "schema"):
    sys.path.insert(0, str(REPO_ROOT / sub))


@pytest.fixture(scope="session")
def twin():
    from pinocchio_model import load_panda
    return load_panda()
