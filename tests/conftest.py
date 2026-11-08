"""Shared setup: make model/, observers/ and schema/ importable, and provide a twin model.

The Franka Panda was dropped on 2026-11-08 and the arm is now a UR10e. Most of this directory
tests shared logic (residual, detector, payload, trajectories, IK, schema) and runs against
whichever arm is loaded. Four modules do not: they assert against RECORDED PANDA RUNS in `out/`
and `data/`, which no UR10e data exists to replace yet, so they are skipped until the suites are
regenerated on the new arm. See docs/UR10E_PORT.md.

`PAL_ROBOT` is set here because `pal_config` reads it once at import time, and a conftest loads
before the test modules beside it.
"""
import os
import sys
from pathlib import Path

os.environ.setdefault("PAL_ROBOT", "ur10e")

import pytest  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[1]
for sub in ("model", "observers", "schema"):
    sys.path.insert(0, str(REPO_ROOT / sub))


# Recorded-evidence suites: every assertion in these is a measurement of the Franka Panda plant,
# so there is nothing to port — they need UR10e runs captured first (docs/UR10E_PORT.md).
PANDA_EVIDENCE = {
    "test_baseline.py": "asserts against recorded Panda fault runs in out/",
    "test_proposal_objectives.py": "scores the proposal table on Panda data and synthetic Panda logs",
    "test_real_faults.py": "regression-locks findings measured on the Panda plant",
    "test_real_data.py": "reads data/ref_matched_* , which are Panda reference runs",
}


def pytest_collection_modifyitems(config, items):
    skip = pytest.mark.skip(reason="Panda evidence; awaiting regenerated UR10e runs (docs/UR10E_PORT.md)")
    for item in items:
        why = PANDA_EVIDENCE.get(Path(str(item.fspath)).name)
        if why:
            item.add_marker(skip)


@pytest.fixture(scope="session")
def twin():
    from pinocchio_model import load_robot
    return load_robot()
