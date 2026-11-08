"""Shared setup: make model/, observers/, schema/ and workspace/ importable, and provide a twin.

Everything in this directory runs against the arm the project actually has (a UR10e) except four
modules, which assert against RECORDED PANDA RUNS in `out/` and `data/`. Those cannot be ported —
every assertion in them is a measurement of a plant that no longer exists — so they are skipped
until the suites are regenerated on the new arm. See docs/TODO.md item 1.

HISTORY, because the shape of this file is the record of a piece of work. Right after the UR10e
port it also carried a `PANDA_ASSUMPTION` dict: fifteen individually named tests, each with a
reason, that assumed something about the old arm beyond a fixture swap — a Panda torque figure, the
Panda's 0.855 m reach as a bound, a single-cube finger pick, a fingertip ellipse, a tolerance set
against a smaller arm's torque scale. Listing them one by one rather than skipping whole files was
what made them cheap to work through afterwards. All fifteen were ported on 2026-11-08 and the dict
is gone; what is left is the one category that genuinely cannot be fixed without GPU time.

`PAL_ROBOT` is set here because `pal_config` reads it once at import time, and a conftest loads
before the test modules beside it.
"""
import os
import sys
from pathlib import Path

os.environ.setdefault("PAL_ROBOT", "ur10e")

import pytest  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[1]
for sub in ("model", "observers", "schema", "workspace"):   # workspace holds pal_robot
    sys.path.insert(0, str(REPO_ROOT / sub))


# Recorded-evidence suites: every assertion in these is a measurement of the Franka Panda plant, so
# there is nothing to port — they need UR10e runs captured first (tools/pal_suite.sh on a GPU).
# Until that happens the project has no validated detection or payload result for the arm it runs,
# which is the single biggest gap in it. Skipping them loudly, with the reason in every line of
# `pytest -rs` output, is the point: a green run must not be able to imply otherwise.
PANDA_EVIDENCE = {
    "test_baseline.py": "asserts against recorded Panda fault runs in out/",
    "test_proposal_objectives.py": "scores the proposal table on Panda data and synthetic Panda logs",
    "test_real_faults.py": "regression-locks findings measured on the Panda plant",
    "test_real_data.py": "reads data/ref_matched_* , which are Panda reference runs",
}


def pytest_collection_modifyitems(config, items):
    ev = pytest.mark.skip(reason="Panda evidence; awaiting regenerated UR10e runs (docs/TODO.md item 1)")
    for item in items:
        if Path(str(item.fspath)).name in PANDA_EVIDENCE:
            item.add_marker(ev)


@pytest.fixture(scope="session")
def twin():
    from pinocchio_model import load_robot
    return load_robot()
