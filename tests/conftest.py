"""Shared setup: make model/, observers/ and schema/ importable, and provide a twin model."""
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
for sub in ("model", "observers", "schema"):
    sys.path.insert(0, str(REPO_ROOT / sub))


@pytest.fixture(scope="session")
def twin():
    from pinocchio_model import load_panda
    return load_panda()
