"""Checks against dev's Isaac Sim logs in data/real/ (not committed; skipped when absent)."""
import numpy as np
import pytest

from conftest import REPO_ROOT
from payload import estimate_payload
from residual import compute_residual, load_run
import json

import pandas as pd
from validate import load_schema, validate_frame, validate_manifest

REAL = REPO_ROOT / "data" / "real"
RUN_0 = REAL / "slow_reference_0kg_1000hz.parquet"
RUN_1 = REAL / "slow_reference_1p0kg_1000hz.parquet"

pytestmark = pytest.mark.skipif(not (RUN_0.exists() and RUN_1.exists()), reason="data/real/ logs not present")


@pytest.fixture(scope="module")
def results(twin):
    return {k: compute_residual(load_run(p), twin) for k, p in (("0", RUN_0), ("1", RUN_1))}


@pytest.mark.parametrize("path", [RUN_0, RUN_1])
def test_schema_valid(path):
    df = pd.read_parquet(path)
    man = json.loads(path.with_suffix(".json").read_text())
    schema = load_schema()
    assert validate_frame(df, schema)[0] == [] and validate_manifest(man, schema, df) == []


def test_empty_gripper_identity_closes(results):
    """Model matches the simulator: every joint under 2% of its torque limit."""
    r = results["0"].r
    limits = np.array([87, 87, 87, 87, 12, 12, 12.0])
    assert np.all(np.sqrt(np.nanmean(r ** 2, axis=0)) / limits < 0.02)


def test_one_kg_shows_in_wrench_and_payload(twin, results):
    assert results["1"].wrench[-1000:, 2].mean() == pytest.approx(-9.81, abs=1.2)
    m, _ = estimate_payload(results["1"], twin).final
    assert m == pytest.approx(1.0, abs=0.05)


def test_tare_from_empty_run_gives_exact_mass(twin, results):
    m0, _ = estimate_payload(results["0"], twin).final
    m1, _ = estimate_payload(results["1"], twin, tare_kg=m0).final
    assert m1 == pytest.approx(1.0, abs=0.01)
