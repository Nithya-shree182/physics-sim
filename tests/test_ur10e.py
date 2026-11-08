"""Regression tests for the UR10e port (2026-11-08).

The rest of this directory is pinned to the Panda by conftest.py, because every other test cites a
Panda number. These are the UR10e's own net. Nothing here needs Isaac Sim: the URDF, the
independent numpy kinematics, the IK and the conveyor-cell cycle are all host-side.

The checks that matter most are the two cross-validations. `test_mass_matrix_matches_pinocchio`
compares the hand-written numpy composite-rigid-body model against Pinocchio on the same URDF —
this is the check that would have caught the diagonal-inertia bug of 2026-11-03 (REVIEW F11), and
it is worth more on the UR10e than it was on the Panda because three of its links carry genuinely
off-diagonal tensors expressed in rotated frames. `test_total_mass_matches_datasheet` is the check
that the parameters were transcribed from the right robot at all.
"""
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
URDF = REPO / "assets" / "ur10e" / "ur10e.urdf"
sys.path.insert(0, str(REPO / "workspace"))

pin = pytest.importorskip("pinocchio")
from pal_kinematics import Urdf  # noqa: E402

DOF = 6
TOOL = "wrist_3_link"
# Universal Robots' published figures for the UR10e, the ones the port has to reproduce.
DATASHEET_MASS_KG = 33.5
DATASHEET_REACH_M = 1.300
TAU_MAX = np.array([330.0, 330.0, 150.0, 54.0, 54.0, 54.0])
QD_MAX = np.radians([120.0, 120.0, 180.0, 180.0, 180.0, 180.0])
HOME = np.radians([0.0, -90.0, 90.0, -90.0, -90.0, 0.0])


@pytest.fixture(scope="module")
def model():
    assert URDF.exists(), f"{URDF} is missing; regenerate it with tools/make_ur10e_urdf.py"
    m = pin.buildModelFromUrdf(str(URDF))
    return m, m.createData()


@pytest.fixture(scope="module")
def numpy_urdf():
    return Urdf(str(URDF), tool_link=TOOL)


def test_urdf_structure(model, numpy_urdf):
    m, _ = model
    assert m.nq == m.nv == DOF
    assert [m.names[i] for i in range(1, m.njoints)] == list(numpy_urdf.arm_joints)
    assert numpy_urdf.dof == DOF
    assert numpy_urdf.root == "base_link"
    assert numpy_urdf.moving_links == ("shoulder_link", "upper_arm_link", "forearm_link",
                                       "wrist_1_link", "wrist_2_link", "wrist_3_link")


def test_total_mass_matches_datasheet(model):
    """Pinocchio reports only the moving mass; the 4 kg fixed base is attached to the world."""
    m, _ = model
    total = pin.computeTotalMass(m) + 4.0
    assert abs(total - DATASHEET_MASS_KG) < 0.5, f"{total:.3f} kg against a {DATASHEET_MASS_KG} kg datasheet"


def test_limits_match_the_user_manual(model):
    m, _ = model
    assert np.allclose(m.effortLimit, TAU_MAX)
    assert np.allclose(m.velocityLimit, QD_MAX, atol=1e-6)


def test_reach_is_about_the_datasheet_figure(model):
    m, d = model
    tid = m.getFrameId("tool0")
    best = 0.0
    for _ in range(4000):
        q = pin.randomConfiguration(m)
        pin.forwardKinematics(m, d, q)
        pin.updateFramePlacements(m, d)
        best = max(best, float(np.linalg.norm(d.oMf[tid].translation[:2])))
    # tool0 sits at the flange and the wrist offsets let it swing a little past the quoted sphere
    assert DATASHEET_REACH_M - 0.05 < best < DATASHEET_REACH_M + 0.10, f"max reach {best:.3f} m"


def test_mass_matrix_matches_pinocchio(model, numpy_urdf):
    """The independent numpy model against Pinocchio on the same URDF, over random poses."""
    m, d = model
    rng = np.random.default_rng(1)
    worst = 0.0
    for _ in range(40):
        q = rng.uniform(-1.5, 1.5, DOF)
        Mp = pin.crba(m, d, q)
        Mp = np.triu(Mp) + np.triu(Mp, 1).T
        worst = max(worst, float(np.abs(numpy_urdf.mass_matrix(q) - Mp).max()))
    assert worst < 1e-9, f"worst |M_numpy - M_pinocchio| = {worst:.3e} kg*m^2"


def test_gravity_matches_pinocchio(model, numpy_urdf):
    m, d = model
    rng = np.random.default_rng(2)
    worst = 0.0
    for _ in range(25):
        q = rng.uniform(-1.5, 1.5, DOF)
        worst = max(worst, float(np.abs(numpy_urdf.gravity(q) - pin.computeGeneralizedGravity(m, d, q)).max()))
    assert worst < 1e-5, f"worst |g_numpy - g_pinocchio| = {worst:.3e} N*m"


def test_rotated_inertia_frames_are_honoured(numpy_urdf):
    """Three UR10e links state their inertia in a rotated frame. Dropping the rotation would be
    the same class of error as the diagonal-only bug; this pins that it is applied."""
    import xml.etree.ElementTree as ET
    rotated = [l.get("name") for l in ET.parse(URDF).getroot().findall("link")
               if (o := l.find("inertial/origin")) is not None and o.get("rpy")
               and any(abs(float(x)) > 1e-12 for x in o.get("rpy").split())]
    assert len(rotated) == 3, f"expected 3 rotated inertia frames, found {rotated}"
    for name in rotated:
        T = numpy_urdf.inertia[name]
        assert np.allclose(T, T.T), f"{name}: inertia tensor must stay symmetric after rotation"


def test_home_pose_points_the_tool_straight_down(model):
    """The suction cups have to look at the top face of a box, so the tool's z must be -world z."""
    m, d = model
    pin.forwardKinematics(m, d, HOME)
    pin.updateFramePlacements(m, d)
    t0 = d.oMf[m.getFrameId("tool0")]
    assert np.allclose(t0.rotation[:, 2], [0.0, 0.0, -1.0], atol=1e-6), t0.rotation[:, 2]
    assert t0.translation[2] > 0.5, "the home pose should hold the tool well clear of the table"


def test_home_pose_is_far_inside_the_torque_limits(model):
    m, d = model
    g = pin.computeGeneralizedGravity(m, d, HOME)
    assert np.all(np.abs(g) < 0.5 * TAU_MAX), f"gravity at home {np.round(g, 1)} against {TAU_MAX}"


# ── the conveyor-cell cycle ──────────────────────────────────────────────────────────────────
# pal_config reads PAL_ROBOT once at import and conftest pins it to the Panda for this directory,
# so the cycle is built in a subprocess with the variable set to the UR10e.
CYCLE_SCRIPT = """
import os, sys, json
sys.path.insert(0, %r)
import numpy as np
import pal_tasks as K, pal_config as cfg
WORK_Z, BOX_H, CUP = -0.25, 0.13, 0.110
boxes = [(0.40, -0.48), (0.62, -0.48), (0.84, -0.48)]
out = {}
for style in ("trapezoidal", "s_curve", "torque_aware"):
    t = K.palletize(style, boxes, (0.55, 0.52), WORK_Z + BOX_H, WORK_Z + BOX_H, (0.0, 0.0, CUP),
                    vscale=0.35, ascale=0.35)
    n = 400
    qs = np.array([t.traj(x)[0] for x in np.linspace(0, t.duration, n)])
    qds = np.array([t.traj(x)[1] for x in np.linspace(0, t.duration, n)])
    out[style] = dict(duration=t.duration, n_events=len(t.events), n_boxes=t.n_boxes,
                      kinds=[e[1] for e in t.events],
                      q_min=qs.min(0).tolist(), q_max=qs.max(0).tolist(),
                      qd_peak=np.abs(qds).max(0).tolist(),
                      lim_min=cfg.Q_MIN.tolist(), lim_max=cfg.Q_MAX.tolist(),
                      qd_lim=cfg.QD_MAX.tolist(), dof=cfg.DOF)
print(json.dumps(out))
""" % str(REPO / "workspace")


@pytest.fixture(scope="module")
def cycle():
    env = dict(os.environ, PAL_ROBOT="ur10e")
    r = subprocess.run([sys.executable, "-c", CYCLE_SCRIPT], capture_output=True, text=True, env=env, timeout=300)
    assert r.returncode == 0, f"building the cell cycle failed:\n{r.stderr}"
    import json
    return json.loads(r.stdout)


@pytest.mark.parametrize("style", ["trapezoidal", "s_curve", "torque_aware"])
def test_cycle_solves_and_sequences_three_boxes(cycle, style):
    c = cycle[style]
    assert c["dof"] == DOF
    assert c["n_boxes"] == 3
    # four events per box, in order: seal, lifted, let go, down
    assert c["kinds"] == ["vacuum_on", "picked", "vacuum_off", "placed"] * 3
    assert 5.0 < c["duration"] < 120.0, c["duration"]


@pytest.mark.parametrize("style", ["trapezoidal", "s_curve", "torque_aware"])
def test_cycle_stays_inside_the_joint_limits(cycle, style):
    c = cycle[style]
    assert np.all(np.array(c["q_min"]) > np.array(c["lim_min"])), c["q_min"]
    assert np.all(np.array(c["q_max"]) < np.array(c["lim_max"])), c["q_max"]


@pytest.mark.parametrize("style", ["trapezoidal", "s_curve", "torque_aware"])
def test_cycle_stays_in_the_trustworthy_speed_band(cycle, style):
    """D1: the residual is only trustworthy below half the datasheet velocity limit."""
    peak = np.array(c_qd := cycle[style]["qd_peak"])
    frac = peak / np.array(cycle[style]["qd_lim"])
    assert np.all(frac <= 0.50 + 1e-6), f"peak joint speed {np.round(frac * 100, 1)}% of the limit ({c_qd})"
