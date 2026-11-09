"""The cell's layout and its camera, checked without a simulator.

`pal_cell_geom.py` holds the whole cell as plain numbers with no USD import, which is what makes
these checkable at all. Everything here is geometry: whether the pick targets are inside the arm's
reach, whether the carry height actually clears the boxes still on the table, and whether the
camera is looking at the cell from where it is supposed to be.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for sub in ("workspace", "model"):
    sys.path.insert(0, str(ROOT / sub))

import pal_cell_geom as G     # noqa: E402
import pal_robot             # noqa: E402

SPEC = pal_robot.active()


# ── the layout ──────────────────────────────────────────────────────────────────────────
def test_every_pick_and_place_target_is_inside_the_arms_reach():
    """With margin. This layout is the reason the arm was changed: none of it fits an 855 mm arm,
    and an unreachable target shows up as an IK failure deep inside a run rather than here."""
    targets = [(x, y, G.BOX_TOP_Z) for x, y in G.box_positions()]
    targets += [(G.PLACE_XY[0], G.PLACE_XY[1], G.PLACE_TOP_Z)]
    targets += [(x, y, G.CARRY_Z) for x, y in G.box_positions()]
    for p in targets:
        d = float(np.linalg.norm(p))
        assert d < 0.88 * SPEC.reach, f"{p} is {d:.3f} m out, against a {SPEC.reach} m reach"


def test_the_carry_height_clears_the_boxes_still_on_the_table():
    """A carried box hangs a full box-height below the cup plane, so the carry height has to be
    the box top PLUS a box PLUS clearance. An approach of one box height left the carried box's
    underside exactly level with its neighbours' lids and clipping them."""
    underside_of_carried_box = G.CARRY_Z - G.BOX[2]
    assert underside_of_carried_box > G.BOX_TOP_Z, "a carried box would scrape the waiting ones"
    assert underside_of_carried_box - G.BOX_TOP_Z >= 0.10, "less than 100 mm of clearance"


def test_boxes_do_not_overlap_each_other_on_the_infeed_table():
    xs = sorted(x for x, _ in G.box_positions())
    for a, b in zip(xs, xs[1:]):
        assert b - a > G.BOX[0], f"boxes at x {a} and {b} overlap ({G.BOX[0]} m wide)"


def test_the_boxes_fit_on_the_table_and_the_place_point_is_on_the_belt():
    half_x, half_y = G.TABLE_SIZE[0] / 2, G.TABLE_SIZE[1] / 2
    for x, y in G.box_positions():
        assert abs(x - G.TABLE_C[0]) + G.BOX[0] / 2 <= half_x, f"box at x {x} overhangs the table"
        assert abs(y - G.TABLE_C[1]) + G.BOX[1] / 2 <= half_y, f"box at y {y} overhangs the table"
    px, py = G.PLACE_XY
    assert G.BELT_X0 < px < G.BELT_X1, "the place point is off the end of the belt"
    assert abs(py - G.BELT_Y) + G.BOX[1] / 2 <= G.BELT_W / 2, "a placed box hangs off the belt"


def test_a_box_is_released_above_the_belt_not_set_down_on_it():
    """The belt should catch the box rather than the cups scuffing the surface, but the drop has to
    stay small enough that the box does not bounce or topple."""
    assert 0.001 <= G.DROP_GAP <= 0.010
    assert G.PLACE_TOP_Z > G.BOX_TOP_Z


# ── the camera ──────────────────────────────────────────────────────────────────────────
# The real lens, from pal_config: a 22 mm focal length on a 20.955 mm aperture at 1280x720.
FOCAL, APERTURE = 22.0, 20.955
WIDTH, HEIGHT = 1280, 720
HALF_H = math.atan(APERTURE / 2 / FOCAL)
HALF_V = math.atan(APERTURE * HEIGHT / WIDTH / 2 / FOCAL)

# What has to be on screen from every view: the arm and its pedestal, the three boxes, the place
# point over the belt, and the carry height above each box.
WORKING_VOLUME = ([(0.0, 0.0, 0.0), (0.0, 0.0, G.WORK_Z)]
                  + [(x, y, G.BOX_TOP_Z) for x, y in G.box_positions()]
                  + [(G.PLACE_XY[0], G.PLACE_XY[1], G.PLACE_TOP_Z)]
                  + [(x, y, G.CARRY_Z) for x, y in G.box_positions()])


def _frame_angles(view, points):
    """Each point's horizontal and vertical angle off the camera axis, in radians.

    A camera basis with no roll: `right` is horizontal, `up` completes it. The two axes are kept
    separate because the frame is not square."""
    eye, target = G.camera(view)
    fwd = np.array(target, float) - np.array(eye, float)
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    out = []
    for p in points:
        v = np.array(p, float) - np.array(eye, float)
        fore = float(v @ fwd)
        assert fore > 0, f"{view}: {p} is behind the camera"
        out.append((math.atan2(float(v @ right), fore), math.atan2(float(v @ up), fore)))
    return out


def _span(view):
    """(vertical, horizontal) angular extent of the working volume, as a fraction of the frame.
    This is apparent size: how big the cell looks, not merely whether it fits."""
    a = _frame_angles(view, WORKING_VOLUME)
    h = [x[0] for x in a]
    v = [x[1] for x in a]
    return (max(v) - min(v)) / (2 * HALF_V), (max(h) - min(h)) / (2 * HALF_H)

def test_the_default_view_is_isometric_and_not_a_side_shot():
    """The default was an almost level 16-degree shot across the table, which flattened the infeed
    table and the belt into one another. It is now a three-quarter view looking down; this pins the
    elevation so it cannot drift back without a test saying so."""
    _, _, elevation, _ = G.CAM_VIEWS[G.CAM_DEFAULT]
    assert 25.0 <= elevation <= 40.0, f"the default view's elevation is {elevation} deg"

    eye, target = G.camera()
    assert eye[2] > target[2] + 1.0, "the camera should be well above what it is looking at"
    # In front of the cell (-y) and to its right (+x): the side the conveyor runs away from, which
    # leaves the back wall and its sign visible behind the arm rather than behind the camera.
    assert eye[0] > target[0] and eye[1] < target[1]


def test_the_default_view_shows_the_arm_larger_than_the_old_side_view_did():
    """Raising the camera was not supposed to cost apparent size, and measured against the frustum
    it does not: the working volume spans a larger share of the frame from the isometric default
    than from the view it replaced. Worth pinning, because the obvious way to fit more in is to
    back away, and that would undo the point of the change."""
    assert _span(G.CAM_DEFAULT)[0] > _span("side")[0] * 1.2


def test_every_view_matches_its_own_distance_and_angles():
    """`camera()` derives the eye from a distance and two angles rather than carrying coordinates.
    This checks the derivation, so a nudged angle cannot silently also move the standoff."""
    for name, (d, az, el, tgt) in G.CAM_VIEWS.items():
        eye, target = G.camera(name)
        assert target == tuple(tgt), name
        v = np.array(eye) - np.array(target)
        assert abs(np.linalg.norm(v) - d) < 1e-9, name
        assert abs(math.degrees(math.asin(v[2] / d)) - el) < 1e-9, name
        assert abs(math.degrees(math.atan2(v[1], v[0])) - az) < 1e-9, name


def test_an_explicit_target_overrides_the_views_own():
    eye, target = G.camera("iso", target=(1.0, 0.0, 0.0))
    assert target == (1.0, 0.0, 0.0)
    d = G.CAM_VIEWS["iso"][0]
    assert abs(np.linalg.norm(np.array(eye) - np.array(target)) - d) < 1e-9


def test_a_sixteen_by_nine_frame_is_narrower_vertically():
    """The fact the next test rests on. Checking one cone on the horizontal half-angle would pass a
    point that is in fact cropped off the top, and an isometric view is precisely the one that
    pushes things up the frame."""
    assert HALF_V < HALF_H
    assert 14.5 < math.degrees(HALF_V) < 15.5 and 25.0 < math.degrees(HALF_H) < 26.0


def test_every_view_frames_the_whole_cell():
    """Nothing in the working volume may fall outside the frustum, from any named view. Checked as
    an angle rather than by eye, because "it looked fine on my monitor" is how a view gets cropped
    for the next person."""
    for name in G.CAM_VIEWS:
        for p, (ah, av) in zip(WORKING_VOLUME, _frame_angles(name, WORKING_VOLUME)):
            assert abs(ah) < HALF_H, (f"{name}: {p} is {math.degrees(abs(ah)):.1f} deg off axis "
                                      f"horizontally, outside {math.degrees(HALF_H):.1f} deg")
            assert abs(av) < HALF_V, (f"{name}: {p} is {math.degrees(abs(av)):.1f} deg off axis "
                                      f"vertically, outside {math.degrees(HALF_V):.1f} deg")


def test_the_wide_views_keep_the_sign_and_the_screen_in_shot():
    """The NTT sign and the telemetry screen are background, not subjects — framing *for* them
    forces the camera so far back that the arm goes small, which is the trade the view set exists
    to offer. So this pins which views keep them and does not demand that all of them do.

    `iso` keeping both is the point of its particular target: aiming a little towards the back wall
    is what pulls them in, and that is easy to lose while nudging the angles."""
    sign = (-0.62, 1.70, G.FLOOR_Z + 1.50)
    screen = (0.95, 1.42, G.FLOOR_Z + 1.26)

    def visible(view, p):
        ah, av = _frame_angles(view, [p])[0]
        return abs(ah) < HALF_H and abs(av) < HALF_V

    assert visible(G.CAM_DEFAULT, sign) and visible(G.CAM_DEFAULT, screen)
    assert visible("side", sign) and visible("side", screen)
    # And the close view is genuinely close: it gives up the sign to make the arm bigger.
    assert not visible("iso_close", sign)
    assert _span("iso_close")[0] > _span(G.CAM_DEFAULT)[0]


def test_an_unknown_view_is_refused_by_name():
    try:
        G.camera("three_quarter")
    except ValueError as e:
        assert "three_quarter" in str(e) and "iso" in str(e)
    else:
        raise AssertionError("an unknown view name was accepted")


def test_every_suction_cup_hangs_from_a_bar():
    """The cups sit at the corners of a square; the two bars must run through them. They once ran along the axes, leaving each cup 0.085 m
    from the nearest bar, which showed in the sim as cups with stems hanging in the air."""
    import sys
    from pathlib import Path
    import numpy as np
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "workspace"))
    import pal_suction as S
    bars = [np.radians(a) for a in S.BAR_ANGLES_DEG]                # the angles pal_suction._build gives its two bars
    for sx in (1, -1):
        for sy in (1, -1):
            c = np.array([sx * S.CUP_SPAN, sy * S.CUP_SPAN])
            ok = False
            for a in bars:
                d = np.array([np.cos(a), np.sin(a)])
                along, across = float(c @ d), float(abs(c[0] * d[1] - c[1] * d[0]))
                ok |= abs(along) <= S.ARM_LEN / 2 and across <= S.ARM_W / 2 + 1e-9
            assert ok, f"cup at {c} is not on either bar"
