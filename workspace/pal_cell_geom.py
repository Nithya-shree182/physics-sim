"""The conveyor cell's geometry, as plain numbers. No Isaac, no USD, no pxr.

Split out of `pal_cell.py` so that the things which only need to KNOW the layout — the palletize
cycle, the suites, the tests, the dashboard — can read it without a simulator. `pal_cell.py`
imports these and builds the actual stage from them, so there is still one definition of where
anything is.

Layout, in the robot's base frame (the base sits at the origin, on a pedestal):

      y
      ^            +-------------------------------------+   back wall
      |            |   desk + telemetry screen           |
   +0.55  ========== CONVEYOR (runs towards +x) ===========
      |
    0 +            (O) UR10e
      |
   -0.52           [box] [box] [box]   on the infeed table
      +-------------------------------------------------> x
                  0.38   0.66   0.94

Heights: the floor is FLOOR_Z below the robot's base, the table top and the belt are both at
WORK_Z, and a box is BOX[2] tall, so its top face — what the suction cups aim at — is at
WORK_Z + BOX[2]. Every pick target is inside 1.1 m of the base, comfortably inside the UR10e's
1.3 m reach; this layout is the reason the arm was changed, because none of it fits an 855 mm arm.
"""
from __future__ import annotations

# ── cell geometry, metres ─────────────────────────────────────────────────────────────────────
FLOOR_Z = -0.90                 # the shop floor, below the robot's base
WORK_Z = -0.25                  # table top and the top of the belt
BOX = (0.22, 0.22, 0.17)        # a cardboard box
BOX_DEFAULT_KG = 1.5
N_BOXES = 3
BOX_X = (0.38, 0.66, 0.94)      # where the three boxes start, lined up on the infeed table
BOX_Y = -0.52
# Boxes are let go this far above the belt rather than set down on it, so the belt takes them
# with a small drop the way a real singulation cell does, and the cups never scuff the surface.
DROP_GAP = 0.0035
# How much air is left under a carried box as it travels. It has to clear the boxes still waiting
# on the table: the carry height is measured to the CUP PLANE, so the box hangs a full box-height
# below it, and an approach height of one box height left the carried box's underside exactly
# level with its neighbours' lids and clipping them. Carry height = box top + box height + this.
LIFT_CLEARANCE = 0.16

TABLE_C = (0.66, BOX_Y)         # infeed table centre
TABLE_SIZE = (1.02, 0.62, 0.06)

BELT_Y = 0.55                   # conveyor centre line
BELT_X0, BELT_X1 = -0.55, 2.25  # the belt runs this far along x
BELT_W = 0.46
BELT_SPEED = 0.28               # m/s, towards +x
PLACE_XY = (0.50, BELT_Y)       # where every box is let go over the belt
ROLLER_R = 0.028
N_ROLLERS = 20

PEDESTAL_R = 0.15

WALL_Y = 2.45                   # back wall
WALL_X = -2.10                  # side wall

# ── colours ───────────────────────────────────────────────────────────────────────────────────
CARDBOARD = (0.74, 0.56, 0.37)
FLOOR_COL = (0.38, 0.40, 0.43)
WALL_COL = (0.90, 0.91, 0.92)
STEEL = (0.60, 0.63, 0.68)
BELT_COL = (0.13, 0.14, 0.16)
ROLLER_COL = (0.72, 0.74, 0.78)
DESK_COL = (0.78, 0.76, 0.72)
SCREEN_COL = (0.04, 0.05, 0.07)
BAR_COL = (0.21, 0.72, 0.95)

# ── the viewport camera ───────────────────────────────────────────────────────────────────────
# Named views rather than one pair of coordinates, because what you want to look at depends on what
# you are looking FOR, and because an eye position is far easier to get right as a distance and two
# angles than as three metres.
#
# What has to be on screen: the arm (base at the origin), the three boxes on the infeed table at
# y = -0.52, the place point over the belt at y = +0.55, and the carry height above each box. The
# NTT sign at (-0.62, 1.70) and the telemetry screen at (0.95, 1.42) are BACKGROUND — nice to have
# in shot, and the wide views keep them, but framing for them forces the camera so far back that
# the arm itself goes small. That trade is what separates the views below.
#
# All of them are checked against the real 22 mm / 1280x720 frustum in tests/test_cell_geometry.py,
# horizontally and vertically separately: a 16:9 frame is much narrower vertically (15.0 degree
# half-angle against 25.5), so a view can look fine on the horizontal check and still be cropped
# off the top — which is exactly the mistake an isometric view invites.

# (distance m, azimuth deg, elevation deg, target). Azimuth is measured in the xy plane from +x
# towards +y, so a negative value puts the camera in front of the cell (-y) and to its right (+x):
# the side the conveyor runs away from, which leaves the back wall visible behind the arm.
CAM_VIEWS = {
    # The default, since 2026-11-08. A three-quarter view looking down 30 degrees. It replaced an
    # almost level 16-degree shot that flattened the infeed table and the belt into one another;
    # measured against the frustum, this is better on both counts at once — the arm spans 42% of
    # the frame's height where the old view gave it 27%, and the sign and the screen are still in
    # shot. 30 rather than a true isometric 35.26 because the sign foreshortens away above that.
    "iso": (3.20, -45.0, 30.0, (0.65, 0.30, 0.25)),
    # A true isometric: 35.26 degrees is the elevation at which the three axes foreshorten equally,
    # so it reads as a technical drawing. Pulled back a little to keep the cell in frame.
    "iso_high": (3.50, -45.0, 35.26, (0.65, 0.30, 0.30)),
    # Closer, aimed at the work rather than the room. The arm and the boxes fill the frame; the
    # sign and most of the back wall fall outside it. The one to use for watching the cups seal.
    "iso_close": (2.45, -50.0, 28.0, (0.60, 0.00, 0.05)),
    # The original framing, kept so it can be asked for by name: low and almost level, like the
    # reference photograph the cell was built from.
    "side": (3.35, -62.0, 16.0, (0.45, 0.35, 0.10)),
    # Nearly overhead, for checking the layout rather than watching the arm.
    "top": (3.20, -75.0, 60.0, (0.60, 0.05, 0.00)),
}
CAM_DEFAULT = "iso"
CAM_TARGET = CAM_VIEWS[CAM_DEFAULT][3]


def camera(view=None, target=None):
    """(eye, target) for a named view, as metres in the robot's base frame.

    Derived from the distance and the two angles rather than written down, so a view can be nudged
    by changing an angle instead of solving for three coordinates by hand."""
    import math
    name = view or CAM_DEFAULT
    if name not in CAM_VIEWS:
        raise ValueError(f"unknown camera view {name!r}; have {sorted(CAM_VIEWS)}")
    d, az, el, tgt = CAM_VIEWS[name]
    tgt = tuple(tgt if target is None else target)
    h = d * math.cos(math.radians(el))          # horizontal standoff
    return (tgt[0] + h * math.cos(math.radians(az)),
            tgt[1] + h * math.sin(math.radians(az)),
            tgt[2] + d * math.sin(math.radians(el))), tgt


CAM_EYE, _ = camera()               # the default view, for callers that want the pair directly

# ── derived, so nothing recomputes them ───────────────────────────────────────────────────────
BOX_TOP_Z = WORK_Z + BOX[2]              # the face the suction cups aim at
PLACE_TOP_Z = BOX_TOP_Z + DROP_GAP       # where a box is released over the belt
CARRY_Z = BOX_TOP_Z + BOX[2] + LIFT_CLEARANCE   # cup-plane height while carrying


def box_positions(n=N_BOXES):
    """(x, y) of each box waiting on the infeed table, in pick order."""
    return [(x, BOX_Y) for x in BOX_X[:n]]
