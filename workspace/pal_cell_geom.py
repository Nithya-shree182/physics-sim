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
# Named views rather than one pair of coordinates, because what you want to look at depends on
# what you are looking FOR, and because an eye position is much easier to get right as a distance
# and two angles than as three metres.
#
# What has to fit in frame: the arm (base at the origin), the three boxes on the infeed table at
# y = -0.52, the belt at y = +0.55, and the sign and telemetry screen on the back wall. That is
# about 2.4 m across, which at the 22 mm lens (a 51 degree horizontal field) needs roughly 2.5 m
# of standoff; the distances below carry a little margin on top.
CAM_TARGET = (0.60, 0.00, 0.05)     # between the box row and the belt, at about carry height

# (distance m, azimuth deg, elevation deg). Azimuth is measured in the xy plane from +x towards
# +y, so a negative value puts the camera in front of the cell (-y) and to its right (+x), which
# is the side the conveyor runs away from and leaves the back wall visible behind the arm.
CAM_VIEWS = {
    # The default. A three-quarter view looking down about 30 degrees: enough to read the depth of
    # the cell and see into the boxes, without going so steep that the sign and the screen on the
    # back wall foreshorten away.
    "iso": (3.20, -52.0, 30.0),
    # A true isometric, 35.26 degrees — the angle at which the three axes foreshorten equally.
    # Reads as a technical drawing; slightly more top-down than most people mean by "isometric".
    "iso_high": (3.40, -45.0, 35.26),
    # The original framing: low and almost level, like the reference photograph. Keeps the arm
    # against the factory background but flattens the table and the belt into one another.
    "side": (3.30, -62.0, 11.0),
    # Nearly overhead, for checking the layout rather than watching the arm.
    "top": (3.20, -75.0, 62.0),
}
CAM_DEFAULT = "iso"


def camera(view=None, target=None):
    """(eye, target) for a named view, as metres in the robot's base frame.

    Derived from the distance and the two angles rather than written down, so a view can be
    nudged by changing an angle instead of solving for three coordinates by hand."""
    import math
    name = view or CAM_DEFAULT
    if name not in CAM_VIEWS:
        raise ValueError(f"unknown camera view {name!r}; have {sorted(CAM_VIEWS)}")
    d, az, el = CAM_VIEWS[name]
    tgt = tuple(CAM_TARGET if target is None else target)
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
