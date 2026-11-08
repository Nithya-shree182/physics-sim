"""The industrial workcell the UR10e works in: an infeed table of cardboard boxes, a running
conveyor, and enough of a room around it to read as a real place.

Layout, in the robot's base frame (the base sits at the origin, on a pedestal):

      y
      ^            +-------------------------------------+   back wall
      |            |   desk + laptop        desk + laptop |
   +0.52  ========== CONVEYOR (runs towards +x) ===========
      |
    0 +            (O) UR10e
      |
   -0.48           [box] [box] [box]   on the infeed table
      +-------------------------------------------------> x
                  0.40   0.62   0.84

Heights: the floor is FLOOR_Z below the robot's base, the table top and the belt are both at
WORK_Z, and a box is BOX[2] tall, so its top face — what the suction cups aim at — is at
WORK_Z + BOX[2]. Every pick target is inside 1.1 m of the base, comfortably inside the UR10e's
1.3 m reach; this layout is the reason the arm was changed, because none of it fits a Panda.

The conveyor really moves: `step()` puts the belt's surface speed onto any box resting on it, so a
placed box is carried away and the next pick has a clear spot. The rollers turn at the matching
rate so the motion is visible rather than implied.

The two laptops on the back desks are not decoration: their screens carry a live bar per joint
showing that joint's torque as a fraction of its limit, so the physics being measured is visible
in the same picture as the arm measuring it.
"""
from __future__ import annotations

import numpy as np

import pal_config as cfg

# ── cell geometry, metres ─────────────────────────────────────────────────────────────────────
FLOOR_Z = -0.90                 # the shop floor, below the robot's base
WORK_Z = -0.25                  # table top and belt surface
BOX = (0.16, 0.16, 0.13)        # a cardboard box
BOX_DEFAULT_KG = 1.5
N_BOXES = 3
BOX_X = (0.40, 0.62, 0.84)      # where the three boxes start, lined up on the infeed table
BOX_Y = -0.48

TABLE_C = (0.62, BOX_Y)         # infeed table centre
TABLE_SIZE = (0.72, 0.42, 0.055)

BELT_Y = 0.52                   # conveyor centre line
BELT_X0, BELT_X1 = -0.45, 1.95  # the belt runs this far along x
BELT_W = 0.40
BELT_SPEED = 0.28               # m/s, towards +x
PLACE_XY = (0.55, BELT_Y)       # where every box is set down on the belt

PEDESTAL_R = 0.14

WALL_Y = 2.35                   # back wall
WALL_X = -2.10                  # side wall
DESKS = ((-0.35, 1.70), (1.05, 1.70))

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

CAM_EYE = (2.45, -2.15, 1.05)
CAM_TARGET = (0.45, 0.05, -0.22)


def _check_path(path):
    """USD prim names are identifiers: letters, digits and underscore, never starting with a digit.
    A minus sign is the easy mistake (an f-string over (-1, 1) gives "Rail-1"), and USD reports it
    as the baffling "Path must be an absolute path: <>" rather than naming the offender."""
    from pxr import Sdf
    if not Sdf.Path.IsValidPathString(path)[0]:
        raise ValueError(f"{path!r} is not a usable USD prim path; names must be [A-Za-z_][A-Za-z0-9_]*")
    return path


def _xform_cube(stage, path, size, color, translate=(0, 0, 0), rotate=None):
    from pxr import Gf, UsdGeom
    g = UsdGeom.Cube.Define(stage, _check_path(path))
    g.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(g)
    x.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in translate]))
    if rotate:
        x.AddRotateXYZOp().Set(Gf.Vec3f(*[float(v) for v in rotate]))
    sc = x.AddScaleOp()
    sc.Set(Gf.Vec3f(*[float(v) for v in size]))
    g.CreateDisplayColorAttr([Gf.Vec3f(*[float(c) for c in color])])
    return g, sc


def _cyl(stage, path, radius, height, color, translate=(0, 0, 0), rotate=None):
    from pxr import Gf, UsdGeom
    g = UsdGeom.Cylinder.Define(stage, _check_path(path))
    g.CreateRadiusAttr(float(radius)); g.CreateHeightAttr(float(height)); g.CreateAxisAttr("Z")
    x = UsdGeom.Xformable(g)
    x.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in translate]))
    rop = x.AddRotateXYZOp()
    rop.Set(Gf.Vec3f(*[float(v) for v in (rotate or (0, 0, 0))]))
    g.CreateDisplayColorAttr([Gf.Vec3f(*[float(c) for c in color])])
    return g, rop


class Box:
    """One cardboard box: a real rigid body with a settable mass."""

    def __init__(self, world, index, xy, kg, material):
        from isaacsim.core.api.objects import DynamicCuboid
        self.index = index
        self.path = f"/World/Cell/Box{index}"
        self.name = f"box{index}"
        self.size = np.array(BOX, float)
        self.start_xy = tuple(xy)
        world.scene.add(DynamicCuboid(
            prim_path=self.path, name=self.name,
            position=np.array([xy[0], xy[1], WORK_Z + BOX[2] / 2 + 0.001]),
            scale=self.size, color=np.array(CARDBOARD), mass=float(kg),
            physics_material=material))
        self.view = None
        self.placed = False          # has it been set down on the belt yet

    def initialize(self):
        from isaacsim.core.prims import RigidPrim
        self.view = RigidPrim(self.path)
        self.view.initialize()

    def position(self):
        return np.asarray(self.view.get_world_poses()[0]).reshape(3).copy()

    def top_centre(self):
        p = self.position()
        return p + np.array([0.0, 0.0, self.size[2] / 2])

    def set_mass(self, kg):
        self.view.set_masses(np.array([float(kg)], np.float32))
        back = float(np.asarray(self.view.get_masses()).reshape(-1)[0])
        assert abs(back - kg) < 1e-3, f"box {self.index} mass write did not take: asked {kg}, got {back}"
        return back

    def reset(self, xy=None, kg=None):
        xy = self.start_xy if xy is None else xy
        if kg is not None:
            self.set_mass(kg)
        self.view.set_world_poses(np.array([[xy[0], xy[1], WORK_Z + BOX[2] / 2 + 0.001]], np.float32),
                                  np.array([[1.0, 0.0, 0.0, 0.0]], np.float32))
        self.view.set_velocities(np.zeros((1, 6), np.float32))
        self.placed = False

    def on_belt(self):
        p = self.position()
        return (BELT_X0 < p[0] < BELT_X1 and abs(p[1] - BELT_Y) < BELT_W / 2 + 0.05
                and abs(p[2] - (WORK_Z + self.size[2] / 2)) < 0.06)


class ConveyorCell:
    """Everything in the room except the robot."""

    def __init__(self, world, spec, n_boxes=N_BOXES, box_kg=BOX_DEFAULT_KG):
        import omni.usd
        from isaacsim.core.api.materials import PhysicsMaterial
        from pxr import UsdGeom

        self.world = world
        self.spec = spec
        self.stage = omni.usd.get_context().get_stage()
        UsdGeom.Xform.Define(self.stage, "/World/Cell")

        self.grip_mat = PhysicsMaterial(prim_path="/World/Physics/Cardboard", static_friction=0.90,
                                        dynamic_friction=0.75, restitution=0.0)
        self.surface_mat = PhysicsMaterial(prim_path="/World/Physics/CellSurface", static_friction=0.70,
                                           dynamic_friction=0.60, restitution=0.0)
        self._room()
        self._pedestal()
        self._table()
        self._conveyor()
        self._desks()
        self.boxes = [Box(world, i, (BOX_X[i % len(BOX_X)], BOX_Y), box_kg, self.grip_mat)
                      for i in range(n_boxes)]
        self.tool = None                       # the SuctionTool, mounted once the robot exists
        self._roller_angle = 0.0

    # -- static scenery ------------------------------------------------------------------------
    def _room(self):
        from isaacsim.core.api.objects import FixedCuboid
        self.world.scene.add(FixedCuboid(
            prim_path="/World/Cell/Floor", name="cell_floor",
            position=np.array([0.3, 0.3, FLOOR_Z - 0.05]), scale=np.array([9.0, 9.0, 0.10]),
            color=np.array(FLOOR_COL), physics_material=self.surface_mat))
        # Walls are visual only: nothing should ever touch them, and a collider there would only
        # give the solver more pairs to test every step.
        _xform_cube(self.stage, "/World/Cell/WallBack", (9.0, 0.08, 3.1), WALL_COL, (0.3, WALL_Y, FLOOR_Z + 1.55))
        _xform_cube(self.stage, "/World/Cell/WallSide", (0.08, 9.0, 3.1), WALL_COL, (WALL_X, 0.3, FLOOR_Z + 1.55))
        # A painted safety border on the floor, the usual yellow line around a robot cell.
        for i, sx in enumerate((-1, 1)):
            _xform_cube(self.stage, f"/World/Cell/SafetyLine{i}", (0.06, 4.4, 0.004), (0.86, 0.72, 0.12),
                        (0.45 + sx * 1.55, 0.1, FLOOR_Z + 0.003))

    def _pedestal(self):
        _cyl(self.stage, "/World/Cell/Pedestal", PEDESTAL_R, abs(FLOOR_Z), (0.22, 0.24, 0.28),
             (0, 0, FLOOR_Z / 2))
        _cyl(self.stage, "/World/Cell/PedestalFoot", PEDESTAL_R * 1.9, 0.03, (0.30, 0.32, 0.36),
             (0, 0, FLOOR_Z + 0.015))

    def _table(self):
        from isaacsim.core.api.objects import FixedCuboid
        cx, cy = TABLE_C
        self.world.scene.add(FixedCuboid(
            prim_path="/World/Cell/InfeedTable", name="infeed_table",
            position=np.array([cx, cy, WORK_Z - TABLE_SIZE[2] / 2]),
            scale=np.array(TABLE_SIZE), color=np.array(STEEL),
            physics_material=self.surface_mat))
        for i, sx in enumerate((-1, 1)):
            for j, sy in enumerate((-1, 1)):
                _cyl(self.stage, f"/World/Cell/TableLeg{i}{j}", 0.022,
                     abs(FLOOR_Z - WORK_Z) - TABLE_SIZE[2],
                     (0.30, 0.32, 0.36),
                     (cx + sx * (TABLE_SIZE[0] / 2 - 0.05), cy + sy * (TABLE_SIZE[1] / 2 - 0.05),
                      (FLOOR_Z + WORK_Z - TABLE_SIZE[2]) / 2))

    def _conveyor(self):
        from isaacsim.core.api.objects import FixedCuboid
        length = BELT_X1 - BELT_X0
        cx = (BELT_X0 + BELT_X1) / 2
        # The belt surface is the only part with a collider: boxes rest on it and are carried.
        self.world.scene.add(FixedCuboid(
            prim_path="/World/Cell/BeltSurface", name="belt_surface",
            position=np.array([cx, BELT_Y, WORK_Z - 0.02]),
            scale=np.array([length, BELT_W, 0.04]), color=np.array(BELT_COL),
            physics_material=self.surface_mat))
        for i, sy in enumerate((-1, 1)):                     # side rails
            _xform_cube(self.stage, f"/World/Cell/BeltRail{i}", (length, 0.035, 0.075), STEEL,
                        (cx, BELT_Y + sy * (BELT_W / 2 + 0.02), WORK_Z + 0.015))
        for i, sy in enumerate((-1, 1)):                     # frame beams and legs
            _xform_cube(self.stage, f"/World/Cell/BeltBeam{i}", (length, 0.05, 0.10), (0.34, 0.36, 0.40),
                        (cx, BELT_Y + sy * (BELT_W / 2 + 0.02), WORK_Z - 0.14))
            for k in range(5):
                x = BELT_X0 + 0.25 + k * (length - 0.5) / 4
                _cyl(self.stage, f"/World/Cell/BeltLeg{i}{k}", 0.028, abs(FLOOR_Z - WORK_Z) - 0.19,
                     (0.30, 0.32, 0.36),
                     (x, BELT_Y + sy * (BELT_W / 2 + 0.02), (FLOOR_Z + WORK_Z - 0.19) / 2))
        # Rollers: visual, and they spin at the belt's surface rate so the motion reads.
        self._rollers = []
        n_roll = 14
        for k in range(n_roll):
            x = BELT_X0 + 0.08 + k * (length - 0.16) / (n_roll - 1)
            _, rop = _cyl(self.stage, f"/World/Cell/Roller{k}", 0.028, BELT_W - 0.03, ROLLER_COL,
                          (x, BELT_Y, WORK_Z + 0.004), (90, 0, 0))
            self._rollers.append(rop)

    def _desks(self):
        """Two desks against the back wall, each with a laptop whose screen shows live joint torque."""
        self.bars = []           # (scale op, base translate op) per joint, across both laptops
        self._bar_ops = []
        for d, (dx, dy) in enumerate(DESKS):
            _xform_cube(self.stage, f"/World/Cell/Desk{d}/Top", (1.15, 0.58, 0.04), DESK_COL,
                        (dx, dy, FLOOR_Z + 0.72))
            for i, sx in enumerate((-1, 1)):
                for j, sy in enumerate((-1, 1)):
                    _cyl(self.stage, f"/World/Cell/Desk{d}/Leg{i}{j}", 0.018, 0.70, (0.35, 0.36, 0.38),
                         (dx + sx * 0.52, dy + sy * 0.24, FLOOR_Z + 0.35))
            # laptop: base, and a screen tilted back, facing -y (towards the robot and the camera)
            _xform_cube(self.stage, f"/World/Cell/Desk{d}/Base", (0.34, 0.24, 0.015), (0.22, 0.23, 0.25),
                        (dx, dy, FLOOR_Z + 0.748))
            _xform_cube(self.stage, f"/World/Cell/Desk{d}/Lid", (0.34, 0.015, 0.23), (0.22, 0.23, 0.25),
                        (dx, dy + 0.11, FLOOR_Z + 0.855), (-15, 0, 0))
            _xform_cube(self.stage, f"/World/Cell/Desk{d}/Screen", (0.315, 0.004, 0.205), SCREEN_COL,
                        (dx, dy + 0.1, FLOOR_Z + 0.855), (-15, 0, 0))
        # One bar per joint, spread over the two screens.
        n = self.spec.dof
        per = int(np.ceil(n / len(DESKS)))
        for k in range(n):
            d = k // per
            dx, dy = DESKS[min(d, len(DESKS) - 1)]
            slot = k - d * per
            x = dx - 0.12 + slot * 0.075
            _, sc = _xform_cube(self.stage, f"/World/Cell/Desk{d}/Bar{k}", (0.05, 0.003, 0.02), BAR_COL,
                                (x, dy + 0.097, FLOOR_Z + 0.785), (-15, 0, 0))
            self._bar_ops.append(sc)

    # -- life cycle ----------------------------------------------------------------------------
    def initialize(self):
        for b in self.boxes:
            b.initialize()

    def mount_tool(self, robot_prim):
        """Build the suction tool on the robot's wrist. Idempotent: the plant mounts it before the
        articulation is built, and callers may ask for it again later."""
        if self.tool is None:
            from pal_suction import SuctionTool
            self.tool = SuctionTool(self.stage, robot_prim, [b.path for b in self.boxes])
        return self.tool

    def frame_camera(self):
        try:
            from isaacsim.core.utils.viewports import set_camera_view
            set_camera_view(eye=np.array(CAM_EYE), target=np.array(CAM_TARGET))
        except Exception as e:
            print(f"[cell] camera not set: {e!r}", flush=True)

    # -- per step ------------------------------------------------------------------------------
    def step(self, dt, tau_frac=None):
        """Carry whatever is on the belt, spin the rollers, and refresh the torque bars."""
        for b in self.boxes:
            if self.tool is not None and self.tool.held == b.path:
                continue
            if b.on_belt():
                v = np.asarray(b.view.get_velocities()).reshape(6).copy()
                v[0], v[1] = BELT_SPEED, 0.0           # drive along the belt, hold the line
                b.view.set_velocities(v.reshape(1, 6).astype(np.float32))
        self._roller_angle = (self._roller_angle + np.degrees(BELT_SPEED / 0.028) * dt) % 360.0
        if self._rollers:
            from pxr import Gf
            for rop in self._rollers:
                rop.Set(Gf.Vec3f(90.0, float(self._roller_angle), 0.0))
        if tau_frac is not None:
            self.set_torque_bars(tau_frac)

    def set_torque_bars(self, frac):
        """`frac` is |tau| / tau_limit per joint; the bars grow upward from the screen's base."""
        from pxr import Gf
        for k, sc in enumerate(self._bar_ops):
            h = 0.02 + 0.16 * float(np.clip(frac[k], 0.0, 1.0))
            sc.Set(Gf.Vec3f(0.05, 0.003, h))

    # -- the task's view of the cell -------------------------------------------------------------
    def reset_boxes(self, kg=None):
        for b in self.boxes:
            b.reset(kg=kg)

    def pick_targets(self):
        """Top-face centres of the boxes still waiting on the infeed table, nearest first."""
        out = [(b, b.top_centre()) for b in self.boxes if not b.placed]
        return sorted(out, key=lambda bt: np.linalg.norm(bt[1][:2]))

    def position(self):
        """Compatibility with the old Workcell: where the 'current' box is."""
        return self.boxes[0].position() if self.boxes else np.zeros(3)
