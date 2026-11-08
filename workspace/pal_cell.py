"""The industrial workcell the UR10e works in: an infeed table of cardboard boxes, a running
conveyor, and enough of a room around it to read as a real place.

The layout and every dimension are in `pal_cell_geom.py`, which imports no simulator; this
module turns them into USD. See that file for the plan view and the height convention.

The conveyor really moves: `step()` puts the belt's surface speed onto any box resting on it, so a
placed box is carried away and the next pick has a clear spot. The rollers turn at the matching
rate so the motion is visible rather than implied.

The telemetry screen on the back desk is not decoration: it carries a live bar per joint showing
that joint's torque as a fraction of its limit, so the physics being measured is visible in the
same picture as the arm measuring it.
"""
from __future__ import annotations

import numpy as np

import pal_cell_scenery as SC
import pal_config as cfg

# Geometry and colours live in pal_cell_geom, which imports no simulator, so the palletize cycle
# and the tests can read the layout without Isaac. Re-exported here because this module's own
# readers (and pal_cell_demo) refer to them unqualified.
from pal_cell_geom import *                                        # noqa: F401,F403
from pal_cell_geom import (BOX, BOX_DEFAULT_KG, BOX_TOP_Z, BOX_X, BOX_Y, BELT_SPEED, BELT_W,
                           BELT_X0, BELT_X1, BELT_Y, CAM_DEFAULT, CAM_EYE, CAM_TARGET, CAM_VIEWS,
                           CARDBOARD, CARRY_Z, DROP_GAP, FLOOR_COL, FLOOR_Z, LIFT_CLEARANCE,
                           N_BOXES, N_ROLLERS, PEDESTAL_R, PLACE_TOP_Z, PLACE_XY, ROLLER_R,
                           TABLE_C, TABLE_SIZE, WALL_X, WALL_Y, WORK_Z, camera)   # noqa: F401


def _check_path(path):
    """USD prim names are identifiers: letters, digits and underscore, never starting with a digit.
    A minus sign is the easy mistake (an f-string over (-1, 1) gives "Rail-1"), and USD reports it
    as the baffling "Path must be an absolute path: <>" rather than naming the offender."""
    from pxr import Sdf
    if not Sdf.Path.IsValidPathString(path)[0]:
        raise ValueError(f"{path!r} is not a usable USD prim path; names must be [A-Za-z_][A-Za-z0-9_]*")
    return path


_xform_cube_unused = None


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
        self.kg = float(kg)          # what the simulator currently has, so we never rewrite it for nothing
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
        """Write a new mass, and only a new one: changing a rigid body's mass mid-run makes PhysX
        re-read the scene, which has been seen to drop the articulation's physics view."""
        kg = float(kg)
        if abs(kg - self.kg) < 1e-9:
            return self.kg
        self.view.set_masses(np.array([kg], np.float32))
        back = float(np.asarray(self.view.get_masses()).reshape(-1)[0])
        assert abs(back - kg) < 1e-3, f"box {self.index} mass write did not take: asked {kg}, got {back}"
        self.kg = back
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
        self._screen()
        self.boxes = [Box(world, i, (BOX_X[i % len(BOX_X)], BOX_Y), box_kg, self.grip_mat)
                      for i in range(n_boxes)]
        self.tool = None                       # the SuctionTool, mounted once the robot exists
        self._roller_angle = 0.0
        self._t_vis = 0.0

    # -- static scenery ------------------------------------------------------------------------
    def _room(self):
        from isaacsim.core.api.objects import FixedCuboid
        self.world.scene.add(FixedCuboid(
            prim_path="/World/Cell/Floor", name="cell_floor",
            position=np.array([1.0, 2.0, FLOOR_Z - 0.05]), scale=np.array([26.0, 24.0, 0.10]),
            color=np.array(SC.FLOOR_COL), physics_material=self.surface_mat))
        SC.build_factory(self.stage, "/World/Cell", FLOOR_Z)
        SC.build_fence(self.stage, "/World/Cell", FLOOR_Z)
        SC.build_sign(self.stage, "/World/Cell", FLOOR_Z)
        SC.build_bollards(self.stage, "/World/Cell", FLOOR_Z)

    def _pedestal(self):
        SC.cyl(self.stage, "/World/Cell/Pedestal", PEDESTAL_R, abs(FLOOR_Z), (0.24, 0.26, 0.30),
               (0, 0, FLOOR_Z / 2))
        SC.cyl(self.stage, "/World/Cell/PedestalFoot", PEDESTAL_R * 1.8, 0.035, (0.32, 0.34, 0.38),
               (0, 0, FLOOR_Z + 0.018))
        SC.cyl(self.stage, "/World/Cell/PedestalCollar", PEDESTAL_R * 1.15, 0.04, (0.34, 0.36, 0.40),
               (0, 0, -0.03))

    def _table(self):
        from isaacsim.core.api.objects import FixedCuboid
        cx, cy = TABLE_C
        self.world.scene.add(FixedCuboid(
            prim_path="/World/Cell/InfeedTable", name="infeed_table",
            position=np.array([cx, cy, WORK_Z - TABLE_SIZE[2] / 2]),
            scale=np.array(TABLE_SIZE), color=np.array([0.78, 0.80, 0.83]),
            physics_material=self.surface_mat))
        for i, sx in enumerate((-1, 1)):            # extrusion rails under the top
            SC.cube(self.stage, f"/World/Cell/TableRail{i}", (TABLE_SIZE[0], 0.045, 0.045), SC.ALU,
                    (cx, cy + sx * (TABLE_SIZE[1] / 2 - 0.05), WORK_Z - TABLE_SIZE[2] - 0.022))
            for j, sy in enumerate((-1, 1)):        # square aluminium legs, as on the reference
                SC.cube(self.stage, f"/World/Cell/TableLeg{i}{j}", (0.045, 0.045,
                        abs(FLOOR_Z - WORK_Z) - TABLE_SIZE[2]), SC.ALU,
                        (cx + sy * (TABLE_SIZE[0] / 2 - 0.06), cy + sx * (TABLE_SIZE[1] / 2 - 0.05),
                         (FLOOR_Z + WORK_Z - TABLE_SIZE[2]) / 2))
                SC.cube(self.stage, f"/World/Cell/TableFoot{i}{j}", (0.07, 0.07, 0.016), (0.32, 0.34, 0.38),
                        (cx + sy * (TABLE_SIZE[0] / 2 - 0.06), cy + sx * (TABLE_SIZE[1] / 2 - 0.05),
                         FLOOR_Z + 0.008))

    def _conveyor(self):
        """A flat belt in an aluminium frame, as on the reference photograph.

        It used to be a bed of rollers, which looked wrong twice over: the rollers were drawn
        32 mm proud of the surface boxes actually rest on, so every box sat half sunk into them.
        The belt's top face and its collision surface are now the same plane at WORK_Z. Motion is
        shown by seam lines that run along the belt and wrap, plus turning end rollers."""
        from isaacsim.core.api.objects import FixedCuboid
        length = BELT_X1 - BELT_X0
        cx = (BELT_X0 + BELT_X1) / 2
        self.world.scene.add(FixedCuboid(
            prim_path="/World/Cell/BeltSurface", name="belt_surface",
            position=np.array([cx, BELT_Y, WORK_Z - 0.012]),
            scale=np.array([length, BELT_W, 0.024]), color=np.array(BELT_COL),
            physics_material=self.surface_mat))
        for i, sy in enumerate((-1, 1)):
            SC.cube(self.stage, f"/World/Cell/BeltRail{i}", (length, 0.05, 0.085), SC.ALU,
                    (cx, BELT_Y + sy * (BELT_W / 2 + 0.03), WORK_Z - 0.005))
            SC.cube(self.stage, f"/World/Cell/BeltBeam{i}", (length, 0.07, 0.13), SC.ALU,
                    (cx, BELT_Y + sy * (BELT_W / 2 + 0.03), WORK_Z - 0.125))
            for k in range(6):
                x = BELT_X0 + 0.3 + k * (length - 0.6) / 5
                SC.cube(self.stage, f"/World/Cell/BeltLeg{i}{k}", (0.05, 0.05,
                        abs(FLOOR_Z - WORK_Z) - 0.19), SC.ALU,
                        (x, BELT_Y + sy * (BELT_W / 2 + 0.03), (FLOOR_Z + WORK_Z - 0.19) / 2))
                SC.cube(self.stage, f"/World/Cell/BeltFoot{i}{k}", (0.09, 0.09, 0.018), (0.32, 0.34, 0.38),
                        (x, BELT_Y + sy * (BELT_W / 2 + 0.03), FLOOR_Z + 0.009))
            for k in range(5):                                  # cross braces under the belt
                x = BELT_X0 + 0.45 + k * (length - 0.9) / 4
                SC.cube(self.stage, f"/World/Cell/BeltBrace{i}{k}", (0.04, BELT_W + 0.06, 0.04), SC.ALU,
                        (x, BELT_Y, WORK_Z - 0.20))
        self._rollers = []
        for k, x in enumerate((BELT_X0 + ROLLER_R, BELT_X1 - ROLLER_R)):   # end drums only
            _, rop = SC.cyl(self.stage, f"/World/Cell/Roller{k}", ROLLER_R, BELT_W, SC.STEEL,
                            (x, BELT_Y, WORK_Z - ROLLER_R), (90, 0, 0))
            self._rollers.append(rop)
        # seam lines: the belt's motion made visible
        self._seams = []
        self._seam_pitch = 0.26
        n_seam = int(length / self._seam_pitch) + 1
        for k in range(n_seam):
            _, _, tr = SC.cube(self.stage, f"/World/Cell/Seam{k}", (0.012, BELT_W - 0.04, 0.003),
                               (0.30, 0.32, 0.35), (BELT_X0 + k * self._seam_pitch, BELT_Y, WORK_Z + 0.002))
            self._seams.append(tr)
        self._seam_phase = 0.0

    def _screen(self):
        # Behind the robot and beyond the conveyor, raised so it clears the belt: standing it in
        # front of the cell put a large dark slab between the camera and everything else.
        self.screen = SC.TelemetryScreen(self.stage, "/World/Cell", self.spec.dof, FLOOR_Z,
                                         x=0.95, y=1.42, yaw=4.0, z_centre=1.26)
        self.controls = SC.ControlBox(self.stage, "/World/Cell", FLOOR_Z)
        SC.relight(self.stage, FLOOR_Z)

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

    def frame_camera(self, view=None):
        """Point the viewport at the cell. `view` is a name from pal_cell_geom.CAM_VIEWS and
        defaults to $PAL_CAM, then to the isometric default.

        A bad name prints the choices and falls back rather than raising: losing the camera should
        not end a run that is otherwise fine."""
        import os
        name = view or os.environ.get("PAL_CAM") or CAM_DEFAULT
        try:
            eye, target = camera(name)
        except ValueError as e:
            print(f"[cell] {e}; using {CAM_DEFAULT}", flush=True)
            name = CAM_DEFAULT
            eye, target = camera(name)
        try:
            from isaacsim.core.utils.viewports import set_camera_view
            set_camera_view(eye=np.array(eye), target=np.array(target))
            print(f"[cell] camera '{name}': eye {np.round(eye, 2).tolist()} "
                  f"-> target {list(target)}", flush=True)
        except Exception as e:
            print(f"[cell] camera not set: {e!r}", flush=True)

    # -- per step ------------------------------------------------------------------------------
    def step(self, dt, tau_frac=None, visual=True):
        """Carry whatever is on the belt; when `visual`, also spin the rollers and move the bars.

        The split matters for speed. Driving the belt is physics and belongs on every step, but the
        rollers and the torque bars are USD attribute writes, and USD writes are expensive: 14
        rollers plus one bar per joint on every step at 250 Hz is 5000 writes a second, which was
        enough on its own to hold the cell to half of real time (measured 2026-11-08). They are
        only ever looked at, so they are refreshed on rendered steps instead."""
        for b in self.boxes:
            if self.tool is not None and self.tool.held == b.path:
                continue
            if b.on_belt():
                v = np.asarray(b.view.get_velocities()).reshape(6).copy()
                v[0], v[1] = BELT_SPEED, 0.0           # drive along the belt, hold the line
                b.view.set_velocities(v.reshape(1, 6).astype(np.float32))
        self._roller_angle = (self._roller_angle + np.degrees(BELT_SPEED / ROLLER_R) * dt) % 360.0
        if not visual:
            return
        if self._rollers:
            from pxr import Gf
            for rop in self._rollers:
                rop.Set(Gf.Vec3f(90.0, float(self._roller_angle), 0.0))
        if tau_frac is not None:
            self._t_vis += dt
            self.set_torque_bars(tau_frac, self._t_vis)

    def set_torque_bars(self, frac, t=0.0):
        """`frac` is |tau| / tau_limit per joint. Also feeds the screen's scrolling traces."""
        frac = np.asarray(frac, float)
        on_belt = sum(1 for b in self.boxes if b.on_belt())
        self.screen.update(frac, on_belt / max(len(self.boxes), 1), t)
        held = self.tool is not None and self.tool.held is not None
        self.controls.update(True, held, float(frac.max()) if frac.size else 0.0, t)

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
