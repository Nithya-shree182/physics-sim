"""A vacuum end-effector for the UR10e: a four-cup suction array, and the physics of picking with it.

Why suction and not fingers. The cell this arm works in moves cardboard boxes, and in a real
warehouse those are moved by vacuum, not by a parallel gripper. It also removes a measurement
problem the Panda had: a finger grasp carries part of the box's weight through finger-contact
friction, a load path whose stiffness depends on the solver timestep, which is the leading
suspect for the unexplained 2 kg payload error at 1000 Hz (STATUS problem 11/18). A suction grip
is modelled here as a rigid attachment, so the whole of the box's weight and inertia reaches the
wrist and shows up in the joint torques, which is exactly what the residual is built to read.

Geometry. `tool0` and `wrist_3_link` turn out to be the same frame on this robot (the flange and
tool-frame rotations cancel), so the tool is a child of `wrist_3_link` and its cups look along that
link's +z. At the home pose that is straight down. The cup contact plane sits CUP_PLANE_Z metres
out, which is the number the IK has to aim with.

Grip and release. One fixed joint per box is created up front, before `world.reset()`, and left
disabled; gripping writes the joint's frames from the measured relative pose and enables it, and
releasing disables it again. Creating or deleting a joint while the simulation runs changes the
stage's topology, which tears down the articulation's physics view and kills the run on the next
read of a joint torque (seen 2026-11-08). Enabling an existing joint is only a property write, so
it does not. A real cup needs a moment to seal and bleeds air when it lets go; `LATCH_S` and
`BLEED_S` reproduce that as a delay, so the pick has the small timing imperfection a real cell has
and the trajectory cannot cheat by releasing instantly.
"""
from __future__ import annotations

import numpy as np

# ── geometry, metres, measured along the wrist's +z ───────────────────────────────────────────
PLATE_R, PLATE_H = 0.055, 0.020        # the mounting flange plate
ARM_LEN, ARM_W, ARM_H = 0.230, 0.030, 0.018    # the two cross bars the cups hang from
FRAME_Z = 0.030                        # centre height of the cross
CUP_R, CUP_H = 0.030, 0.040            # a single cup
CUP_SPAN = 0.085                       # cup centre offset from the tool axis, along x and y
CUP_PLANE_Z = 0.110                    # where the cup faces are: what the IK aims at
GRIP_LATERAL = 0.055                   # m, how far off centre the cups may still seal
GRIP_VERTICAL = 0.030                  # m, how far above the face they may still seal
GRIP_RANGE = GRIP_LATERAL              # kept for callers that quote one number
LATCH_S = 0.12                         # s, vacuum build-up before the box is actually held
BLEED_S = 0.08                         # s, air bleed before the box is actually let go

ORANGE = (0.95, 0.62, 0.11)
GREEN = (0.16, 0.55, 0.30)
STEEL = (0.62, 0.65, 0.70)
DARK = (0.18, 0.19, 0.22)


def _cube(stage, path, size, color, translate=(0, 0, 0), rotate=None):
    from pxr import Gf, UsdGeom
    g = UsdGeom.Cube.Define(stage, path)
    g.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(g)
    x.AddTranslateOp().Set(Gf.Vec3d(*translate))
    if rotate:
        x.AddRotateXYZOp().Set(Gf.Vec3f(*rotate))
    x.AddScaleOp().Set(Gf.Vec3f(*size))
    g.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return g


def _cyl(stage, path, radius, height, color, translate=(0, 0, 0), rotate=None):
    from pxr import Gf, UsdGeom
    g = UsdGeom.Cylinder.Define(stage, path)
    g.CreateRadiusAttr(radius); g.CreateHeightAttr(height); g.CreateAxisAttr("Z")
    x = UsdGeom.Xformable(g)
    x.AddTranslateOp().Set(Gf.Vec3d(*translate))
    if rotate:
        x.AddRotateXYZOp().Set(Gf.Vec3f(*rotate))
    g.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return g


class SuctionTool:
    """The vacuum tool: its geometry, and the one box it may be holding."""

    def __init__(self, stage, robot_prim: str, box_paths=(), wrist_link: str = "wrist_3_link"):
        self.stage = stage
        self.wrist_path = f"{robot_prim}/{wrist_link}"
        if not stage.GetPrimAtPath(self.wrist_path).IsValid():
            # Isaac's own asset may nest the links; find the link by name anywhere under the robot.
            found = [p.GetPath().pathString for p in stage.Traverse()
                     if p.GetPath().pathString.startswith(robot_prim) and p.GetName() == wrist_link]
            if not found:
                raise RuntimeError(f"no link named {wrist_link!r} under {robot_prim}; "
                                   f"cannot mount the suction tool")
            self.wrist_path = found[0]
        self.root = f"{self.wrist_path}/SuctionTool"
        self._build()
        self._wrist_view = None
        self.held = None            # the prim path of the box currently gripped
        self.on = False             # is the vacuum commanded on
        self._t_on = None           # when it was switched on, for LATCH_S
        self._t_off = None          # when it was switched off, for BLEED_S
        self._missed = False        # so a failed seal is reported once, not every step
        self.joints = {}            # box prim path -> its (disabled) fixed joint
        for i, bp in enumerate(box_paths):
            self.joints[bp] = self._make_joint(i, bp)

    # -- geometry ------------------------------------------------------------------------------
    def _build(self):
        from pxr import UsdGeom
        UsdGeom.Xform.Define(self.stage, self.root)
        _cyl(self.stage, f"{self.root}/Plate", PLATE_R, PLATE_H, STEEL, (0, 0, PLATE_H / 2))
        for i, rot in enumerate(((0, 0, 0), (0, 0, 90))):
            _cube(self.stage, f"{self.root}/Bar{i}", (ARM_LEN, ARM_W, ARM_H), DARK, (0, 0, FRAME_Z), rot)
        for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
            x, y = sx * CUP_SPAN, sy * CUP_SPAN
            _cyl(self.stage, f"{self.root}/Stem{i}", 0.008, 0.030, DARK, (x, y, FRAME_Z + 0.022))
            # the cup: an orange bellows with a green sealing lip, as on the photo
            _cyl(self.stage, f"{self.root}/Cup{i}", CUP_R, CUP_H, ORANGE, (x, y, CUP_PLANE_Z - CUP_H / 2 - 0.006))
            _cyl(self.stage, f"{self.root}/Lip{i}", CUP_R * 0.92, 0.012, GREEN, (x, y, CUP_PLANE_Z - 0.006))
        # The tool is visual only: it must not collide with the boxes it is picking up, or the cups
        # would push them away before the vacuum can take hold. The grip is the fixed joint below.

    # -- state ---------------------------------------------------------------------------------
    def set_vacuum(self, on: bool, t: float):
        if on and not self.on:
            self._t_on = t
            self._missed = False
        if (not on) and self.on:
            self._t_off = t
        self.on = bool(on)

    def _wrist(self):
        """The wrist link as a rigid body. Made on first use: the view needs a running simulation."""
        if getattr(self, "_wrist_view", None) is None:
            from isaacsim.core.prims import RigidPrim
            self._wrist_view = RigidPrim(self.wrist_path)
            self._wrist_view.initialize()
        return self._wrist_view

    def cup_pose(self):
        """World position and rotation of the cup contact plane's centre."""
        pos, quat = self._wrist().get_world_poses()
        p = np.asarray(pos).reshape(3)
        R = _quat_to_R(np.asarray(quat).reshape(4))
        return p + R @ np.array([0.0, 0.0, CUP_PLANE_Z]), R

    def update(self, t: float, boxes) -> str | None:
        """Called every physics step. Seals onto a box under the cups, or lets the held one go.
        Returns "grip" or "release" on the step the state changed, else None."""
        if self.on and self.held is None and self._t_on is not None and t - self._t_on >= LATCH_S:
            p, R = self.cup_pose()
            target, best = None, None
            dists = []
            for b in boxes:
                top = b.top_centre()
                # A cup array seals if it is over the face and close to it. Judging that as one
                # distance made the tolerance depend on how the error split between sideways and
                # down: a 32 mm lateral miss and a 32 mm hover are very different things, and the
                # first still seals. Measured 2026-11-08: the arm is still converging when the
                # vacuum latches, so lateral error at that moment runs to about 32 mm.
                lat = float(np.linalg.norm((p - top)[:2]))
                vert = float(abs(p[2] - top[2]))
                dists.append((b.name, round(lat, 4), round(vert, 4)))
                if lat <= GRIP_LATERAL and vert <= GRIP_VERTICAL and (best is None or lat < best):
                    target, best = b, lat
            if target is None and not self._missed:
                self._missed = True
                print(f"[suction] t={t:.2f}s no seal: cups at {np.round(p, 3).tolist()}, need lateral "
                      f"<={GRIP_LATERAL * 1000:.0f} mm and vertical <={GRIP_VERTICAL * 1000:.0f} mm; "
                      f"(box, lateral, vertical) = {dists}", flush=True)
            if target is not None:
                self._attach(target.path)
                self.held = target.path
                return "grip"
        if (not self.on) and self.held is not None and self._t_off is not None and t - self._t_off >= BLEED_S:
            self._detach()
            self.held = None
            return "release"
        return None

    # -- the grip itself -----------------------------------------------------------------------
    def _make_joint(self, i: int, box_path: str):
        """One wrist-to-box fixed joint, born disabled. Built before the simulation starts."""
        from pxr import Gf, UsdPhysics
        j = UsdPhysics.FixedJoint.Define(self.stage, f"{self.root}/GripJoint{i}")
        j.CreateBody0Rel().SetTargets([self.wrist_path])
        j.CreateBody1Rel().SetTargets([box_path])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, CUP_PLANE_Z))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateJointEnabledAttr().Set(False)
        return j

    def _attach(self, box_path: str):
        """Seal onto a box: point the joint's wrist-side frame at wherever the box actually is, so
        it is held where it sits rather than snapped into place, then switch the joint on."""
        from pxr import Gf
        from isaacsim.core.prims import RigidPrim
        j = self.joints.get(box_path)
        if j is None:
            raise RuntimeError(f"no grip joint was made for {box_path}; pass every box to SuctionTool")
        bv = RigidPrim(box_path); bv.initialize()
        bp, bq = bv.get_world_poses()
        bp = np.asarray(bp).reshape(3); bR = _quat_to_R(np.asarray(bq).reshape(4))
        wp, wq = self._wrist().get_world_poses()
        wp = np.asarray(wp).reshape(3); wR = _quat_to_R(np.asarray(wq).reshape(4))
        j.GetLocalPos0Attr().Set(Gf.Vec3f(*(wR.T @ (bp - wp)).astype(float)))   # the box, in wrist coordinates
        j.GetLocalRot0Attr().Set(_R_to_quatf(wR.T @ bR))
        j.GetJointEnabledAttr().Set(True)

    def _detach(self):
        j = self.joints.get(self.held)
        if j is not None:
            j.GetJointEnabledAttr().Set(False)


def _quat_to_R(q):
    """(w, x, y, z) -> 3x3, the convention Isaac's core prims return."""
    w, x, y, z = [float(v) for v in q]
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _R_to_quatf(R):
    from pxr import Gf
    tr = float(np.trace(R))
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return Gf.Quatf(float(w), float(x), float(y), float(z))
