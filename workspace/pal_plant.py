"""The one place the plant is built.

"Plant" is control-engineering for the thing being controlled: here, the simulated arm with its
masses, friction and motors. It stands in for the real robot; the analysis side's Pinocchio model
is the separate prediction it is compared against.

Every capture / sweep script used to carry its own ~60-line copy of "build the arm, author
the URDF inertials, set friction", and the copies drifted: that is how an invalid rate sweep
(a World rebuilt in one process, physics_dt silently ignored) got into the docs. Scripts now
call build_plant() once per process.

Import this only AFTER SimulationApp has started (it imports Isaac modules).

Order matters and is deliberate: every tensor-API write happens AFTER the final world.reset(),
because reset discards them. Every write is read back and asserted.

FRICTION (fixed 2026-11-03, docs/REVIEW.md F1). The old call, set_friction_coefficients, takes a
DIMENSIONLESS PhysX coefficient; passing Gaz's N*m values produced ~38 N*m of friction on joint 2.
Friction here goes through the physics view's set_dof_friction_properties, which takes a static
effort and a dynamic effort in N*m and a viscous coefficient in N*m*s/rad. Verified by
pal_selftest.py: 1.4 N*m injected shows up as 1.388 N*m.

MASS MISMATCH. The proposal requires the twin's model to differ from the plant ("otherwise
detection would succeed trivially"). `mass_scale` scales the mass and inertia of the DRIVEN links
only — the ones an arm joint moves, inferred from the URDF — and not the fixed base or the tool,
which are part of the robot rather than the payload. The analysis side's model keeps the URDF
values, so the mismatch is known and one-directional.
"""
import xml.etree.ElementTree as ET

import numpy as np
from pxr import PhysxSchema
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation

import pal_config as cfg
import pal_robot
from pal_kinematics import Urdf, rpy

SPEC = cfg.ROBOT                  # the arm this process is running (pal_robot.active())
ARM = SPEC.dof                    # the arm's driven joints; 6 on the UR10e
# Where a carried load sits, from the robot spec, so the plant and the analysis side's model
# agree on the lever arm. Hard-coding it once cost the estimator the whole lever: the load went
# on the flange while the model had it half a box beyond the cups.
PAYLOAD_TCP_Z = float(SPEC.tool_offset[2])
DEFAULT_MASS_SCALE = cfg.LINK_MASS_SCALE   # deliberate plant-vs-model mass mismatch, driven links only
# The links mass_scale applies to: the ones an arm joint drives, inferred from the URDF rather
# than listed, so it is right for any arm (on the UR10e, shoulder_link..wrist_3_link).
ARM_LINKS = Urdf(SPEC.urdf_file(), tool_link=SPEC.tool_frame).moving_links


def _urdf_inertials(path):
    """Mass, COM and the full 3x3 inertia tensor per link, with the tensor rotated into link axes.

    A URDF may express the inertia in a frame rotated against the link (`<inertial><origin rpy=...>`).
    The UR10e does this on three links with genuinely off-diagonal tensors, where ignoring the
    rotation would be the same class of error as the diagonal-only inertia bug of 2026-11-03
    (REVIEW F11). Cross-checked against Pinocchio on the same URDF to 1e-15 kg*m^2
    (tests/test_ur10e.py)."""
    out = {}
    for l in ET.parse(path).getroot().findall("link"):
        it = l.find("inertial")
        if it is None:
            continue
        m, o, I = it.find("mass"), it.find("origin"), it.find("inertia")
        T = np.array([[float(I.get("ixx")), float(I.get("ixy")), float(I.get("ixz"))],
                      [float(I.get("ixy")), float(I.get("iyy")), float(I.get("iyz"))],
                      [float(I.get("ixz")), float(I.get("iyz")), float(I.get("izz"))]])
        if o is not None and o.get("rpy"):
            R = rpy(*[float(x) for x in o.get("rpy").split()])
            T = R @ T @ R.T
        out[l.get("name")] = {
            "mass": float(m.get("value")),
            "com": np.array([float(x) for x in o.get("xyz").split()])
                   if o is not None and o.get("xyz") else np.zeros(3),
            "tensor": T}
    return out


def base_friction():
    """(dof, 3) static N*m, dynamic N*m, viscous N*m*s/rad: the spec's friction x the plant scale.

    The scale is the deliberate plant-vs-model mismatch; the model side uses the same spec values
    unscaled and through a different law. Asked through the spec rather than rebuilt here so the
    two sides cannot drift apart."""
    return SPEC.friction_3col(cfg.PLANT_FRICTION_SCALE)


def _resolve_usd(spec):
    """The robot's USD on the asset server. Asset paths move between Isaac releases, so the spec
    carries fallbacks and the first one that actually exists wins."""
    root = get_assets_root_path()
    try:
        import omni.client
        for sub in spec.usd_candidates:
            res, _ = omni.client.stat(root + sub)
            if res == omni.client.Result.OK:
                if sub != spec.usd_subpath:
                    print(f"[plant] {spec.usd_subpath} not on this asset server; using {sub}", flush=True)
                return root + sub
        print(f"[plant] none of {list(spec.usd_candidates)} found on {root}; trying the first anyway", flush=True)
    except Exception as exc:                        # omni.client missing or offline: fall through
        print(f"[plant] could not probe the asset server ({exc!r}); using the declared path", flush=True)
    return root + spec.usd_subpath


def dress_scene(world, with_table=True, floor=True):
    """Lights, a ground plane and the camera, so a window shows something. Does not touch the arm's physics.

    The ground sits at cfg.GROUND_PLANE_Z (below the base), so no pose in any suite touches it; physics is identical with and without
    it (checked with tools/compare_runs.py). `floor=False` leaves the ground out, for the conveyor cell, which lays its own."""
    import omni.usd
    from pxr import Gf, UsdGeom, UsdLux
    stage = omni.usd.get_context().get_stage()
    L = cfg.LIGHTS
    UsdLux.DomeLight.Define(stage, "/World/Lights/DomeLight").CreateIntensityAttr(L["dome"]["intensity"])
    key = UsdLux.DistantLight.Define(stage, "/World/Lights/KeyLight")
    UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(*L["key"]["rotation"]))
    key.CreateIntensityAttr(L["key"]["intensity"]); key.CreateColorAttr(Gf.Vec3f(*L["key"]["color"]))
    if floor:
        world.scene.add_default_ground_plane(z_position=cfg.GROUND_PLANE_Z)
    if not with_table:
        return
    table = UsdGeom.Cube.Define(stage, "/World/Table")                 # visual only: no collision, nothing touches it
    table.CreateSizeAttr(1.0)
    UsdGeom.Xformable(table).AddTranslateOp().Set(Gf.Vec3d(*cfg.TABLE_POSITION))
    UsdGeom.Xformable(table).AddScaleOp().Set(Gf.Vec3f(*cfg.TABLE_SCALE))
    table.CreateDisplayColorAttr([Gf.Vec3f(0.30, 0.33, 0.40)])


def frame_camera():
    """Point the viewport at the arm (the old scene's isometric three-quarter overview)."""
    try:
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=np.array(cfg.CAM_POSITION), target=np.array(cfg.CAM_LOOK_AT))
    except Exception as e:                                   # a missing viewport (headless) is not an error
        print(f"[view] camera not set: {e!r}", flush=True)


class Plant:
    def __init__(self, hz, kp=None, kd=None, scene=False, cell=False):
        """`cell` builds the industrial conveyor cell (conveyor, boxes, background) and mounts the
        suction tool; without it the arm stands bare, which is what the sweeps and selftests want.

        kp/kd are per-radian joint stiffness and damping in N*m/rad and N*m*s/rad. They default to
        the active drive mode's, which scales with the arm's effort limits — a fixed 400 N*m/rad is
        a stiff drive on an 87 N*m joint and a soft one on a 330 N*m joint."""
        kp = cfg.DEFAULT_KP if kp is None else kp
        kd = cfg.DEFAULT_KD if kd is None else kd
        self.hz, self.dt = hz, 1.0 / hz
        # A rendered step (world.step(render=True)) advances the simulation by rendering_dt. With the usual 1/30 s that is 33 physics
        # steps inside ONE call, so windowed telemetry was time-warped (33x the motion on every rendered row; found 2026-11-04).
        # In a window the rendering interval equals the physics step, so a rendered step is exactly one physics step. Headless
        # never renders, and keeps its original value so existing runs stay bit-identical.
        self.world = World(stage_units_in_meters=1.0, physics_dt=self.dt, rendering_dt=self.dt if scene else 1 / 30.0)
        pc = self.world.get_physics_context()
        pc.set_solver_type(cfg.SOLVER_TYPE); pc.enable_ccd(cfg.ENABLE_CCD)
        pc.enable_stablization(cfg.ENABLE_STABILIZATION); pc.set_gravity(cfg.GRAVITY)
        sp = PhysxSchema.PhysxSceneAPI.Apply(pc.get_current_physics_scene_prim())
        sp.CreateMinPositionIterationCountAttr(cfg.POSITION_ITERATIONS)
        sp.CreateMinVelocityIterationCountAttr(cfg.VELOCITY_ITERATIONS)
        add_reference_to_stage(usd_path=_resolve_usd(SPEC), prim_path=cfg.ROBOT_PRIM)
        self.spec = SPEC
        self.cell = None
        self.tool = None
        if cell:
            # The industrial cell (conveyor, cardboard boxes, background) and a suction tool, the
            # UR10e layout. Built before the Articulation exists so the tool's prims are part of it.
            from pal_cell import ConveyorCell
            if scene:
                dress_scene(self.world, with_table=False, floor=False)
            self.cell = ConveyorCell(self.world, SPEC)
            # The suction tool is geometry parented to the wrist link, so it must exist before
            # world.reset() builds the articulation; mounting it afterwards may not take.
            self.tool = self.cell.mount_tool(cfg.ROBOT_PRIM)
        elif scene:
            dress_scene(self.world)
        self.art = Articulation(cfg.ROBOT_PRIM)
        self.world.reset(); self.art.initialize()
        self.idx = np.arange(ARM)
        if self.cell is not None:
            self.cell.initialize()
        self._check_joint_order()
        self.pv = self.art._physics_view
        self.bodies = list(self.art.body_names)
        # The rigid body a tool-frame force or payload acts on. The spec's tool_frame is a URDF
        # FRAME, and a frame at the end of a fixed joint is not necessarily a body in the
        # articulation — on the UR10e `tool0` is not, and the body list ends at `wrist_3_link`.
        # Resolved once, by name, with the last body as the fallback: asking for a frame that is
        # not a body used to raise "'tool0' is not in list" from inside push(), on the first step
        # of every run.
        self.tool_body = self._resolve_tool_body()
        self.urdf = _urdf_inertials(SPEC.urdf_file())
        self.stock_masses = np.asarray(self.art.get_body_masses()).reshape(-1)
        # Gains go in per DEGREE: the tensor API is per radian but the USD drive attribute the
        # write lands on is per degree, and the two differ by exactly 57.3 (cfg section 3).
        self.kp = np.broadcast_to(np.asarray(kp, float), (ARM,)).copy()
        self.kd = np.broadcast_to(np.asarray(kd, float), (ARM,)).copy()
        self.art.set_gains(kps=(self.kp * cfg.DEG_PER_RAD).reshape(1, ARM).astype(np.float32),
                           kds=(self.kd * cfg.DEG_PER_RAD).reshape(1, ARM).astype(np.float32),
                           joint_indices=self.idx)
        self.art.set_armatures(cfg.ARMATURE.reshape(1, ARM).astype(np.float32), joint_indices=self.idx)
        self.base_friction = base_friction()
        self.payload_kg, self.mass_scale = 0.0, 1.0
        self.set_inertials(0.0, DEFAULT_MASS_SCALE)
        self.set_friction(self.base_friction)

    def _check_joint_order(self):
        """The articulation's first `dof` joints must be the spec's arm joints, in that order.

        Everything downstream indexes by position: gains, armature, friction, torques and every
        logged row. Isaac takes its order from the USD asset, which is not obliged to match the
        URDF the model reads, and a silent permutation would put joint 3's torque under joint 5's
        name and poison every residual. Cheap to check, so it is checked."""
        names = None
        for attr in ("dof_names", "joint_names"):
            names = getattr(self.art, attr, None)
            if names is not None:
                break
        if names is None:
            print("[plant] articulation exposes no joint names; order NOT verified", flush=True)
            return
        names = [str(n) for n in names]
        want = list(SPEC.joint_names)
        got = names[:len(want)]
        if got != want:
            raise RuntimeError(
                "the simulator's joint order does not match pal_robot's.\n"
                f"  expected: {want}\n  got:      {got}\n"
                f"  full articulation order: {names}\n"
                "Fix the order in pal_robot before running: every index downstream depends on it.")
        print(f"[plant] joint order verified: {got}", flush=True)

    def _resolve_tool_body(self):
        """The articulation body the tool frame sits on, or the last body if it is not one."""
        want = SPEC.tool_frame
        if want in self.bodies:
            return want
        for cand in (SPEC.joint_names[-1].replace("_joint", "_link"), self.bodies[-1]):
            if cand in self.bodies:
                print(f"[plant] tool frame {want!r} is not an articulation body; "
                      f"forces and payloads act on {cand!r} (bodies: {self.bodies})", flush=True)
                return cand
        raise RuntimeError(f"no usable tool body; articulation has {self.bodies}")

    # -- inertials ------------------------------------------------------------------------
    def set_inertials(self, payload_kg, mass_scale):
        """Author masses/COMs/inertias from the URDF, scaled arm links, payload on the hand.

        Callable between runs (no reset needed); read back and asserted every time."""
        nb = len(self.bodies)
        masses, coms, tensors = [], [], []
        for i, b in enumerate(self.bodies):
            if b not in self.urdf:
                masses.append(float(self.stock_masses[i])); coms.append(np.zeros(3))
                tensors.append(np.eye(3) * 1e-3); continue
            u = self.urdf[b]
            m0, c0, T0 = u["mass"], u["com"], u["tensor"]
            if b in ARM_LINKS:
                m0, T0 = m0 * mass_scale, T0 * mass_scale
            if b == self.tool_body and payload_kg > 0:
                # tool link + point mass at the TCP: combined mass and COM, full tensor by the
                # parallel-axis theorem. A point mass is the right model here because this path
                # exists for a payload the arm is TOLD it has; a really gripped box is a separate
                # rigid body and reaches the joints through the suction joint instead.
                cp = np.array([0.0, 0.0, PAYLOAD_TCP_Z])
                m = m0 + payload_kg
                c = (m0 * c0 + payload_kg * cp) / m
                dh, dp = c0 - c, cp - c
                T = (T0 + m0 * (np.dot(dh, dh) * np.eye(3) - np.outer(dh, dh))
                     + payload_kg * (np.dot(dp, dp) * np.eye(3) - np.outer(dp, dp)))
                m0, c0, T0 = m, c, T
            masses.append(m0); coms.append(c0); tensors.append(T0)
        masses = np.array(masses, np.float32); coms = np.array(coms, np.float32)
        quats = np.tile(np.array([1.0, 0, 0, 0], np.float32), (nb, 1))
        self.art.set_body_masses(masses.reshape(1, nb))
        self.art.set_body_coms(coms.reshape(1, nb, 3), quats.reshape(1, nb, 4))
        # set_body_inertias takes the FULL 3x3 tensor (9 numbers) in the link frame; it rejects the
        # 3-number form. Before 2026-11-03 only the diagonal was written (off-diagonal terms dropped,
        # up to 0.008 kg*m^2 on M). Verified end to end by pal_selftest.py T5 against an independent
        # numpy mass matrix (pal_inertia_convention.py tried the alternatives: a principal-axes
        # quaternion is NOT how this API works).
        self.art.set_body_inertias(np.array(tensors, np.float32).reshape(1, nb, 9))
        back = np.asarray(self.art.get_body_masses()).reshape(nb)
        assert np.allclose(back, masses, atol=1e-4), "mass write did not take"
        self.payload_kg, self.mass_scale = payload_kg, mass_scale

    # -- friction -------------------------------------------------------------------------
    def set_friction(self, props):
        """Write (dof, 3) [static N*m, dynamic N*m, viscous N*m*s/rad]; any extra dofs keep zero.

        This goes through the physics view's set_dof_friction_properties, which takes efforts in
        N*m. The older set_friction_coefficients takes a DIMENSIONLESS PhysX coefficient, and
        passing N*m values to it produced ~38 N*m of friction on one joint (REVIEW F1). Read back
        and asserted every time, because that bug was silent."""
        props = np.asarray(props, np.float64)
        full = np.asarray(self.pv.get_dof_friction_properties()).copy()      # (1, ndof, 3)
        full[0, :ARM, :] = props
        self.pv.set_dof_friction_properties(full.astype(np.float32), np.array([0], dtype=np.int32))
        back = np.asarray(self.pv.get_dof_friction_properties())[0, :ARM, :]
        assert np.allclose(back, props, atol=1e-5), "friction write did not take"
        self.friction = props.copy()

    def to_effort_mode(self):
        """Drive off (kp = kd = 0) and the arm joints switched to explicit torque commands."""
        self.art.set_gains(kps=np.zeros((1, ARM), np.float32), kds=np.zeros((1, ARM), np.float32),
                           joint_indices=self.idx)
        self.art.switch_control_mode("effort", joint_indices=self.idx)

    # -- external force (validation push) ---------------------------------------------------
    def push(self, force_xyz, link_name=None):
        """Apply a world-frame force (N) at the origin of `link_name` (default: the tool); zero clears it."""
        link_name = link_name or self.tool_body
        if link_name not in self.bodies:
            raise ValueError(f"{link_name!r} is not an articulation body; have {self.bodies}")
        nb = len(self.bodies)
        f = np.zeros((1, nb, 3), np.float32); f[0, self.bodies.index(link_name)] = force_xyz
        t = np.zeros((1, nb, 3), np.float32)
        self.pv.apply_forces_and_torques_at_position(f, t, None, np.array([0], dtype=np.int32), True)


def build_plant(hz, payload_kg=0.0, mass_scale=DEFAULT_MASS_SCALE, kp=None, kd=None,
                scene=False, cell=False):
    plant = Plant(hz, kp=kp, kd=kd, scene=scene, cell=cell)
    if scene:
        frame_camera()
    plant.set_inertials(payload_kg, mass_scale)
    print(f"[plant] {SPEC.name} ({ARM} joints) | {hz:.0f} Hz | payload {payload_kg:g} kg | arm link mass scale {mass_scale:g} | "
          f"friction (N*m) static {plant.friction[:, 0].round(3).tolist()} | readbacks ok", flush=True)
    return plant
