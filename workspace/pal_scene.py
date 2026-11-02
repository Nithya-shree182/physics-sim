#!/usr/bin/env python3
"""Physics Attestation Layer — scene builder and physics audit.

Builds the Isaac Sim scene this project runs on: the isometric overview camera, the studio
lighting and the dark-blue grid backdrop carried over from the Physyk AI project, a Franka
Panda configured for realistic dynamics rather than for a convincing-looking pick, and the
work table / tray geometry those calibrated constants belong to.

Then it audits itself. Nothing in the proposal's objective O2 can be trusted until it is
known (a) which decomposed-dynamics quantities this Isaac Sim build will actually hand back,
and (b) whether the arm, standing still at a known pose, reports a measured torque that
matches its own gravity term. Both are checked here and written to a JSON report, because
every later result is a measurement of the difference between the model and the plant, and
a configuration error is indistinguishable from a physical effect.

Run:
    ./start_sim_docker.sh                   # headless, writes the audit report and a render
    HEADLESS=0 ./start_sim_docker.sh        # with a window

Environment:
    PAL_PHYSICS_HZ      override the 400 Hz physics rate
    PAL_DRIVE_MODE      position_stiff (default) | impedance | torque
    PAL_BACKGROUND      grid (default, the blue backdrop) | factory
    PAL_PAYLOAD_KG      payload cube mass; 0 for none
    PAL_OUT             output directory (default /workspace/out)
"""

import json
import os
import sys
import time

import numpy as np

# ── 1. Boot SimulationApp before any other Isaac import ───────────────────────────────
from isaacsim import SimulationApp

HEADLESS = os.environ.get("ISAAC_SIM_HEADLESS", "1") != "0"
simulation_app = SimulationApp({
    "headless": HEADLESS,
    "width": 1280,
    "height": 720,
    "renderer": "RaytracedLighting",
    "anti_aliasing": 3,
})

# ── 2. Imports that require a live SimulationApp ───────────────────────────────────────
from pxr import Usd, UsdGeom, UsdLux, UsdShade, Gf, PhysxSchema
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid, DynamicCuboid
from isaacsim.core.api.materials import PhysicsMaterial
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import ParallelGripper
import omni.replicator.core as rep

import pal_config as cfg

OUT_DIR = os.environ.get("PAL_OUT", "/workspace/out")
os.makedirs(OUT_DIR, exist_ok=True)

# Environment overrides. Kept to the few knobs a run actually varies; everything else
# lives in pal_config.py so that it stays auditable.
PHYSICS_HZ = float(os.environ.get("PAL_PHYSICS_HZ", cfg.PHYSICS_HZ))
PHYSICS_DT = 1.0 / PHYSICS_HZ
DRIVE_MODE = os.environ.get("PAL_DRIVE_MODE", cfg.DEFAULT_DRIVE_MODE)
BACKGROUND = os.environ.get("PAL_BACKGROUND", "grid").lower()
PAYLOAD_KG = float(os.environ.get("PAL_PAYLOAD_KG", cfg.PAYLOAD_DEFAULT_KG))

if DRIVE_MODE not in cfg.DRIVE_MODES:
    print(f"[FATAL] unknown PAL_DRIVE_MODE '{DRIVE_MODE}'; "
          f"expected one of {sorted(cfg.DRIVE_MODES)}", flush=True)
    simulation_app.close()
    sys.exit(2)


def banner(text):
    print("\n" + "═" * 78, flush=True)
    print(f"  {text}", flush=True)
    print("═" * 78, flush=True)


banner("Physics Attestation Layer — scene build")
print(f"  physics        {PHYSICS_HZ:.0f} Hz  (dt = {PHYSICS_DT*1000:.3f} ms)", flush=True)
print(f"  rendering      {cfg.RENDER_HZ:.0f} Hz  ({cfg.STEPS_PER_RENDER} physics steps/frame)", flush=True)
print(f"  drive mode     {DRIVE_MODE}  —  {cfg.DRIVE_MODES[DRIVE_MODE]['note']}", flush=True)
print(f"  backdrop       {BACKGROUND}", flush=True)
print(f"  payload        {PAYLOAD_KG:.3f} kg", flush=True)

# ── 3. World and PhysX scene ───────────────────────────────────────────────────────────
world = World(
    stage_units_in_meters=1.0,
    physics_dt=PHYSICS_DT,
    rendering_dt=cfg.RENDER_DT,
)
stage = world.stage

assets_root = get_assets_root_path()
if assets_root is None:
    print("[FATAL] Nucleus asset root could not be resolved.", flush=True)
    simulation_app.close()
    sys.exit(1)

# Solver configuration. Every one of these is a decision recorded in pal_config.py with
# its reason; the audit report below echoes what PhysX actually accepted, because a
# silently-rejected setting is the kind of thing that only shows up as a mysterious
# residual three objectives later.
physx_applied = {}
try:
    physics_context = world.get_physics_context()
    physics_context.set_solver_type(cfg.SOLVER_TYPE)
    physics_context.enable_ccd(cfg.ENABLE_CCD)
    physics_context.enable_stablization(cfg.ENABLE_STABILIZATION)
    physics_context.set_gravity(cfg.GRAVITY)

    scene_prim = physics_context.get_current_physics_scene_prim()
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
    physx_scene.CreateMinPositionIterationCountAttr(cfg.POSITION_ITERATIONS)
    physx_scene.CreateMinVelocityIterationCountAttr(cfg.VELOCITY_ITERATIONS)
    # Enhanced determinism costs a little performance and buys bit-identical replays of
    # the same run. For an attestation system whose whole output is a comparison between
    # two runs, that is worth paying for.
    physx_scene.CreateEnableEnhancedDeterminismAttr(True)

    physx_applied = {
        "solver": cfg.SOLVER_TYPE,
        "ccd": cfg.ENABLE_CCD,
        "stabilization": cfg.ENABLE_STABILIZATION,
        "position_iterations": cfg.POSITION_ITERATIONS,
        "velocity_iterations": cfg.VELOCITY_ITERATIONS,
        "gravity": cfg.GRAVITY,
        "enhanced_determinism": True,
    }
    print(f"[physx] {cfg.SOLVER_TYPE}, {cfg.POSITION_ITERATIONS}/{cfg.VELOCITY_ITERATIONS} iters, "
          f"CCD={cfg.ENABLE_CCD}, stabilization={cfg.ENABLE_STABILIZATION}, "
          f"gravity={cfg.GRAVITY}", flush=True)
except Exception as e:
    physx_applied = {"error": str(e)}
    print(f"[WARN] PhysX scene tuning failed, engine defaults in use: {e}", flush=True)

# ── 4. Physics materials ───────────────────────────────────────────────────────────────
grasp_material = PhysicsMaterial(
    prim_path="/World/Physics/GraspFrictionMaterial",
    static_friction=cfg.GRASP_STATIC_FRICTION,
    dynamic_friction=cfg.GRASP_DYNAMIC_FRICTION,
    restitution=cfg.RESTITUTION,
)
surface_material = PhysicsMaterial(
    prim_path="/World/Physics/SurfaceFrictionMaterial",
    static_friction=cfg.SURFACE_STATIC_FRICTION,
    dynamic_friction=cfg.SURFACE_DYNAMIC_FRICTION,
    restitution=cfg.RESTITUTION,
)

# ── 5. Lighting — the Physyk "studio" preset, reproduced exactly ───────────────────────
L = cfg.LIGHTS
dome_light = UsdLux.DomeLight.Define(stage, "/World/Lights/DomeLight")
dome_light.CreateIntensityAttr(L["dome"]["intensity"])

overhead_light = UsdLux.DiskLight.Define(stage, "/World/Lights/OverheadLight")
UsdGeom.Xformable(overhead_light).AddTranslateOp().Set(Gf.Vec3d(*L["overhead"]["position"]))
UsdGeom.Xformable(overhead_light).AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 0))
overhead_light.CreateRadiusAttr(L["overhead"]["radius"])
overhead_light.CreateIntensityAttr(L["overhead"]["intensity"])
overhead_light.CreateColorAttr(Gf.Vec3f(*L["overhead"]["color"]))

key_light = UsdLux.DistantLight.Define(stage, "/World/Lights/KeyLight")
UsdGeom.Xformable(key_light).AddRotateXYZOp().Set(Gf.Vec3f(*L["key"]["rotation"]))
key_light.CreateIntensityAttr(L["key"]["intensity"])
key_light.CreateColorAttr(Gf.Vec3f(*L["key"]["color"]))

fill_light = UsdLux.SphereLight.Define(stage, "/World/Lights/FillLight")
UsdGeom.Xformable(fill_light).AddTranslateOp().Set(Gf.Vec3d(*L["fill"]["position"]))
fill_light.CreateRadiusAttr(L["fill"]["radius"])
fill_light.CreateIntensityAttr(L["fill"]["intensity"])

print("[lighting] studio preset applied (dome/overhead/key/fill)", flush=True)

# ── 6. Ground plane and the blue backdrop ──────────────────────────────────────────────
# The ground plane sits below the table's own bottom face. In the Physyk project leaving it
# at its z=0 default put it coplanar with the table TOP, which z-fought and made the table
# read as sunk into the floor. The table top stays at Z=0.00 either way, so every
# calibrated height downstream is untouched.
world.scene.add_default_ground_plane(z_position=cfg.GROUND_PLANE_Z)


def retint_ground_plane(prim_path="/World/defaultGroundPlane", tint=cfg.GROUND_TINT):
    """Recolours the stock grid material to the dark blue this scene uses as its backdrop.

    add_default_ground_plane() loads a fixed USD environment whose light-blue grid colour
    is not exposed as a parameter, so it has to be set on the material's own shader input.
    diffuse_tint multiplies the baked grid texture rather than replacing it, which keeps
    the tile pattern and only shifts its hue. Defensive by design: a different Isaac Sim
    build may not expose that input, and a cosmetic tint is not worth failing a start-up
    over.
    """
    root = stage.GetPrimAtPath(prim_path)
    if not root.IsValid():
        print(f"[WARN] ground plane '{prim_path}' not found — no retint", flush=True)
        return 0
    tinted = 0
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdShade.Shader):
            continue
        shader = UsdShade.Shader(prim)
        for name in ("diffuse_tint", "diffuse_color_constant"):
            shader_input = shader.GetInput(name)
            if shader_input and shader_input.GetAttr().IsValid():
                shader_input.Set(Gf.Vec3f(*tint))
                tinted += 1
    print(f"[backdrop] grid ground retinted dark blue ({tinted} shader input(s))", flush=True)
    return tinted


if BACKGROUND == "factory":
    # The Physyk factory room, if it is ever wanted here. Visual only: no collision, no
    # physics, nothing that touches the arm or the residual.
    try:
        UsdGeom.Imageable(stage.GetPrimAtPath("/World/defaultGroundPlane")).MakeInvisible()
        sys.path.insert(0, "/physyk_workspace")
        import factory_background
        factory_background.build(stage)
    except Exception as e:
        print(f"[WARN] factory backdrop unavailable, falling back to grid: {e}", flush=True)
        retint_ground_plane()
else:
    retint_ground_plane()

# ── 7. Work table and tray ─────────────────────────────────────────────────────────────
world.scene.add(FixedCuboid(
    prim_path="/World/WorkTable", name="work_table",
    position=np.array(cfg.TABLE_POSITION), scale=np.array(cfg.TABLE_SCALE),
    color=np.array(cfg.TABLE_COLOR), physics_material=surface_material,
))
world.scene.add(FixedCuboid(
    prim_path="/World/TargetTray", name="target_tray",
    position=np.array(cfg.TRAY_POSITION), scale=np.array(cfg.TRAY_SCALE),
    color=np.array(cfg.TRAY_COLOR), physics_material=surface_material,
))

# ── 8. Payload ─────────────────────────────────────────────────────────────────────────
# One cube, mass configurable across the arm's real 0.1–3.0 kg range. Physyk's 0.05 kg
# cube is around 1.6 percent of the rated payload and under the residual noise floor;
# a payload estimator cannot converge on a signal that small.
payload_obj = None
if PAYLOAD_KG > 0:
    payload_obj = world.scene.add(DynamicCuboid(
        prim_path="/World/Payload", name="payload",
        position=np.array([0.50, 0.0, cfg.PAYLOAD_SIZE / 2.0]),
        scale=np.array([cfg.PAYLOAD_SIZE] * 3),
        color=np.array([0.92, 0.18, 0.18]),
        mass=PAYLOAD_KG,
        physics_material=grasp_material,
    ))
    try:
        collision_api = PhysxSchema.PhysxCollisionAPI.Apply(stage.GetPrimAtPath("/World/Payload"))
        collision_api.CreateContactOffsetAttr(cfg.CONTACT_OFFSET)
        collision_api.CreateRestOffsetAttr(cfg.REST_OFFSET)
    except Exception as e:
        print(f"[WARN] payload collision offsets not set: {e}", flush=True)

# ── 9. Franka Panda ────────────────────────────────────────────────────────────────────
add_reference_to_stage(usd_path=assets_root + cfg.ROBOT_USD_SUBPATH, prim_path=cfg.ROBOT_PRIM)

# The gripper variant must be chosen before initialize(). The stock variant leaves
# panda_finger_joint2 with kp=kd=effort=0; "AlternateFinger" is the one NVIDIA's own
# pick-place demo validates against.
gripper_variant_selected = None
try:
    variant_set = stage.GetPrimAtPath(cfg.ROBOT_PRIM).GetVariantSet("Gripper")
    available = variant_set.GetVariantNames()
    if cfg.GRIPPER_VARIANT in available:
        variant_set.SetVariantSelection(cfg.GRIPPER_VARIANT)
        gripper_variant_selected = cfg.GRIPPER_VARIANT
    print(f"[arm] gripper variants {available} → selected {gripper_variant_selected}", flush=True)
except Exception as e:
    print(f"[WARN] gripper variant not selected: {e}", flush=True)

gripper = ParallelGripper(
    end_effector_prim_path=cfg.END_EFFECTOR_PRIM,
    joint_prim_names=list(cfg.FINGER_JOINT_NAMES),
    joint_opened_positions=np.array([cfg.FINGER_OPEN] * 2),
    joint_closed_positions=np.array([cfg.FINGER_CLOSED] * 2),
    action_deltas=np.array([0.01, 0.01]),
)
robot = world.scene.add(SingleManipulator(
    prim_path=cfg.ROBOT_PRIM, name="franka_robot",
    end_effector_prim_path=cfg.END_EFFECTOR_PRIM, gripper=gripper,
))

wrist_light = UsdLux.SphereLight.Define(stage, "/World/Franka/panda_hand/WristLight")
wrist_light.CreateIntensityAttr(L["wrist"]["intensity"])
wrist_light.CreateRadiusAttr(L["wrist"]["radius"])
wrist_light.CreateColorAttr(Gf.Vec3f(*L["wrist"]["color"]))
UsdGeom.Xformable(wrist_light).AddTranslateOp().Set(Gf.Vec3d(*L["wrist"]["translate"]))

# ── 10. Cameras ────────────────────────────────────────────────────────────────────────
# The isometric three-quarter overview, reproduced exactly from Physyk: this framing was
# settled after several correction passes, and the dark-blue grid behind it is the backdrop.
scene_cam = rep.create.camera(
    position=cfg.CAM_POSITION, look_at=cfg.CAM_LOOK_AT, focal_length=cfg.CAM_FOCAL_LENGTH,
)
render_product = rep.create.render_product(scene_cam, cfg.CAM_RESOLUTION)
rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
rgb_annotator.attach([render_product])

wrist_cam = stage.DefinePrim("/World/Franka/panda_hand/WristCamera", "Camera")
wrist_cam.GetAttribute("focalLength").Set(cfg.WRIST_CAM_FOCAL_LENGTH)
wrist_cam.GetAttribute("horizontalAperture").Set(cfg.CAM_HORIZONTAL_APERTURE)
wrist_cam.GetAttribute("clippingRange").Set(Gf.Vec2f(*cfg.WRIST_CAM_CLIPPING))
wrist_xf = UsdGeom.Xformable(wrist_cam)
wrist_xf.AddTranslateOp().Set(Gf.Vec3d(*cfg.WRIST_CAM_TRANSLATE))
wrist_xf.AddRotateXYZOp().Set(Gf.Vec3f(*cfg.WRIST_CAM_ROTATE))

print(f"[camera] isometric overview at {cfg.CAM_POSITION} → {cfg.CAM_LOOK_AT}, "
      f"{cfg.CAM_FOCAL_LENGTH}mm, {cfg.CAM_RESOLUTION[0]}x{cfg.CAM_RESOLUTION[1]}", flush=True)

# ── 11. Initialise physics ─────────────────────────────────────────────────────────────
world.reset()
robot.initialize()

arm_dof = len(cfg.JOINT_NAMES)
articulation = robot._articulation_view     # the batched view; carries the dynamics API

home_full = np.concatenate([cfg.HOME_JOINTS, [cfg.FINGER_OPEN, cfg.FINGER_OPEN]])
robot.set_joint_positions(home_full)
for _ in range(30):
    world.step(render=False)

print(f"[arm] {robot.num_dof} DOFs: {robot.dof_names}", flush=True)

# Note for objective O3 / section 7.7: this build exposes switch_dof_control_mode(),
# set_effort_modes() and set_joint_efforts() on the articulation. That is the supported
# path into a real torque mode for the known-push validation — preferable to faking
# compliance by zeroing the position gains, which leaves the drive in position mode and
# still fights an applied force. The "torque" entry in DRIVE_MODES is the fallback.

# ── 12. Apply the realism configuration ────────────────────────────────────────────────
banner("Arm configuration")

stock = {}
applied = {}
try:
    kps, kds = articulation.get_gains()
    efforts = articulation.get_max_efforts()
    stock = {
        "kp": np.array(kps).flatten().tolist(),
        "kd": np.array(kds).flatten().tolist(),
        "max_effort": np.array(efforts).flatten().tolist(),
    }
    print(f"  stock kp          {np.round(np.array(kps).flatten(), 1)}", flush=True)
    print(f"  stock kd          {np.round(np.array(kds).flatten(), 1)}", flush=True)
    print(f"  stock max effort  {np.round(np.array(efforts).flatten(), 2)}", flush=True)
except Exception as e:
    print(f"[WARN] could not read stock gains: {e}", flush=True)

# 12a. Arm drive gains for the selected mode.
# Gains in pal_config are per RADIAN, which is what the tensor API takes. The USD
# attribute underneath is per DEGREE, and the two differ by 180/pi = 57.2958.
#
# Established by probing the live asset rather than reasoning about it, because the two
# conventions sit side by side on the same joint and reading the wrong one inverts the
# conclusion:
#     USD  drive:angular:physics:stiffness  =   400.0   (per degree)
#     articulation.get_gains()              = 22918.3   (per radian)
# So the stock asset's physical stiffness is 22918 N*m/rad. The 400 authored into the USD
# is almost certainly someone intending Isaac Lab's 400 N*m/rad and hitting exactly the
# per-degree trap NVIDIA's own tutorial warns about.
#
# Both conventions are therefore checked on every start: the tensor readback must match
# what was written, AND the USD attribute must be 57.3x smaller. Either check alone can be
# satisfied by a consistent 57x error.
mode = cfg.DRIVE_MODES[DRIVE_MODE]
arm_indices = np.arange(arm_dof)

# Read the asset's AUTHORED stiffness before touching anything. set_gains() writes PhysX
# runtime state and does not write back to USD, so this can only be read meaningfully here.
asset_usd_stiffness, asset_physical, asset_trap = None, None, None
try:
    _jp = stage.GetPrimAtPath(f"{cfg.ROBOT_PRIM}/panda_link0/panda_joint1")
    _attr = _jp.GetAttribute("drive:angular:physics:stiffness") if _jp.IsValid() else None
    if _attr and _attr.IsValid():
        asset_usd_stiffness = float(_attr.Get())              # per degree
        asset_physical = asset_usd_stiffness * cfg.DEG_PER_RAD  # per radian
        # A value that is a round number per degree, and 57x a known Isaac Lab gain per
        # radian, is the signature of someone writing a per-radian gain into a per-degree
        # field. Worth flagging loudly: the arm then simulates 57x stiffer than intended.
        asset_trap = bool(abs(asset_usd_stiffness - 400.0) < 1.0)
        print(f"  asset authored    USD {asset_usd_stiffness:.1f}/deg "
              f"= {asset_physical:.1f} N*m/rad physical", flush=True)
        if asset_trap:
            print(f"                    ← 400 written into a per-DEGREE field: the stock asset", flush=True)
            print(f"                      simulates 57x stiffer than Isaac Lab's intended 400.", flush=True)
except Exception as e:
    print(f"[WARN] could not read authored USD stiffness: {e}", flush=True)

try:
    kp_rad = mode["kp"].reshape(1, arm_dof).astype(np.float32)
    kd_rad = mode["kd"].reshape(1, arm_dof).astype(np.float32)
    articulation.set_gains(kps=kp_rad, kds=kd_rad, joint_indices=arm_indices)

    kp_back = np.array(articulation.get_gains()[0]).flatten()[:arm_dof]
    tensor_ratio = float(np.median(kp_back / np.maximum(kp_rad.flatten(), 1e-9)))
    tensor_ok = bool(abs(tensor_ratio - 1.0) < 0.01)

    applied["arm_kp_per_rad"] = mode["kp"].tolist()
    applied["arm_kd_per_rad"] = mode["kd"].tolist()
    applied["tensor_roundtrip_ratio"] = tensor_ratio
    applied["tensor_convention_ok"] = tensor_ok
    applied["asset_usd_stiffness_per_deg"] = asset_usd_stiffness
    applied["asset_physical_stiffness_per_rad"] = asset_physical
    applied["asset_57x_trap"] = asset_trap

    print(f"  drive mode        {DRIVE_MODE}  —  {mode['note']}", flush=True)
    print(f"    kp (per rad)    {np.round(mode['kp'], 1)}   ← tensor API, physical", flush=True)
    print(f"    kd (per rad)    {np.round(mode['kd'], 1)}", flush=True)
    print(f"    tensor readback {'OK' if tensor_ok else f'MISMATCH ratio {tensor_ratio:.4f}'}", flush=True)
    if not tensor_ok:
        print(f"    If the ratio is ~57 or ~1/57 the tensor API convention has changed;", flush=True)
        print(f"    re-probe before trusting any residual.", flush=True)
except Exception as e:
    print(f"[WARN] arm gains not applied: {e}", flush=True)

    print(f"  effort limits     {cfg.TAU_MAX} N*m  (datasheet)", flush=True)
except Exception as e:
    print(f"[WARN] effort limits not applied: {e}", flush=True)

# 12c. Finger drive. Only patched when a finger has no real drive, which is the case the
# variant switch is meant to fix — forcing gains on a finger NVIDIA already tuned is what
# caused overshoot in the Physyk project.
try:
    finger_indices = np.array([arm_dof, arm_dof + 1])
    kps_now = np.array(articulation.get_gains()[0]).flatten()[finger_indices]
    efforts_now = np.array(articulation.get_max_efforts()).flatten()[finger_indices]
    if np.any(kps_now <= 0.0) or np.any(efforts_now <= 0.0):
        articulation.set_gains(
            kps=np.array([[cfg.FINGER_KP, cfg.FINGER_KP]], dtype=np.float32),
            kds=np.array([[cfg.FINGER_KD, cfg.FINGER_KD]], dtype=np.float32),
            joint_indices=finger_indices,
        )
        articulation.set_max_efforts(
            np.array([[cfg.FINGER_MAX_EFFORT, cfg.FINGER_MAX_EFFORT]], dtype=np.float32),
            joint_indices=finger_indices,
        )
        applied["finger_patched"] = True
        print(f"  fingers           patched to kp={cfg.FINGER_KP} kd={cfg.FINGER_KD} "
              f"effort={cfg.FINGER_MAX_EFFORT}N", flush=True)
    else:
        applied["finger_patched"] = False
        print("  fingers           stock drive is real, left alone", flush=True)
except Exception as e:
    print(f"[WARN] finger drive check failed: {e}", flush=True)

# 12d. Joint friction and armature, written onto the USD drive/joint prims.
#
# This is the realism gap that matters most. Every revolute joint in the shipped URDF
# declares friction="0.0": a frictionless plant compared against a frictionless model
# produces a residual bounded only by solver noise, which would make every detection rate
# in section 9 optimistic and meaningless. The plant gets scaled friction so the twin is
# honestly mismatched, and so the friction fault of O5 perturbs a quantity that was
# already physical.
# The probe showed this build has no PhysxJointAxisAPI, so PhysX's three-part friction
# model (static Coulomb / dynamic Coulomb / viscous) is not reachable through USD here —
# PhysxJointAPI offers only the single scalar jointFriction. The articulation tensor API
# does expose set_friction_coefficients() and set_armatures(), which is both cleaner and
# the same path Isaac Lab uses, so friction goes through that.
#
# Consequence worth recording rather than hiding: this is ONE friction number per joint,
# not three. The O5 fault classifier therefore cannot separate a Coulomb-type fault
# (stiffening, resists breakaway) from a viscous-type one (bearing wear, grows with speed)
# on the strength of the plant model alone — both arrive as a change in the same scalar.
# If that separation turns out to matter, the viscous term has to be injected as an
# applied torque in the control loop instead, proportional to joint velocity.
#
# Directly observed on the stock asset, confirming the zero-friction finding a third time:
#     jointFriction = 0.0        physxJoint:armature = 0.0
friction_report, armature_applied = {}, []
scale = cfg.PLANT_FRICTION_SCALE
friction_values = cfg.FRICTION_COULOMB_STATIC * scale      # N*m, the scalar PhysX accepts

if cfg.APPLY_PLANT_FRICTION:
    try:
        articulation.set_friction_coefficients(
            friction_values.reshape(1, arm_dof).astype(np.float32),
            joint_indices=arm_indices)
        back = np.array(articulation.get_friction_coefficients()).flatten()[:arm_dof]
        friction_report = {
            "api": "articulation.set_friction_coefficients (tensor)",
            "values_nm": np.round(friction_values, 5).tolist(),
            "readback": np.round(back, 5).tolist(),
            "viscous_term": "NOT APPLIED — no PhysxJointAxisAPI on this build; "
                            "single scalar only, see note in pal_scene.py",
            "stock_was": 0.0,
        }
        applied["friction"] = friction_report
        print(f"  friction API      tensor set_friction_coefficients()", flush=True)
        print(f"  friction source   {cfg.FRICTION_SOURCE} (plant scale x{scale})", flush=True)
        print(f"    Coulomb N*m     {np.round(friction_values, 4)}", flush=True)
        print(f"    readback        {np.round(back, 4)}", flush=True)
        print(f"    viscous term    NOT APPLIED (no PhysxJointAxisAPI on this build)", flush=True)
        print(f"                    stock asset ships 0.0 — observed directly on the prim", flush=True)
    except Exception as e:
        print(f"[WARN] joint friction not applied: {e}", flush=True)

if cfg.APPLY_ARMATURE:
    try:
        articulation.set_armatures(
            cfg.ARMATURE.reshape(1, arm_dof).astype(np.float32), joint_indices=arm_indices)
        armature_applied = cfg.ARMATURE.tolist()
        applied["armature"] = armature_applied
        print(f"  armature          {cfg.ARMATURE[0]} kg*m^2/joint via tensor set_armatures() "
              f"(stock 0.0; Isaac Lab uses {cfg.ARMATURE_ISAACLAB[0]})", flush=True)
    except Exception as e:
        print(f"[WARN] armature not applied: {e}", flush=True)

world.reset()
robot.set_joint_positions(home_full)
for _ in range(60):
    world.step(render=False)

# ── 13. Dynamics API probe ─────────────────────────────────────────────────────────────
# Objective O1 needs the mass matrix, the gravity term, the Coriolis term and the
# Jacobian, and O2 needs the measured joint efforts. Which of these a given Isaac Sim
# build exposes, and under what name, is the one thing the whole project rests on — so it
# is established here, first, against the live articulation rather than assumed.
banner("Dynamics API probe  (objective O1 feasibility)")

PROBES = [
    ("mass_matrix", "get_mass_matrices", "M(q)"),
    ("gravity", "get_generalized_gravity_forces", "g(q)"),
    ("coriolis", "get_coriolis_and_centrifugal_forces", "C(q,qdot)qdot"),
    ("jacobian", "get_jacobians", "J(q)"),
    ("measured_efforts", "get_measured_joint_efforts", "tau_measured"),
    ("measured_forces", "get_measured_joint_forces", "joint reaction wrenches"),
    ("applied_efforts", "get_applied_joint_efforts", "tau_commanded"),
]

probe_results = {}
samples = {}
for key, method_name, symbol in PROBES:
    method = getattr(articulation, method_name, None)
    if method is None:
        probe_results[key] = {"available": False, "reason": "method not present"}
        print(f"  ✗ {symbol:<24} {method_name}() — not present on this build", flush=True)
        continue
    try:
        value = np.asarray(method())
        probe_results[key] = {"available": True, "shape": list(value.shape),
                              "method": method_name}
        samples[key] = value
        finite = "finite" if np.all(np.isfinite(value)) else "CONTAINS NON-FINITE"
        print(f"  ✓ {symbol:<24} {method_name}() → shape {value.shape}, {finite}", flush=True)
    except Exception as e:
        probe_results[key] = {"available": False, "reason": str(e), "method": method_name}
        print(f"  ✗ {symbol:<24} {method_name}() — raised: {e}", flush=True)

# ── 14. Static gravity check  (the seed of objective O2) ───────────────────────────────
# The arm is standing still at a known pose. With qdot = qddot = 0 the dynamics reduce to
# tau_model = g(q), so the measured effort must equal the gravity term. This is the
# cheapest possible version of the section 7.2 calibration test, and if it fails nothing
# downstream is worth running: a mismatch here is a configuration error, and every later
# residual would be measuring that bug instead of a physical effect.
banner("Static gravity check  (qdot = qddot = 0, so tau_measured should equal g(q))")

gravity_check = {"ran": False}
if "gravity" in samples and "measured_efforts" in samples:
    g_q = samples["gravity"].flatten()[:arm_dof]
    tau_m = samples["measured_efforts"].flatten()[:arm_dof]
    residual = tau_m - g_q
    # Relative error against the per-joint torque limit, not against g(q) itself: joints
    # whose gravity term passes through zero would otherwise show an infinite error.
    rel = np.abs(residual) / cfg.TAU_MAX
    worst = int(np.argmax(rel))
    passed = bool(np.all(rel <= cfg.CALIBRATION_TOLERANCE))

    print(f"  {'joint':<16}{'g(q)':>10}{'tau_meas':>12}{'residual':>12}{'% of limit':>12}", flush=True)
    for i, name in enumerate(cfg.JOINT_NAMES):
        print(f"  {name:<16}{g_q[i]:>10.3f}{tau_m[i]:>12.3f}{residual[i]:>12.3f}"
              f"{100*rel[i]:>11.2f}%", flush=True)
    print(f"\n  worst joint       {cfg.JOINT_NAMES[worst]} at {100*rel[worst]:.2f}% of its "
          f"{cfg.TAU_MAX[worst]:.0f} N*m limit", flush=True)
    print(f"  tolerance         {100*cfg.CALIBRATION_TOLERANCE:.0f}% (section 7.2)", flush=True)
    print(f"  result            {'PASS' if passed else 'FAIL'}", flush=True)
    if not passed:
        print("\n  A failure here is a configuration problem, not a physical finding.", flush=True)
        print("  Likely causes, in order: the gravity term is returned with the opposite", flush=True)
        print("  sign convention; the measured effort includes the finger DOFs so the", flush=True)
        print("  slice is misaligned; armature is in the plant but not in g(q); the arm", flush=True)
        print("  had not settled before the sample was taken.", flush=True)
    # A clean common scale factor across all seven joints is the signature of a
    # motor-side / link-side mix-up rather than a modelling error: NVIDIA states these
    # conventions are not interchangeable, and they differ by the gear ratio. A ratio
    # that is consistent per joint but not 1.0 tells us which convention we are reading.
    with np.errstate(divide="ignore", invalid="ignore"):
        per_joint_ratio = np.where(np.abs(g_q) > 1e-6, tau_m / g_q, np.nan)
    finite_ratios = per_joint_ratio[np.isfinite(per_joint_ratio)]
    if finite_ratios.size >= 3:
        spread = float(np.nanstd(finite_ratios) / max(abs(np.nanmean(finite_ratios)), 1e-9))
        print(f"\n  tau_measured/g(q) per joint: {np.round(per_joint_ratio, 4)}", flush=True)
        print(f"  mean {np.nanmean(finite_ratios):.4f}, relative spread {spread:.4f}", flush=True)
        if spread < 0.02 and abs(np.nanmean(finite_ratios) - 1.0) > 0.02:
            print("  A consistent ratio that is not 1.0 means a uniform scale factor —", flush=True)
            print("  most likely a motor-side vs link-side torque convention mismatch.", flush=True)
            print("  Record the convention in pal_config.TORQUE_CONVENTION before going on.", flush=True)
        gravity_check["tau_over_gravity_ratio"] = per_joint_ratio.tolist()
        gravity_check["ratio_relative_spread"] = spread

    gravity_check = {
        "ran": True, "passed": passed,
        "pose": "home", "joints": cfg.HOME_JOINTS.tolist(),
        "gravity_term": g_q.tolist(),
        "measured_effort": tau_m.tolist(),
        "residual": residual.tolist(),
        "residual_pct_of_limit": (100 * rel).tolist(),
        "worst_joint": cfg.JOINT_NAMES[worst],
        "tolerance": cfg.CALIBRATION_TOLERANCE,
    }
else:
    print("  skipped — g(q) or tau_measured unavailable (see the probe above)", flush=True)

# ── 15. Pose conditioning survey  (the seed of objective O3) ───────────────────────────
# The confidence score c(q) = sigma_min(J)/sigma_max(J) only means something once its
# distribution over real poses is known. CONFIDENCE_THRESHOLD in the config is a guess
# until this table exists; the probe poses are named for the condition each one tests.
banner("Pose conditioning survey  (objective O3 seed)")

pose_survey = {}
if probe_results.get("jacobian", {}).get("available"):
    print(f"  {'pose':<26}{'sigma_min':>11}{'sigma_max':>11}{'c(q)':>9}"
          f"{'manipulability':>16}", flush=True)
    for pose_name, q in cfg.PROBE_POSES.items():
        robot.set_joint_positions(np.concatenate([q, [cfg.FINGER_OPEN, cfg.FINGER_OPEN]]))
        for _ in range(40):
            world.step(render=False)
        try:
            jac = np.asarray(articulation.get_jacobians())
            # The view returns a batch of per-link Jacobians; the end-effector Jacobian is
            # the last one, and only the 7 arm columns matter for conditioning.
            j_ee = jac.reshape(jac.shape[0], -1, jac.shape[-2], jac.shape[-1])[0, -1]
            j_arm = j_ee[:, :arm_dof]
            sv = np.linalg.svd(j_arm, compute_uv=False)
            sigma_min, sigma_max = float(sv.min()), float(sv.max())
            confidence = sigma_min / sigma_max if sigma_max > 0 else 0.0
            manipulability = float(np.prod(sv))
            flag = "  ← low trust" if confidence < cfg.CONFIDENCE_THRESHOLD else ""
            print(f"  {pose_name:<26}{sigma_min:>11.5f}{sigma_max:>11.5f}"
                  f"{confidence:>9.4f}{manipulability:>16.3e}{flag}", flush=True)
            pose_survey[pose_name] = {
                "joints": q.tolist(), "sigma_min": sigma_min, "sigma_max": sigma_max,
                "confidence": confidence, "manipulability": manipulability,
                "singular_values": sv.tolist(),
            }
        except Exception as e:
            print(f"  {pose_name:<26} failed: {e}", flush=True)
            pose_survey[pose_name] = {"error": str(e)}

    if pose_survey:
        scores = [v["confidence"] for v in pose_survey.values() if "confidence" in v]
        if scores:
            print(f"\n  c(q) spans {min(scores):.4f} to {max(scores):.4f} across these poses.", flush=True)
            print(f"  CONFIDENCE_THRESHOLD is currently {cfg.CONFIDENCE_THRESHOLD}; set it from a", flush=True)
            print("  full workspace sweep, not from this handful of poses.", flush=True)
else:
    print("  skipped — the Jacobian is unavailable (see the probe above)", flush=True)

# ── 16. Render the scene ───────────────────────────────────────────────────────────────
robot.set_joint_positions(home_full)
for _ in range(cfg.STEPS_PER_RENDER * 4):
    world.step(render=True)

render_path = os.path.join(OUT_DIR, "scene_isometric.png")
try:
    import cv2
    frame = rgb_annotator.get_data()
    if frame is not None and getattr(frame, "size", 0) > 0:
        rgb = np.asarray(frame)[:, :, :3]
        cv2.imwrite(render_path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        print(f"\n[render] isometric view written to {render_path}", flush=True)
    else:
        render_path = None
        print("\n[WARN] render product returned no data", flush=True)
except Exception as e:
    render_path = None
    print(f"\n[WARN] could not write the render: {e}", flush=True)

# ── 17. Audit report ───────────────────────────────────────────────────────────────────
report = {
    "generated": time.strftime("%Y-%m-%dT%H:%M:%S"),
    "simulation": {
        "physics_hz": PHYSICS_HZ, "physics_dt": PHYSICS_DT,
        "render_hz": cfg.RENDER_HZ, "steps_per_render": cfg.STEPS_PER_RENDER,
        "physx": physx_applied,
    },
    "arm": {
        "dof_names": list(robot.dof_names), "num_dof": int(robot.num_dof),
        "arm_dof": arm_dof, "drive_mode": DRIVE_MODE,
        "drive_note": mode["note"],
        "gripper_variant": gripper_variant_selected,
        "stock_gains": stock, "applied": applied,
        "mass_total_kg": cfg.ARM_MASS_TOTAL, "tool_mass_kg": cfg.TOOL_MASS,
        "friction_source": cfg.FRICTION_SOURCE,
        "plant_friction_scale": cfg.PLANT_FRICTION_SCALE,
        "torque_convention": cfg.TORQUE_CONVENTION,
        "armature_choice": "physical" if np.allclose(cfg.ARMATURE, cfg.ARMATURE_PHYSICAL) else "isaaclab",
    },
    "payload_kg": PAYLOAD_KG,
    "scene": {
        "backdrop": BACKGROUND, "ground_tint": list(cfg.GROUND_TINT),
        "camera": {"position": list(cfg.CAM_POSITION), "look_at": list(cfg.CAM_LOOK_AT),
                   "focal_length_mm": cfg.CAM_FOCAL_LENGTH,
                   "resolution": list(cfg.CAM_RESOLUTION)},
        "lighting": "studio",
        "render": render_path,
    },
    "dynamics_api": probe_results,
    "static_gravity_check": gravity_check,
    "pose_survey": pose_survey,
}

report_path = os.path.join(OUT_DIR, "physics_audit.json")
with open(report_path, "w") as f:
    json.dump(report, f, indent=2, default=str)

banner("Summary")
ok = [k for k, v in probe_results.items() if v.get("available")]
missing = [k for k, v in probe_results.items() if not v.get("available")]
print(f"  dynamics quantities available   {len(ok)}/{len(PROBES)}: {', '.join(ok)}", flush=True)
if missing:
    print(f"  MISSING                        {', '.join(missing)}", flush=True)
    print("  Objectives O1/O2 depend on these. If M(q), g(q) or the Jacobian is", flush=True)
    print("  missing, they must be computed analytically from the URDF instead —", flush=True)
    print("  and then armature has to be added to the model by hand (see pal_config).", flush=True)
if gravity_check.get("ran"):
    print(f"  static gravity check           {'PASS' if gravity_check['passed'] else 'FAIL'}", flush=True)
print(f"  audit report                   {report_path}", flush=True)
if render_path:
    print(f"  isometric render               {render_path}", flush=True)

# ── 18. Idle, or exit ──────────────────────────────────────────────────────────────────
# With a window open, keep stepping so the scene can be inspected. Headless, the audit is
# the whole product of this script, so it exits.
if not HEADLESS:
    print("\n  Window open — Ctrl+C to stop.", flush=True)
    try:
        while simulation_app.is_running():
            world.step(render=True)
    except KeyboardInterrupt:
        pass

simulation_app.close()
