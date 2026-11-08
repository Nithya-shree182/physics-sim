"""Physics Attestation Layer — every simulator and scene parameter in one place.

Nothing arm-specific is written here. The arm's own numbers (limits, masses, friction,
armature, URDF, tool frame, USD) live in `pal_robot.py` as a single `RobotSpec`, and this
module re-exports the active one under the names the rest of the code already reads
(`cfg.HOME_JOINTS`, `cfg.TAU_MAX`, ...). What stays here is everything that is a property of
the *simulation*, not of the robot: rates, the PhysX solver, contact, drive gains, the
camera and lights, and the attestation thresholds.

Why this file exists at all: the residual r = tau_measured - tau_model is only meaningful if
the model's assumptions and the simulator's configuration are both known exactly. Any
parameter that enters M(q), C(q,qd) or g(q) must be auditable, so none of them is allowed to
sit inline in scene code.

Source tags on each value:

  [DATASHEET]  published manufacturer specification
  [CHOICE]     a decision made for this project, with the reason given
  [MEASURED]   established by a run of this code, with the number quoted

History: until 2026-11-08 this file was ~700 lines of Franka Panda constants, each with its
provenance. The Panda was removed on 2026-11-08 and those values are gone with it; they are
in git history at commit b696c1c if the sourcing is ever wanted again. See docs/UR10E_PORT.md.
"""

import numpy as np

import pal_robot

# ══════════════════════════════════════════════════════════════════════════════════════
# 0. The active arm
# ══════════════════════════════════════════════════════════════════════════════════════
# One RobotSpec, selected by $PAL_ROBOT. Everything arm-specific is a view onto it, so the
# rest of the code never asks which arm is loaded.
ROBOT = pal_robot.active()
ROBOT_NAME = ROBOT.name
DOF = ROBOT.dof

JOINT_NAMES = list(ROBOT.joint_names)
HOME_JOINTS = ROBOT.home
Q_MIN, Q_MAX = ROBOT.q_min, ROBOT.q_max
QD_MAX = ROBOT.qd_max
TAU_MAX = ROBOT.tau_max
ARMATURE = ROBOT.armature
REACH_M = ROBOT.reach

FRICTION_COULOMB_STATIC = ROBOT.friction_static
FRICTION_COULOMB_DYNAMIC = ROBOT.friction_dynamic
FRICTION_VISCOUS = ROBOT.friction_viscous
FRICTION_SOURCE = ROBOT.notes.get("friction_source", "[CHOICE] authored, see pal_robot.py")

URDF_PATH = ROBOT.urdf_file()
ROBOT_PRIM = ROBOT.prim
ROBOT_USD_SUBPATH = ROBOT.usd_subpath
TOOL_FRAME = ROBOT.tool_frame
TCP_OFFSET = np.asarray(ROBOT.tool_offset, dtype=float)

SENSOR_SIGMA_Q = ROBOT.sensors.q
SENSOR_SIGMA_QD = ROBOT.sensors.qd
SENSOR_SIGMA_TAU = ROBOT.sensors.tau

# Named joint configurations the suites sweep. The UR10e has one; the five extra poses that
# used to live here were Panda configurations chosen for a confidence score that no longer
# exists (DECISIONS.md D6).
PROBE_POSES = {"home": ROBOT.home}
POSES = PROBE_POSES

ARMATURE_CONTRACT = f"Pinocchio side sets model.armature = {ROBOT.armature.tolist()}"
APPLY_ARMATURE = True

# The tool is part of the robot, so its mass is in the URDF, not here. What a *payload* adds
# is handled by pal_plant.set_inertials() at TCP_OFFSET.

# ══════════════════════════════════════════════════════════════════════════════════════
# 1. Simulation rates
# ══════════════════════════════════════════════════════════════════════════════════════
# [MEASURED, DECISIONS.md D3] Two effects pull in opposite directions.
#
# THROUGHPUT wants a lower rate. A full telemetry frame costs about 1.29 ms to read whatever
# the physics rate is — the readback dominates, not the solver — so steps/s is roughly flat
# and real-time factor scales as 1/rate.
#
# THE RESIDUAL wants a higher rate, because joint acceleration is estimated by differencing
# joint velocity and that estimate is the dominant noise source. Fitting RMS residual against
# dt gives an exponent of 1.049: first order in dt, the signature of time discretisation
# rather than of a modelling error. Two controls confirm it — solver iteration count changes
# nothing (14.24% at 4 iterations and at 128) and armature changes nothing.
#
# The resolution (docs/RESIDUAL_FLOOR.md): at or below 50% of the velocity limit, which is
# the only regime this project reports (D1), the residual is the same from 250 to 2000 Hz.
# So data runs keep 1000 Hz for margin and the live view runs at 250 Hz, which keeps up with
# real time. Those numbers were measured on the Panda and have not been re-measured on the
# UR10e — see docs/TODO.md, item 1.
PHYSICS_HZ = 1000.0
LIVE_PHYSICS_HZ = 250.0               # [CHOICE] the rate the watched cell runs at
RENDER_HZ = 30.0                      # [CHOICE] rendering is for the human, not the math

PHYSICS_DT = 1.0 / PHYSICS_HZ
RENDER_DT = 1.0 / RENDER_HZ
# Physics steps per rendered frame. The telemetry row is published every physics step, not
# every rendered frame: the dashboard decimates, the dynamics core does not.
STEPS_PER_RENDER = int(round(PHYSICS_HZ / RENDER_HZ))

GRAVITY = -9.81                       # [CHOICE] set explicitly; g(q) is a measured term here
GRAVITY_VEC = np.array([0.0, 0.0, GRAVITY])
GRAVITY_DIRECTION = (0.0, 0.0, -1.0)

# ══════════════════════════════════════════════════════════════════════════════════════
# 2. PhysX solver
# ══════════════════════════════════════════════════════════════════════════════════════
# [CHOICE] TGS with raised position iterations. 32 is the standard recommendation for an
# articulation whose joint forces are being *measured* rather than merely simulated: an
# unconverged solver shows up directly as a residual the detector would read as a fault.
# For comparison, Isaac Lab's robot configs use 8 position / 0 velocity iterations, which is
# tuned for RL throughput across thousands of parallel environments where per-step torque
# accuracy does not matter. This project pays for convergence instead.
SOLVER_TYPE = "TGS"
POSITION_ITERATIONS = 32
VELOCITY_ITERATIONS = 4
ENABLE_CCD = True                     # [CHOICE] needed once carried boxes move fast
ENABLE_STABILIZATION = False          # [CHOICE] OFF, and never to be enabled. Stabilization
                                      # injects non-physical damping, which is
                                      # indistinguishable from the friction fault of O5.
ENABLE_GPU_DYNAMICS = False           # [CHOICE] CPU articulation: the dynamics readback
                                      # (M, Coriolis, gravity) is better supported on the CPU
                                      # pipeline, and one arm does not need GPU broadphase.

# ══════════════════════════════════════════════════════════════════════════════════════
# 3. The degree/radian trap  —  read before touching any gain or friction value
# ══════════════════════════════════════════════════════════════════════════════════════
# [MEASURED] Isaac's USD drive attributes (drive:angular:physics:stiffness) are per DEGREE,
# while the tensor API (Articulation.set_gains / get_gains) is per RADIAN. The two differ by
# exactly 180/pi = 57.3. A gain authored in one slot and read through the other looks
# correctly tuned from one side and 57x wrong from the other; only reading both resolves it.
# Every gain in this file is per RADIAN. pal_plant converts on write.
DEG_PER_RAD = 180.0 / np.pi
RAD_PER_DEG = np.pi / 180.0

# ══════════════════════════════════════════════════════════════════════════════════════
# 4. Friction mismatch between plant and model
# ══════════════════════════════════════════════════════════════════════════════════════
# The proposal requires the twin's model to differ from the plant, otherwise detection
# succeeds trivially. The plant gets the robot spec's friction multiplied by this scale; the
# model uses the spec's values unscaled, and through a different law (a smooth sigmoid rather
# than PhysX's static/dynamic/viscous triple). The gap between those two representations is
# part of what the residual legitimately measures.
PLANT_FRICTION_SCALE = 1.15           # [CHOICE] 15% plant-vs-model friction mismatch
APPLY_PLANT_FRICTION = True
LINK_MASS_SCALE = 1.05                # [CHOICE] 5% plant-vs-model mass mismatch on driven links


def friction_sigmoid(qd, spec=None):
    """The model side's friction law. Delegates to model/pinocchio_model.friction_sigmoid's
    formula without importing Pinocchio, so the simulator container can call it too.

        tau_f(qd) = phi1/(1+exp(-phi2(qd+phi3))) - phi1/(1+exp(-phi2 phi3))

    Smooth through zero and exactly zero at rest, so it does not disturb a gravity check.
    """
    qd = np.asarray(qd, dtype=float)
    s = spec or ROBOT
    a, b, c = s.friction_phi1, s.friction_phi2, s.friction_phi3
    return a / (1.0 + np.exp(-b * (qd + c))) - a / (1.0 + np.exp(-b * c))


# ══════════════════════════════════════════════════════════════════════════════════════
# 5. Torque convention  —  which side of the gearbox
# ══════════════════════════════════════════════════════════════════════════════════════
# The residual is a subtraction of two torques, so it is only meaningful if both are on the
# same side of the gearbox. Motor-side and link-side differ by the gear ratio and by the
# reflected inertia term; mixing them gives a residual that scales with acceleration and
# looks exactly like a payload error — quietly wrong. NVIDIA's own sysid material states
# plainly that motor-side, link-side, commanded and measured efforts are not interchangeable.
#
# Left "unverified" on purpose (DECISIONS.md P5). The cheapest test is a static gravity check:
# at rest the two conventions differ only by the gear ratio, so a mismatch appears as one
# clean scale factor on every joint at once. tests/test_ur10e.py does that check and it
# passes at the identity, which is consistent with link-side but does not prove it.
TORQUE_CONVENTION = "unverified"

# ══════════════════════════════════════════════════════════════════════════════════════
# 6. Joint drive gains
# ══════════════════════════════════════════════════════════════════════════════════════
# [CHOICE] Gains are expressed as a multiple of each joint's own effort limit, so they scale
# with the arm instead of being re-tuned by hand. The reference point is the gain set this
# project used on the Panda (kp 400, kd 80 N*m/rad against an 87 N*m limit), which is
# 4.6 x tau_max and a damping ratio of kd/kp = 0.2.
#
# Why the mode matters: a stiff drive tracks the commanded trajectory almost regardless of
# load, which is good for attestation — the payload shows up in torque rather than in
# position error, and torque is what the residual reads. It makes a push test impossible
# though, because pushing the tool of a stiff drive moves nothing measurable. That test needs
# the "torque" mode below.
KP_PER_TAU = 4.6                      # [CHOICE] N*m/rad per N*m of effort limit
KD_PER_KP = 0.20                      # [CHOICE] damping as a fraction of stiffness

DRIVE_MODES = {
    "position_stiff": {
        "kp": KP_PER_TAU * TAU_MAX,
        "kd": KD_PER_KP * KP_PER_TAU * TAU_MAX,
        "note": "default: stiff position drive, 4.6x effort limit per rad",
    },
    # [CHOICE] A fifth of the stiffness. A second operating point for the trajectory-style
    # benchmark: the same motion under a softer drive spends its torque differently.
    "position_soft": {
        "kp": 0.2 * KP_PER_TAU * TAU_MAX,
        "kd": 0.2 * KD_PER_KP * KP_PER_TAU * TAU_MAX,
        "note": "soft position drive, a fifth of the stiffness",
    },
    # [CHOICE] Pure torque mode for the independent push validation: zero stiffness, so an
    # applied external force at the tool actually deflects the arm and the estimated wrench
    # can be compared against the applied one. kd stays small but non-zero; at exactly zero
    # the arm free-falls the instant it engages and the test is unrunnable.
    "torque": {
        "kp": np.zeros(DOF),
        "kd": 0.02 * TAU_MAX,
        "note": "low-gain torque mode for the known-push validation",
    },
}
DEFAULT_DRIVE_MODE = "position_stiff"
DEFAULT_KP = DRIVE_MODES[DEFAULT_DRIVE_MODE]["kp"]
DEFAULT_KD = DRIVE_MODES[DEFAULT_DRIVE_MODE]["kd"]

# ══════════════════════════════════════════════════════════════════════════════════════
# 7. Payload and contact
# ══════════════════════════════════════════════════════════════════════════════════════
# The carried object is a cardboard box handled by a suction cup array (pal_suction.py), not
# a cube in two fingers. Its geometry lives with the cell (pal_cell.BOX); what is here is the
# mass ladder the payload objective sweeps and the contact parameters.
PAYLOAD_LADDER_KG = [0.25, 0.50, 1.00, 2.00, 4.00, 8.00]   # [CHOICE] within the 12.5 kg rating
PAYLOAD_DEFAULT_KG = 2.0              # [CHOICE] a believable carton
PAYLOAD_RATED_MAX_KG = 12.5           # [DATASHEET] UR10e rated payload
PAYLOAD_SIZE = 0.22                   # m, box footprint (pal_cell.BOX x/y); kept for the
                                      # schematic views, which draw the carried box

CONTACT_OFFSET = 0.004                # [CHOICE] m, PhysX begins generating contacts here
REST_OFFSET = 0.001                   # [CHOICE] m, resting separation
GRASP_STATIC_FRICTION = 1.05          # [CHOICE] cup-to-carton
GRASP_DYNAMIC_FRICTION = 0.95
SURFACE_STATIC_FRICTION = 0.70        # [CHOICE] carton-to-belt
SURFACE_DYNAMIC_FRICTION = 0.60
RESTITUTION = 0.0                     # [CHOICE] cardboard does not bounce

# ══════════════════════════════════════════════════════════════════════════════════════
# 8. Fallback workcell geometry
# ══════════════════════════════════════════════════════════════════════════════════════
# Only used by dress_scene() when a bare arm is wanted without the conveyor cell (selftests,
# sweeps). The cell lays out its own floor and furniture in pal_cell.py.
TABLE_POSITION = (0.60, 0.0, -0.05)
TABLE_SCALE = (0.90, 1.30, 0.10)      # top face at z = 0
TABLE_COLOR = (0.16, 0.18, 0.22)
GROUND_PLANE_Z = -0.12                # below the table's bottom face, avoids z-fighting
GROUND_TINT = (0.10, 0.16, 0.34)

# ══════════════════════════════════════════════════════════════════════════════════════
# 9. Camera and lighting
# ══════════════════════════════════════════════════════════════════════════════════════
# The isometric three-quarter overview. Pulled back from the Panda framing because the UR10e
# is two thirds taller and reaches half again as far. The cell overrides these with its own
# camera (pal_cell.CAM_EYE / CAM_TARGET), which has to see a conveyor as well as the arm.
CAM_POSITION = (2.40, -2.10, 1.60)
CAM_LOOK_AT = (0.45, 0.10, 0.45)
CAM_FOCAL_LENGTH = 22.0
CAM_HORIZONTAL_APERTURE = 20.955      # USD Camera schema default
CAM_RESOLUTION = (1280, 720)          # [CHOICE] 720p

LIGHTS = {
    "dome": {"intensity": 700.0},
    "key": {"intensity": 1600.0, "color": (1.0, 0.95, 0.90), "rotation": (-45.0, 30.0, 0.0)},
    "fill": {"intensity": 550.0, "position": (1.40, -1.00, 1.60), "radius": 0.40},
}

# ══════════════════════════════════════════════════════════════════════════════════════
# 10. Attestation thresholds
# ══════════════════════════════════════════════════════════════════════════════════════
# [CHOICE] The proposal's own stated pass conditions, pulled out so the evaluation table is
# filled in from the same constants the code runs on.
CALIBRATION_TOLERANCE = 0.05          # model within 5% per joint
RESIDUAL_ALARM_NM = 2.0               # per-joint residual alarm level
RESIDUAL_ALARM_CONSECUTIVE = 5        # consecutive steps above the level before alarming
PAYLOAD_FILTER_ALPHA = 0.02           # running filter on the payload estimate (O4)
SPEED_BAND = 0.50                     # D1: the fraction of the velocity limit this project reports


if __name__ == "__main__":
    print(f"arm            {ROBOT_NAME}  ({DOF} joints, reach {REACH_M} m)")
    print(f"physics        {PHYSICS_HZ:.0f} Hz data / {LIVE_PHYSICS_HZ:.0f} Hz live, {SOLVER_TYPE} "
          f"{POSITION_ITERATIONS}/{VELOCITY_ITERATIONS}")
    print(f"tau_max        {TAU_MAX.tolist()}")
    print(f"drive          {DEFAULT_DRIVE_MODE}: kp {DEFAULT_KP.round(0).tolist()} N*m/rad")
    print(f"friction       plant = spec x {PLANT_FRICTION_SCALE}, model = spec sigmoid")
    print(f"urdf           {URDF_PATH}")
