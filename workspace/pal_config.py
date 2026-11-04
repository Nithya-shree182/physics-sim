"""Physics Attestation Layer — every tunable scene and arm parameter in one place.

Each value below carries its source. Three kinds appear:

  [DATASHEET]  Franka Emika Panda published specification. Authoritative.
  [ASSET]      What NVIDIA's franka.usd / panda_arm_hand.urdf actually ships, read off a
               live run of the Physyk AI service (logs/isaac_sim.log) or the URDF itself.
  [LITERATURE] Identified dynamic parameters from published Franka identification work
               (Gaz et al. 2019, "Dynamic Identification of the Franka Emika Panda Robot
               with Retrieval of Feasible Parameters Using Penalty-Based Optimization").
               These are NOT verified on this asset — see FRICTION_* notes.
  [CHOICE]     A decision made for this project, with the reason given.

Why this file exists separately: the attestation residual r = tau_measured - tau_model is
only meaningful if the model's assumptions and the simulator's configuration are known
exactly. Any parameter that enters M(q), C(q,qdot) or g(q) must be auditable, so none of
them are allowed to sit inline in the scene code.
"""

import numpy as np

# ══════════════════════════════════════════════════════════════════════════════════════
# 1. Simulation rates
# ══════════════════════════════════════════════════════════════════════════════════════
# [CHOICE] 400 Hz physics, decoupled from rendering.
#
# The Physyk AI scene runs physics at 60 Hz because it only needs a pick to look right.
# This project estimates joint acceleration as a finite difference of joint velocity
# (objective O2), and that estimate is the dominant noise source in the residual —
# risk 2 in the proposal's section 11. Finite-difference noise scales as 1/dt for a
# fixed velocity quantisation, so the physics rate is the cheapest lever on residual
# quality that exists.
#
# 1000 Hz. Two measurements pull in opposite directions and the residual wins.
#
# THROUGHPUT wants a lower rate. A full telemetry frame costs ~1.29 ms to read regardless
# of the physics rate — the readback dominates, not the solver — so throughput is flat at
# about 780 steps/s:
#       1000 Hz -> 0.76x real time      500 Hz -> 1.52x      400 Hz -> 1.94x
#
# RESIDUAL wants a higher rate, and much more strongly. Measured on a 100%-of-limit
# excitation trajectory, worst joint as a fraction of its torque limit:
#       200 Hz -> 50.91%    500 Hz -> 14.24%    1000 Hz -> 2.78%    2000 Hz -> 1.44%
#
# Fitting RMS residual against dt gives an exponent of 1.049 — first order in dt, which is
# the signature of time discretization, not of a modelling error. Confirmed by two
# controls: solver iteration count changes nothing (14.24% at 4 iterations and at 128),
# and armature changes nothing (2.3692 vs 2.3892). Residual also rises with drive
# stiffness (kp 80 -> 12.17%, kp 5000 -> 33.87%), which is the same effect seen from the
# other side: a stiffer drive is a stiffer ODE and discretizes worse at a given dt.
#
# So dropping to 500 Hz to buy real-time telemetry would multiply the residual floor by
# five, and the residual IS the measurement. The right trade is to keep 1000 Hz and
# decimate the expensive readbacks instead: M (49 floats) and J (42) are 57% of the frame
# and dominate its cost, while both change very little in one millisecond. The analysis
# side has proposed encoding that as a held last value plus an `mj_fresh` flag.
#
# UPDATE 2026-11-04 (DECISIONS.md D3): that sweep rebuilt the World per rate and was later found wrong
# (docs/RESIDUAL_FLOOR.md). At or below 50% of the velocity limit the residual is the same at 250-2000 Hz.
# Data runs keep 1000 Hz; the LIVE view runs at 250 Hz (pal.sh), which keeps up with real time.
PHYSICS_HZ = 1000.0
RENDER_HZ = 30.0                      # [CHOICE] rendering is for the human, not the math

PHYSICS_DT = 1.0 / PHYSICS_HZ
RENDER_DT = 1.0 / RENDER_HZ
# Physics steps per rendered frame. The telemetry frame (O1) is published every PHYSICS
# step, not every render step — the dashboard decimates, the dynamics core does not.
STEPS_PER_RENDER = int(round(PHYSICS_HZ / RENDER_HZ))

GRAVITY = -9.81                       # [CHOICE] set explicitly; g(q) is a measured term here
GRAVITY_DIRECTION = (0.0, 0.0, -1.0)

# ══════════════════════════════════════════════════════════════════════════════════════
# 2. PhysX solver
# ══════════════════════════════════════════════════════════════════════════════════════
# [CHOICE] TGS with raised position iterations. Physyk uses TGS 16/4 and that proved
# stable for grasping; dynamics readback is more sensitive than grasping is, because an
# unconverged solver shows up directly as a residual the detector would read as a fault.
# 32 position iterations is the standard recommendation for articulations whose joint
# forces are being measured rather than merely simulated.
# [NVIDIA] For reference, Isaac Lab's Franka configs use solver_position_iteration_count=8
# and solver_velocity_iteration_count=0. That is tuned for RL throughput across thousands
# of parallel environments, where per-step torque accuracy does not matter. This project
# reads joint forces back as its primary measurement, so it pays for convergence instead:
# an unconverged solver shows up directly as a residual the detector would read as a fault.
SOLVER_TYPE = "TGS"
POSITION_ITERATIONS = 32
VELOCITY_ITERATIONS = 4
POSITION_ITERATIONS_ISAACLAB = 8      # [NVIDIA] for reference; deliberately not used
VELOCITY_ITERATIONS_ISAACLAB = 0
ENABLE_CCD = True                     # [ASSET] matches Physyk; needed once payloads move fast
ENABLE_STABILIZATION = False          # [CHOICE] OFF. Stabilization injects non-physical
                                      # damping, which is indistinguishable from the
                                      # friction fault of objective O5. Never enable it.
ENABLE_GPU_DYNAMICS = False           # [CHOICE] CPU articulation. The dynamics readback
                                      # (mass matrix / Coriolis / gravity) is better
                                      # supported on the CPU pipeline, and one arm does
                                      # not need GPU broadphase.

# ══════════════════════════════════════════════════════════════════════════════════════
# 3. Franka Panda — joint limits
# ══════════════════════════════════════════════════════════════════════════════════════
# [DATASHEET] + [ASSET] These agree exactly between the Panda specification and
# assets/franka_panda/panda_arm_hand.urdf, which is a useful independent confirmation
# that the shipped asset is the real robot and not a simplified stand-in.
#
# Note the two asymmetric joints. They matter for objective O3: the reachable set is not
# a box centred on zero, so "near-singular" poses are not symmetric either.
#   joint4 lives entirely at negative angles: [-3.0718, -0.0698]
#   joint6 cannot go below -0.0175
JOINT_NAMES = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
               "panda_joint5", "panda_joint6", "panda_joint7"]
FINGER_JOINT_NAMES = ["panda_finger_joint1", "panda_finger_joint2"]

Q_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
Q_MAX = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])

# [DATASHEET] rad/s. Note a real divergence in NVIDIA's own assets: Isaac Lab's flat
# FRANKA_PANDA_CFG sets velocity limits of 20.0 rad/s (joints 1-4) and 25.0 rad/s
# (joints 5-7) — roughly 10x the datasheet, which is RL headroom rather than a physical
# claim. Its FRANKA_ROBOTIQ_GRIPPER_CFG, by contrast, uses the datasheet 2.175 / 2.61.
# The datasheet values are used here: a trajectory benchmark (O6) that compared motion
# styles at 10x the arm's real speed limit would be measuring nothing.
QD_MAX = np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100])
QD_MAX_ISAACLAB = np.array([20.0, 20.0, 20.0, 20.0, 25.0, 25.0, 25.0])   # [NVIDIA] reference

# [DATASHEET] rad/s^2. Used by the trajectory generators of objective O6 — a trapezoidal
# or S-curve profile is only a fair comparison if both respect the same real limits.
QDD_MAX = np.array([15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0])

# [DATASHEET] rad/s^3. The jerk limit is what makes the S-curve profile of O6 a
# physically meaningful alternative rather than a cosmetic smoothing.
QDDD_MAX = np.array([7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0])

# [DATASHEET] + [ASSET] N*m. Agrees with the live readback from the Physyk service
# ("Stock joint max efforts: [87. 87. 87. 87. 12. 12. 12. 7.2 0.]").
# This is the denominator for the "share of torque limit" link tint in section 8.
TAU_MAX = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])

# [DATASHEET] N*m/s. Torque rate limit — a commanded torque step beyond this is something
# the real arm would reject, so a spoofed setpoint that violates it is detectable without
# any dynamics at all. Worth logging as a cheap cross-check alongside the residual.
TAU_RATE_MAX = np.array([1000.0] * 7)

# ══════════════════════════════════════════════════════════════════════════════════════
# 4. Link masses
# ══════════════════════════════════════════════════════════════════════════════════════
# [ASSET] Read directly out of panda_arm_hand.urdf. Full 3x3 inertia tensors are present
# for all 11 links, so M(q) from the simulator is a real mass matrix, not a point-mass
# approximation.
#
# LINK_MASSES is here for one purpose: the deliberate twin/plant mismatch of section 9.
# The perturbation is applied as a fraction of these, so the magnitude of the mismatch is
# known rather than incidental.
LINK_MASSES = np.array([
    4.970684,   # panda_link1
    0.646926,   # panda_link2
    3.228604,   # panda_link3
    3.587895,   # panda_link4
    1.225946,   # panda_link5
    1.666555,   # panda_link6
    0.735522,   # panda_link7
])
LINK0_MASS = 0.6297                   # [ASSET] base, does not move, no contribution to g(q)
HAND_MASS = 0.73                      # [ASSET] panda_hand
FINGER_MASS = 0.1                     # [ASSET] each of two fingers

# [CHOICE] The gripper is part of the robot, not part of the payload. A payload estimator
# (O4) that does not subtract this will report every empty gripper as carrying 0.93 kg.
# Stated here so that bug is impossible rather than merely unlikely.
TOOL_MASS = HAND_MASS + 2 * FINGER_MASS            # 0.93 kg

ARM_MASS_TOTAL = float(LINK_MASSES.sum() + LINK0_MASS + TOOL_MASS)   # ~18.0 kg, matches spec

# ══════════════════════════════════════════════════════════════════════════════════════
# 5. The degree/radian trap  —  read before touching any gain or friction value
# ══════════════════════════════════════════════════════════════════════════════════════
# USD expresses ANGULAR drive and friction quantities per DEGREE, not per radian. NVIDIA's
# own system-identification tutorial calls this out explicitly: "USD angular drive
# stiffness is expressed per degree, so a gain intended per radian is about 57 times too
# stiff."
#
# This is not a footnote for this project, it is the first thing that must be right. Every
# number read back from an articulation or written to a joint prim has to be labelled with
# its convention, because a 57x error in a gain or a friction coefficient produces a
# residual that looks exactly like a large unmodelled payload.
#
# Worked example, which is also how the stock asset was decoded (see DRIVE_MODES below):
#     22918.3 per-degree  x  pi/180  =  400.0 per-radian
#      4583.7 per-degree  x  pi/180  =   80.0 per-radian
DEG_PER_RAD = 180.0 / np.pi          # 57.2958 — multiply per-radian to get per-degree
RAD_PER_DEG = np.pi / 180.0          # 0.0174533 — multiply per-degree to get per-radian

# ══════════════════════════════════════════════════════════════════════════════════════
# 6. Joint friction  —  the stock asset ships none
# ══════════════════════════════════════════════════════════════════════════════════════
# Three independent sources agree that the shipped Franka has no joint friction:
#   [ASSET]  panda_arm_hand.urdf declares friction="0.0" on every revolute joint.
#   [NVIDIA] The sysid tutorial states that the packaged franka.usda "authors no Newton
#            joint friction at all, so every arm joint would otherwise run at the solver
#            default."
#   [NVIDIA] Isaac Lab's FRANKA_PANDA_CFG sets joint friction explicitly to viscous 0.0.
#
# That matters more than it looks. It is the proposal's section 11 risk "model matches the
# simulator too well", already baked into the asset: a frictionless plant compared against
# a frictionless model yields a residual bounded only by solver noise, so every detection
# rate measured against it would be optimistic and meaningless. Friction has to be put
# into the plant deliberately before any O5 number is worth reporting.
#
# PhysX's friction model has three parts, and the project needs all of them separately
# because they have different fault signatures (section 7.5):
#   static Coulomb effort   N*m, velocity-independent, dominates at breakaway
#   dynamic Coulomb effort  N*m, velocity-independent, dominates in steady motion
#   viscous coefficient     proportional to speed  ← expressed per DEGREE, see section 5
#
# A real friction fault raises Coulomb first (a stiffening joint resists breakaway) while
# bearing wear tends to raise viscous. Modelling them as one scalar would throw away the
# distinction the fault classifier of O5 is supposed to learn.

# ── 6a. Primary friction values  [NVIDIA, sysid tutorial] ─────────────────────────────
# Recovered by NVIDIA's own optimiser against a real Franka MCAP recording (9,439 samples,
# 7 arm joints, ~30 Hz), reported as scales on a 1.0 N*m*s/rad seed, so these ARE the
# viscous coefficients in N*m*s/rad. Preferred over the literature values below for one
# reason: they were fitted inside this simulator family, in its own units, against this
# exact asset.
#
# NVIDIA's own caveat, which carries over verbatim to any result built on them: the
# recording "carries no authored friction value for the optimizer to rediscover", so the
# recovered number "is a predictive fit, not a certificate of true joint friction".
FRICTION_VISCOUS_NVIDIA = np.array([0.214, 0.105, 0.848, 0.184, 0.172, 0.147, 0.156])

# ── 6b. Gaz et al. 2019  [LITERATURE, VERIFIED against the source PDF] ───────────────
# Gaz, Cognetti, Oliva, Giordano, De Luca (2019), "Dynamic Identification of the Franka
# Emika Panda Robot with Retrieval of Feasible Parameters Using Penalty-Based
# Optimization", IEEE RA-L. Values below read directly from the supplementary material
# held at docs/references/gaz2019_panda_dynamic_identification_supplementary.pdf on the
# nithya branch — NOT from memory, and NOT from a text extraction of a table.
#
# The paper gives TWO friction models. They are not interchangeable.

# ── Model A (Table I): the one the paper actually fits ────────────────────────────────
# A smooth sigmoid in joint velocity, with no discontinuity at zero:
#
#     tau_f,j(qd) = phi1 / (1 + exp(-phi2 * (qd + phi3)))  -  phi1 / (1 + exp(-phi2 * phi3))
#
# The second term makes tau_f(0) = 0 exactly, so there is no spurious torque at rest.
# This is the better model for this project: a Coulomb sign() term chatters around qd = 0,
# and the arm spends much of a pick-and-place near zero velocity at the waypoints — which
# is precisely where a chattering friction term would masquerade as a residual spike and
# be classified as a fault.
FRICTION_SIGMOID_PHI1 = np.array([0.54615, 0.87224, 0.64068, 1.27940, 0.83904, 0.30301, 0.56489])   # N*m
FRICTION_SIGMOID_PHI2 = np.array([5.1181, 9.0657, 10.136, 5.5903, 8.3469, 17.133, 10.336])          # s/rad
FRICTION_SIGMOID_PHI3 = np.array([0.039533, 0.025882, -0.046070, 0.036194, 0.026226, -0.021047, 0.0035526])  # rad/s


def friction_sigmoid(qd):
    """Gaz et al. Table I friction torque, N*m, for a 7-vector of joint velocities.

    Smooth through zero by construction, so it can be applied every step without the
    chatter a sign() term produces at the waypoints of a pick-and-place.
    """
    qd = np.asarray(qd, dtype=float)
    a, b, c = FRICTION_SIGMOID_PHI1, FRICTION_SIGMOID_PHI2, FRICTION_SIGMOID_PHI3
    return a / (1.0 + np.exp(-b * (qd + c))) - a / (1.0 + np.exp(-b * c))


# ── Model B (Table III): linear viscous + Coulomb + offset ───────────────────────────
#     tau_f,j(qd) = fv,j * qd  +  fc,j * sign(qd)  +  fo,j
#
# The paper adopts this form only "because of their linearity in the defining parameters",
# and says plainly that it "may suffer from sudden discontinuities in the neighbourhood of
# qd = 0". Kept because PhysX's own friction is of this family, so it is what the plant can
# actually be configured with.
#
# NOTE the constant offset fo. It is a real identified term and it is easy to miss: a
# constant per-joint torque that does not vanish at zero velocity. Omitting it biases the
# residual by up to 0.25 N*m on joint 4 — a standing error that a payload estimator would
# quietly absorb as about 25 g of phantom mass.
FRICTION_VISCOUS_GAZ = np.array([0.0665, 0.1987, 0.0399, 0.2257, 0.1023, -0.0132, 0.0638])
FRICTION_COULOMB_GAZ = np.array([0.2450, 0.1523, 0.1827, 0.3591, 0.2669, 0.1658, 0.2109])
FRICTION_OFFSET_GAZ  = np.array([-0.1073, -0.1566, -0.0686, -0.2522, 0.0045, 0.0910, -0.0127])

# Relative standard deviations reported alongside each estimate in the same table. These
# decide which numbers may be leaned on and which may not:
#   fv,6 = -0.0132 with a standard deviation of 12.26 — the estimate is noise, and it is
#   also negative, which is physically impossible. Clamped to zero below, now on the
#   evidence of its own error bar rather than on taste.
#   fo,5 (std 19.46) and fo,7 (std 6.26) are likewise not meaningful.
FRICTION_VISCOUS_STD_GAZ = np.array([1.9658, 0.9615, 3.4971, 0.9124, 1.5868, 12.2607, 2.1388])
FRICTION_OFFSET_STD_GAZ  = np.array([0.6932, 2.9549, 1.1566, 0.9563, 19.4633, 0.8390, 6.2628])

FRICTION_VISCOUS_GAZ_CLAMPED = np.maximum(FRICTION_VISCOUS_GAZ, 0.0)

# ── 6c. What the plant actually gets  [CHOICE] ────────────────────────────────────────
# "gaz" is now the default, not "nvidia". The Gaz values are a published identification of
# a real Panda with per-parameter error bars; NVIDIA's are a fit to one recording that
# their own tutorial says is "a predictive fit, not a certificate of true joint friction".
# Both stay available and the choice is recorded in every audit report.
FRICTION_SOURCE = "gaz"               # "gaz" | "nvidia"
FRICTION_VISCOUS = FRICTION_VISCOUS_GAZ_CLAMPED if FRICTION_SOURCE == "gaz" else FRICTION_VISCOUS_NVIDIA
FRICTION_COULOMB_STATIC = FRICTION_COULOMB_GAZ
FRICTION_OFFSET = FRICTION_OFFSET_GAZ
FRICTION_COULOMB_DYNAMIC = 0.85 * FRICTION_COULOMB_GAZ   # [CHOICE] dynamic below static
# CORRECTED 2026-11-03 (docs/REVIEW.md F1). An earlier note here said the three-part friction model
# was unreachable and that set_friction_coefficients was the only route. That call takes a
# DIMENSIONLESS PhysX coefficient, so passing these N*m values produced ~38 N*m of friction on
# joint 2. The physics view's set_dof_friction_properties takes a static effort and a dynamic
# effort in N*m and a viscous coefficient in N*m*s/rad; pal_plant.py uses it with
# FRICTION_COULOMB_STATIC, FRICTION_COULOMB_DYNAMIC and FRICTION_VISCOUS below, each times
# PLANT_FRICTION_SCALE. pal_selftest.py T2 verifies 1.0 N*m injected reads back as 1.0 N*m.
#
# ── DECIDED (2026-11-02) ─────────────────────────────────────────────────────────────
# STICTION and BEARING WEAR are deliberately kept INDISTINGUISHABLE. One scalar carries
# both, and no velocity-proportional torque injector will be built to separate them.
#
# Reporting rule that follows from that decision, and it is not optional: wherever this
# friction value or a fault derived from it is named — in the fault list, in the O5
# confusion matrix, in the dashboard alert panel, in the results tables — it must be
# labelled with BOTH classes, e.g. "friction (stiction / bearing wear)". Never with one
# alone. A single label would assert a distinction the plant cannot make, and would make
# the detector look more specific than it is.

# The twin's model uses the values above; the plant gets them scaled. 1.0 makes a matched
# pair, which is useful only while debugging the residual itself. 1.15 is the honest
# setting: a 15 percent unmodelled friction excess is the deliberate mismatch section 9
# asks for.
PLANT_FRICTION_SCALE = 1.15
APPLY_PLANT_FRICTION = True

# ── 6d. Armature (reflected rotor inertia) ────────────────────────────────────────────
# Units are mass*distance^2, i.e. kg*m^2 in this stage, per the PhysX joint schema.
#
# [LITERATURE] ~0.0061 kg*m^2 per joint is the physical reflected rotor inertia of a Panda.
# [NVIDIA]     Isaac Lab's FRANKA_PANDA_LEGACY_CFG uses 1e-3 on all seven arm joints.
#
# These differ by 6x and they are answering different questions. NVIDIA's 1e-3 is a
# numerical stabiliser for stiff drives in an RL setting; 0.0061 is a physical claim about
# the motor. This project measures torque, so it takes the physical value — but see the
# warning below, which is the reason the choice is dangerous either way.
#
# WARNING. Armature enters the SIMULATOR's mass matrix. Because the twin reads M(q) back
# from the articulation API rather than deriving it from the URDF, the two stay consistent
# and the residual is unaffected. An analytic model built from the URDF alone would NOT
# include armature, and would show a constant acceleration-proportional residual that is
# indistinguishable from an unmodelled payload. If the model is ever reimplemented
# analytically, armature must be added to it by hand or set to zero here. This is the most
# likely single cause of a calibration run (section 7.2) that fails for no visible reason.
# MEASURED: the stock asset ships physxJoint:armature = 0.0, observed directly on the prim.
ARMATURE_PHYSICAL = np.array([0.0061] * 7)    # [LITERATURE] Panda reflected rotor inertia
ARMATURE_ISAACLAB = np.array([1e-3] * 7)      # [NVIDIA] Isaac Lab's numerical value
# ── DECIDED (2026-11-02) ──────────────────────────────────────────────────────────────
# Armature STAYS in the plant at the physical 0.0061 kg*m^2, and the analysis side adds the
# matching term to its Pinocchio model (model.armature = 0.0061 on all 7 joints). This is
# option 1 of the three in INTEGRATION.md: the plant stays physically honest and the two
# mass matrices agree by agreement rather than by removal.
#
# This value is now a CONTRACT between two codebases. Changing it here without changing
# Pinocchio reintroduces an acceleration-proportional residual worth about 27 g of phantom
# payload — small against a 3 kg load, 27 percent against a 0.1 kg one, and systematic, so
# averaging will not remove it. Do not change one side alone.
ARMATURE = ARMATURE_PHYSICAL
APPLY_ARMATURE = True
ARMATURE_CONTRACT = "Pinocchio side sets model.armature = 0.0061 on all 7 joints"

# ══════════════════════════════════════════════════════════════════════════════════════
# 6e. Torque convention  —  which side of the gearbox
# ══════════════════════════════════════════════════════════════════════════════════════
# [NVIDIA] The sysid tutorial states plainly that "motor-side, link-side, commanded, and
# measured efforts are not interchangeable", and that its own reference recording declares
# its torque as link-side.
#
# The residual r = tau_measured - tau_model is a subtraction of two torques, so it is only
# meaningful if both are on the same side of the gearbox. Motor-side and link-side differ
# by the gear ratio and by the reflected inertia term. Mixing them produces a residual that
# scales with acceleration and looks precisely like a payload error — the same failure mode
# as the armature warning above, and just as quiet.
#
# This must be established on the first run, not assumed: confirm which convention
# get_measured_joint_efforts() reports, record it here, and state it in the results. The
# static gravity check in pal_scene.py is the cheapest test, since at rest the two
# conventions differ only by the gear ratio and a mismatch shows up as a clean scale factor
# on every joint at once.
TORQUE_CONVENTION = "unverified"      # set to "link_side" or "motor_side" after the audit

# ══════════════════════════════════════════════════════════════════════════════════════
# 6f. Joint drive gains  —  three modes, because the project needs all three
# ══════════════════════════════════════════════════════════════════════════════════════
# What NVIDIA ships, and the trap sitting in the middle of it.
#
# Measured on the live asset (pal_probe_api.py), both conventions side by side on the SAME
# joint prim:
#     USD  drive:angular:physics:stiffness  =   400.0   per DEGREE
#     articulation.get_gains()              = 22918.3   per RADIAN
#
# Ratio exactly 180/pi. So the stock asset's PHYSICAL stiffness is 22918 N*m/rad — about
# 38x a real Panda's 600 N*m/rad joint impedance, and 57x Isaac Lab's intended 400.
#
# The 400 authored into that USD field is almost certainly someone intending Isaac Lab's
# FRANKA_PANDA_HIGH_PD_CFG value of 400 N*m/rad and writing it into a per-degree slot —
# precisely the trap NVIDIA's sysid tutorial warns about ("a gain intended per radian is
# about 57 times too stiff"). It is a good illustration of why this project cannot take a
# single logged number at face value: read the USD and the arm looks correctly tuned; read
# the tensor API and it is 57x stiff. Only reading both resolves it.
#
# What Isaac Lab's configs intend, for comparison (all per radian):
#     FRANKA_PANDA_LEGACY_CFG       stiffness   80.0  damping  4.0
#     FRANKA_PANDA_HIGH_PD_CFG      stiffness  400.0  damping 80.0   (gravity OFF)
#     FRANKA_ROBOTIQ_GRIPPER_CFG    stiffness 1100.0  damping 80.0   (gravity OFF)
#     libfranka joint impedance     600/600/600/600/250/150/50       (the real arm)
#
# Every value below is per RADIAN and is passed to the tensor API unconverted. The scene
# code verifies BOTH conventions on start, because a consistent 57x error satisfies either
# check on its own.
#
# One critical caveat: Isaac Lab's high-PD configs DISABLE GRAVITY, because they are built
# for task-space control where gravity compensation happens elsewhere. This project
# measures g(q) as a physical quantity, so gravity stays on. Copying NVIDIA's gains is
# correct here; copying its gravity setting would be fatal, and silently so.
#
# Why the mode matters for this project specifically: a stiff drive tracks the commanded
# trajectory almost regardless of load. That is good for attestation — the payload shows up
# in torque rather than in position error, which is exactly the signal the residual reads —
# but it makes the section 7.7 validation test impossible, because pushing the gripper of a
# stiff drive moves nothing measurable. That test needs a compliant arm, and this build
# exposes switch_dof_control_mode() / set_effort_modes() as the supported way in.
DRIVE_MODES = {
    # [ASSET] + [NVIDIA] The asset default, equal to Isaac Lab's HIGH_PD gains. Default for
    # objectives O1, O2, O4, O5, O6.
    "asset_stock": {
        "kp": np.array([22918.3] * 7),
        "kd": np.array([4583.7] * 7),
        "note": "what franka.usd ACTUALLY simulates (USD 400/deg); 57x Isaac Lab's intent",
    },
    "position_stiff": {
        "kp": np.array([400.0] * 7),
        "kd": np.array([80.0] * 7),
        "note": "Isaac Lab FRANKA_PANDA_HIGH_PD_CFG as intended (400/80 per rad)",
    },
    # [NVIDIA] Isaac Lab's legacy default. Markedly softer than the asset. Useful as a
    # second operating point for O6: the same trajectory under a softer drive spends its
    # torque differently, and the benchmark should say so.
    "position_soft": {
        "kp": np.array([80.0] * 7),
        "kd": np.array([4.0] * 7),
        "note": "Isaac Lab FRANKA_PANDA_LEGACY_CFG (80/4 per rad)",
    },
    # [DATASHEET] libfranka's documented default joint impedance, N*m/rad. Damping set to
    # 2*sqrt(k) as a critically-damped starting point ([CHOICE] — libfranka publishes no
    # damping vector). This is the only mode here that is per-joint rather than uniform,
    # and it is the one closest to how a real Panda is actually driven.
    "impedance": {
        "kp": np.array([600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0]),
        "kd": 2.0 * np.sqrt(np.array([600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0])),
        "note": "libfranka default joint impedance; per-joint, closest to the real arm",
    },
    # [CHOICE] Pure torque mode for the section 7.7 independent validation: zero stiffness
    # so an applied external force at the gripper actually deflects the arm and the
    # estimated wrench can be compared against the applied one. kd is left small but
    # non-zero; at exactly zero the arm free-falls under gravity the instant it engages,
    # which makes the test unrunnable.
    "torque": {
        "kp": np.zeros(7),
        "kd": np.array([2.0] * 7),
        "note": "low-gain torque mode for the known-push validation (section 7.7)",
    },
}
DEFAULT_DRIVE_MODE = "position_stiff"   # not "asset_stock": that is the 57x-stiff default

# [ASSET] + [NVIDIA] Finger drive. The USD ships panda_finger_joint1 at kp=400 / kd=80 /
# effort=7.2 N and panda_finger_joint2 at all zeros, because joint2 is a mimic of joint1 —
# Isaac Lab encodes that same fact as a separate "panda_finger2_passive" group with
# stiffness and damping 0.0, which is independent confirmation that the zeros are intended
# and not a broken asset.
#
# Isaac Lab sets the finger effort limit to 200.0, well above the asset's 7.2 N and above
# the hand's real 70 N capability; that is RL headroom, not a physical claim. 20 N is used
# here because it is Physyk's proven-working value for a stable grasp. Do not raise it
# without re-checking grasp stability: the fingers are position-driven and a large effort
# limit lets them crush through the contact offsets.
FINGER_KP = 400.0
FINGER_KD = 80.0
FINGER_MAX_EFFORT = 20.0
FINGER_MAX_EFFORT_ISAACLAB = 200.0    # [NVIDIA] for reference; deliberately not used
GRIP_FORCE_RATED_N = 70.0             # [DATASHEET] what the real hand can deliver
FINGER_OPEN = 0.04                    # [DATASHEET] m, per finger (80 mm total stroke)
FINGER_CLOSED = 0.0
GRIPPER_VARIANT = "AlternateFinger"   # [ASSET] NVIDIA's validated grasping variant

# ══════════════════════════════════════════════════════════════════════════════════════
# 7. Payload
# ══════════════════════════════════════════════════════════════════════════════════════
# [DATASHEET] The Panda's rated payload is 3 kg.
#
# [CHOICE] Physyk's cubes are 0.05 kg. That is 0.49 N at the gripper — around 1.6 percent
# of the rated payload, and well under the residual noise floor that finite-difference
# acceleration produces. A payload estimator (O4) cannot converge on it and a detector
# (O5) cannot see it. The ladder below spans the real range so the estimator can be
# characterised rather than merely demonstrated, and so the pose at which it becomes
# unobservable can be found.
PAYLOAD_LADDER_KG = [0.10, 0.25, 0.50, 1.00, 2.00, 3.00]
PAYLOAD_DEFAULT_KG = 0.50
PAYLOAD_SIZE = 0.05                   # m, cube edge — fits the 80 mm finger stroke
PAYLOAD_RATED_MAX_KG = 3.0

# [ASSET] Physyk's tuned contact offsets, kept as-is: they were arrived at empirically to
# stop grasp jitter, and jitter is residual noise here.
CONTACT_OFFSET = 0.004
REST_OFFSET = 0.001

# [ASSET] Physyk's friction materials, unchanged. Grasp friction has to be high enough
# that a 3 kg payload does not slip, since a slipping payload produces a residual
# transient that the detector would class as a fault.
GRASP_STATIC_FRICTION = 1.05
GRASP_DYNAMIC_FRICTION = 0.95
SURFACE_STATIC_FRICTION = 0.70
SURFACE_DYNAMIC_FRICTION = 0.60
RESTITUTION = 0.0

# ══════════════════════════════════════════════════════════════════════════════════════
# 8. Poses
# ══════════════════════════════════════════════════════════════════════════════════════
# [ASSET] Physyk's home pose, which is also the conventional Franka ready pose.
HOME_JOINTS = np.array([0.00, -0.785, 0.00, -2.356, 0.00, 1.571, 0.785])

# [CHOICE] Poses the confidence score of O3 needs in order to be more than a number that
# happens to stay near 0.3. Each is named for what it tests.
PROBE_POSES = {
    # Nominal working pose, well-conditioned Jacobian.
    "home": HOME_JOINTS,
    # Arm stretched nearly straight ahead: the classic elbow singularity, where the
    # smallest singular value collapses and the confidence score should approach zero.
    "near_singular_extended": np.array([0.0, 0.30, 0.0, -0.20, 0.0, 1.00, 0.785]),
    # Wrist singularity: joint5 aligns joint4 and joint6 axes.
    "near_singular_wrist": np.array([0.0, -0.785, 0.0, -2.356, 0.0, 0.00, 0.785]),
    # Folded, high manipulability, worst gravity load on joint2.
    "folded": np.array([0.0, -1.60, 0.0, -2.80, 0.0, 1.40, 0.785]),
    # Arm horizontal: maximum gravity torque on joint2 and joint4, which is where the
    # torque-limit headroom for a 3 kg payload is tightest.
    "horizontal_reach": np.array([0.0, 0.00, 0.0, -1.57, 0.0, 1.57, 0.785]),
}

# ══════════════════════════════════════════════════════════════════════════════════════
# 9. Workcell geometry  —  carried over from Physyk AI unchanged
# ══════════════════════════════════════════════════════════════════════════════════════
# [ASSET] Every one of these is a calibrated constant in the Physyk scene: the table top
# sits at exactly Z=0.00 and the cube rest height, tray height and IK targets are all
# derived from it. Changed here only if the geometry itself changes.
TABLE_POSITION = (0.45, 0.0, -0.05)
TABLE_SCALE = (0.90, 1.30, 0.10)      # top face at Z = 0.00
TABLE_COLOR = (0.16, 0.18, 0.22)
GROUND_PLANE_Z = -0.12                # below the table's own bottom face, avoids z-fighting

TRAY_POSITION = (0.42, -0.32, 0.015)
TRAY_SCALE = (0.22, 0.24, 0.03)
TRAY_COLOR = (0.95, 0.72, 0.12)

ROBOT_PRIM = "/World/Franka"
ROBOT_USD_SUBPATH = "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"

# ══════════════════════════════════════════════════════════════════════════════════════
# Inertial parameters: the shipped asset is NOT dynamically usable
# ══════════════════════════════════════════════════════════════════════════════════════
# MEASURED (pal_inertia_dump.py). The USD asset and the URDF describe different robots:
#
#   mass      differs per link by up to 4.5x (total forced to exactly 18.0000 kg)
#   COM       EVERY body's centre of mass is exactly [0, 0, 0] in the asset, while the
#             URDF has real offsets up to 120 mm (panda_link4)
#   inertia   EVERY body is ~1e-6 kg*m^2. Confirmed by round trip: writing [0.1, 0.2, 0.3]
#             reads back [0.1, 0.2, 0.3] exactly, so the API is in kg*m^2 and the asset
#             really does ship with essentially no rotational inertia. The URDF has
#             ixx = 0.703 on panda_link1; the asset has 0.000002.
#
# Dynamically the shipped asset is a stick figure of point masses sitting on the joint
# axes. That is why g(q) from the asset is exactly zero on joints 3 and 5 at the home
# pose — with every COM on a joint axis, gravity produces no moment about it.
#
# So the plant is AUTHORED from the URDF at scene build. Confirmed to work: after setting
# masses, COMs and inertias from panda_arm_hand.urdf, Isaac Sim produced
#     g(q) = [0, -4.5975, -0.6364, 22.8857, 0.6229, 2.4936, 0.0001]
# against the analysis side's independent Pinocchio model from the same URDF
#     g(q) = [0, -4.58,   -0.64,   22.88,   0.62,   2.50,   0]
# — agreement to rounding on all seven joints, from two independent engines.
#
# This is not cosmetic. Without it the twin and the plant differ by up to 13.65% of a
# joint's torque limit before anything is injected — about 0.6 kg of phantom payload —
# which would be attributed to a payload or a fault. Section 9 calls for a mismatch that
# is CHOSEN and quantified; inheriting this one by accident is the opposite.
URDF_PATH = "/workspace/assets/panda_arm_hand.urdf"
AUTHOR_INERTIAL_FROM_URDF = True
END_EFFECTOR_PRIM = "/World/Franka/panda_rightfinger"
# [ASSET] Fingertip TCP offset along the hand's Z axis. Physyk uses this same constant for
# IK targeting, and it is the point the Jacobian of O2/O3 must be taken at — a wrench
# resolved at the wrong point carries a spurious moment.
TCP_OFFSET = 0.0584

# ── MEASURED: which Jacobian row is the end effector ─────────────────────────────────
# get_jacobians() returns (1, 10, 6, 9) while the articulation reports ELEVEN bodies
# (panda_link0..7, panda_hand, panda_leftfinger, panda_rightfinger). The Jacobian excludes
# the fixed base, so row i corresponds to body i+1:
#     row 6 -> panda_link7     row 7 -> panda_hand
#     row 8 -> panda_leftfinger  row 9 -> panda_rightfinger
#
# Verified by adding a known 3 kg point mass to panda_hand and checking which row
# reproduces the measured change in g(q) through tau = -J_v^T W:
#     row 7 (panda_hand)        max error 0.016 N*m   <- correct
#     row 9 (panda_rightfinger) max error 1.177 N*m   <- wrong by 7%
#
# Row 9 is what a naive [-1] index picks, and it is wrong because panda_rightfinger sits
# about 2.5 cm OFF the hand centreline. Resolving a centreline load at an off-axis frame
# adds a spurious moment, which showed up as ~1 N*m of phantom torque on joints 3 and 5 —
# a standing error that O4 would have absorbed as payload and O5 could have alarmed on.
#
# Note END_EFFECTOR_PRIM is panda_rightfinger: that is correct for the gripper WRAPPER
# (Physyk uses it for IK targeting) but wrong for the Jacobian. They are different things
# and must not be conflated.
EE_JACOBIAN_ROW = 7          # panda_hand
EE_JACOBIAN_BODY = "panda_hand"

# ══════════════════════════════════════════════════════════════════════════════════════
# 10. Camera and lighting  —  carried over from Physyk AI exactly
# ══════════════════════════════════════════════════════════════════════════════════════
# [ASSET] The isometric three-quarter overview camera. Position, look-at and focal length
# are all reproduced exactly: this framing was arrived at over several correction passes
# in the Physyk project (too cropped, then too wide, then settled at 22 mm), and the
# dark-blue grid ground behind it is what gives the view its backdrop.
CAM_POSITION = (1.85, -1.55, 1.30)
CAM_LOOK_AT = (0.35, 0.0, 0.28)
CAM_FOCAL_LENGTH = 22.0
CAM_HORIZONTAL_APERTURE = 20.955      # USD Camera schema default
CAM_RESOLUTION = (1280, 720)          # [CHOICE] 720p; Physyk streams 960x540

# No wrist camera. Physyk mounts one on panda_hand, but this project's views (section 8)
# are all whole-arm — torque share per link, the gripper ellipsoid, estimated force arrows —
# and a wrist view shows none of that. Its config is deliberately absent rather than left
# unused, so nobody re-enables it by accident and pays for a second render product.

# [ASSET] The blue backdrop. Isaac's default ground plane asset ships a light-blue grid
# whose colour is not a parameter on add_default_ground_plane(); it has to be retinted on
# the loaded material's diffuse_tint shader input. This is the value Physyk settled on.
GROUND_TINT = (0.10, 0.16, 0.34)

# [ASSET] The "studio" lighting preset, reproduced exactly.
LIGHTS = {
    "dome": {"intensity": 700.0},
    "overhead": {"intensity": 2400.0, "color": (1.0, 0.98, 0.95),
                 "position": (0.46, 0.0, 1.85), "radius": 0.85},
    "key": {"intensity": 1600.0, "color": (1.0, 0.95, 0.90), "rotation": (-45.0, 30.0, 0.0)},
    "fill": {"intensity": 550.0, "position": (1.10, -0.80, 1.30), "radius": 0.40},
    "wrist": {"intensity": 380.0, "color": (1.0, 1.0, 0.98), "radius": 0.05,
              "translate": (0.0, 0.0, 0.06)},
}

# ══════════════════════════════════════════════════════════════════════════════════════
# 11. Attestation thresholds
# ══════════════════════════════════════════════════════════════════════════════════════
# [CHOICE] All of these are the proposal's own stated pass conditions, pulled out so the
# evaluation table of section 9 is filled in from the same constants the code runs on.
CALIBRATION_TOLERANCE = 0.05          # section 7.2: model within 5 percent per joint
CONFIDENCE_THRESHOLD = 0.08           # section 7.3: below this, a pose is low-trust.
                                      # Starting value only — set it from the measured
                                      # distribution of sigma_min/sigma_max over the
                                      # workspace, not from taste.
RESIDUAL_ALARM_NM = 2.0               # section 7.5: per-joint residual alarm level
RESIDUAL_ALARM_CONSECUTIVE = 5        # consecutive steps above the level before alarming
PAYLOAD_FILTER_ALPHA = 0.02           # running filter on the payload estimate (O4)
