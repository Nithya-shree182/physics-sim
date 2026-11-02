# Franka Panda configuration — what was taken, from where, and why

Written for: whoever has to defend a residual number in this project.

The residual `r = τ_measured − τ_model` is a subtraction of two torques. It is only
meaningful if every parameter feeding either side is known, so each one below carries its
source. The settings live in [pal_config.py](workspace/pal_config.py); this file is the
argument for them.

Four sources were used:

| Tag | Source |
|---|---|
| `[DATASHEET]` | Franka Emika Panda published specification |
| `[ASSET]` | What `franka.usd` / `panda_arm_hand.urdf` actually ship, read off a live run of the Physyk AI service and off the URDF |
| `[NVIDIA]` | Isaac Lab's own `franka.py` asset configs, and NVIDIA's Franka system-identification tutorial |
| `[LITERATURE]` | Gaz et al. 2019, Franka dynamic identification |

---

## 1. The finding that changes the most: a unit trap in the shipped asset

The Physyk AI logs show stock arm gains of `kp = 22918.3`, `kd = 4583.7`. Read as N·m/rad
those are ~38× a real Panda's 600 N·m/rad joint impedance.

Reasoning about the convention from documentation alone gives the wrong answer, in either
direction. Probing the live asset settles it, because **both conventions sit side by side
on the same joint prim**:

```
USD  drive:angular:physics:stiffness  =   400.0    per DEGREE
articulation.get_gains()              = 22918.3    per RADIAN
```

Ratio exactly 180/π. So the tensor API reports SI, and the **stock asset's physical
stiffness really is 22918 N·m/rad** — 57× Isaac Lab's intended 400.

The `400.0` authored into that USD field is almost certainly someone intending Isaac Lab's
`FRANKA_PANDA_HIGH_PD_CFG` value of 400 N·m/rad and writing it into a per-degree slot —
exactly the trap NVIDIA's own sysid tutorial warns about: *"a gain intended per radian is
about 57 times too stiff."*

This is the single best argument for why this project cannot take a logged number at face
value. **Read the USD and the arm looks correctly tuned at 400. Read the tensor API and it
is 57× too stiff. Only reading both resolves it** — and a 57× gain error produces a
residual that looks exactly like a large unmodelled payload, i.e. it would be misread as a
successful O4 payload estimate or an O5 fault.

`pal_config.py` therefore states every gain per radian, passes it to the tensor API
unconverted, and `pal_scene.py` **verifies both conventions on every start** — the tensor
readback must match what was written *and* the USD attribute must be 57.3× smaller. Either
check alone can be satisfied by a consistent 57× error.

`DRIVE_MODES` keeps `asset_stock` (22918, what the asset actually simulates) as a distinct,
non-default entry so the difference stays visible rather than becoming folded into
"the default".

## 2. What NVIDIA actually ships for the Franka

Isaac Lab's `franka.py` carries several configs, and they disagree with each other enough
that "the NVIDIA settings" is not a single thing:

| Config | Stiffness | Damping | Effort | Velocity limit | Armature | Gravity |
|---|---|---|---|---|---|---|
| `FRANKA_PANDA_LEGACY_CFG` | 80.0 | 4.0 | 87 / 12 | not set | 1e-3 | on |
| `FRANKA_PANDA_LEGACY_HIGH_PD_CFG` | 400.0 | 80.0 | 87 / 12 | not set | 1e-3 | **off** |
| `FRANKA_PANDA_CFG` (flat asset) | None → USD's 400/80 | None | 87 / 12 | 20 / 25 rad/s | not set | on |
| `FRANKA_PANDA_HIGH_PD_CFG` | 400.0 | 80.0 | 87 / 12 | 20 / 25 rad/s | not set | **off** |
| `FRANKA_ROBOTIQ_GRIPPER_CFG` | 1100 / 1000 | 80.0 | 5200 / 720 | 2.175 / 2.61 | not set | **off** |

Three things worth taking from that table:

- **The usable stiffness range NVIDIA itself spans is 80 to 1100 N·m/rad.** The asset
  default sits at 400. `pal_config.py` exposes 400 (`position_stiff`), 80
  (`position_soft`), libfranka's per-joint 600/250/150/50 (`impedance`), and a near-zero
  torque mode, because the project needs different ones for different objectives.
- **Every high-PD config disables gravity**, since they are built for task-space control
  where gravity compensation happens elsewhere. This project *measures* `g(q)` as a
  physical quantity. Copying NVIDIA's gains is right; copying its gravity flag would be
  fatal, and quietly so.
- **The flat asset's velocity limits are ~10× the datasheet** (20/25 rad/s against
  2.175/2.61). That is RL headroom, not a physical claim — and NVIDIA's own Robotiq config
  uses the datasheet values instead. The datasheet values are used here: a trajectory
  benchmark (O6) run at ten times the arm's real speed limit would be measuring nothing.

Effort limits `[87, 87, 87, 87, 12, 12, 12]` N·m agree across the datasheet, the URDF, the
live readback and every Isaac Lab config. That is the one parameter nothing disputes.

## 3. The stock arm has no joint friction. Three sources agree.

- `[ASSET]` `panda_arm_hand.urdf` declares `friction="0.0"` on all nine revolute joints.
- `[NVIDIA]` The sysid tutorial: the packaged `franka.usda` *"authors no Newton joint
  friction at all, so every arm joint would otherwise run at the solver default."*
- `[NVIDIA]` Isaac Lab's `FRANKA_PANDA_CFG` sets joint friction explicitly to viscous 0.0.

This is the proposal's §11 risk *"model matches the simulator too well"* already present in
the asset before a line is written. A frictionless plant compared against a frictionless
model gives a residual bounded only by solver noise, so **every detection rate measured
against the stock asset would be optimistic and meaningless.** Friction has to be put into
the plant deliberately.

### Which friction values

NVIDIA's sysid tutorial recovers viscous coefficients for a real Panda against a 9,439-sample
MCAP recording, as scales on a 1.0 N·m·s/rad seed:

```
J1 0.214   J2 0.105   J3 0.848   J4 0.184   J5 0.172   J6 0.147   J7 0.156
```

These are the primary values (`FRICTION_SOURCE = "nvidia"`): fitted inside this simulator
family, in its own units, against this exact asset. Gaz et al.'s identified set is kept as
the alternative, because it is the only source supplying **both** Coulomb and viscous terms
for this robot.

The two disagree substantially — joint 3 is 0.848 under NVIDIA's fit and 0.0399 under
Gaz's. **They are not averaged.** They are two fits to two different robots by two
different methods, and the disagreement is itself the honest statement of how well joint
friction on a Panda is actually known. Re-identify against this simulator before any
reported result leans on the choice.

NVIDIA's own caveat carries over verbatim: the recording *"carries no authored friction
value for the optimizer to rediscover"*, so the recovered number *"is a predictive fit, not
a certificate of true joint friction."*

### Why friction is modelled in three parts

PhysX splits joint friction into static Coulomb effort (N·m), dynamic Coulomb effort (N·m)
and a viscous coefficient (torque·s/degree). The project keeps them separate because they
have **different fault signatures**: a stiffening joint raises Coulomb first, since it
resists breakaway, while bearing wear tends to raise viscous. Collapsing them into one
scalar throws away exactly the distinction the O5 fault classifier is meant to learn.
`pal_scene.py` writes all three via `PhysxJointAxisAPI`, and falls back to the legacy
single-scalar `jointFriction` — recording in the audit report that the viscous term was
lost, since that makes the fault classes harder to separate.

## 4. Two quiet ways the residual can be wrong

Both of these produce an acceleration-proportional residual that is **indistinguishable
from an unmodelled payload**. Either one would be read as a successful O4 estimate.

**Armature.** Units are kg·m². The physical reflected rotor inertia of a Panda is
~0.0061 kg·m²; Isaac Lab uses 1e-3, which is a numerical stabiliser rather than a physical
claim. The physical value is used here, since this project measures torque. But armature
enters the **simulator's** mass matrix — harmless while the twin reads `M(q)` back from the
articulation API, and silently fatal the moment anyone reimplements the model analytically
from the URDF, because a URDF-derived `M(q)` has no armature in it. This is the single most
likely cause of a §7.2 calibration run that fails for no visible reason.

**Torque convention.** NVIDIA states plainly that *"motor-side, link-side, commanded, and
measured efforts are not interchangeable"*, and that its own reference recording declares
link-side torque. Motor-side and link-side differ by the gear ratio. `TORQUE_CONVENTION` is
set to `"unverified"` and must stay that way until the first run settles it — the static
gravity check is the cheapest test, because at rest a convention mismatch shows up as a
clean common scale factor on all seven joints at once, which the audit detects and reports
separately from a genuine modelling error.

## 5. Deliberate departures from the Physyk AI scene

Carried over **unchanged**, because they are calibrated constants that other things depend
on: table at `(0.45, 0, −0.05)` scale `(0.90, 1.30, 0.10)` with its top face at exactly
Z = 0.00; tray; ground plane at Z = −0.12; friction materials; contact/rest offsets
0.004/0.001; the `AlternateFinger` gripper variant; the isometric camera at
`(1.85, −1.55, 1.30)` looking at `(0.35, 0, 0.28)` at 22 mm; the studio lighting preset;
and the dark-blue grid tint `(0.10, 0.16, 0.34)`.

Changed on purpose:

| | Physyk AI | Here | Why |
|---|---|---|---|
| Physics rate | 60 Hz | **400 Hz** | `q̈` comes from a finite difference of velocity, and that is the dominant residual noise source (§11 risk 2). The real Franka loop is 1 kHz; 400 Hz is a midpoint that leaves GPU headroom. Overridable to 1000. |
| Rendering | 60 Hz, coupled | **30 Hz, decoupled** | Rendering is for the human. Telemetry publishes every *physics* step. |
| Solver iterations | 16 / 4 | **32 / 4** | Isaac Lab's Franka uses 8/0, tuned for RL throughput where per-step torque accuracy is irrelevant. Here joint force *is* the measurement, so an unconverged solver shows up as a residual the detector reads as a fault. |
| Stabilization | off | **off, explicitly** | It injects non-physical damping, indistinguishable from the O5 friction fault. Never enable it. |
| Determinism | default | **enhanced** | The output is a comparison between runs. |
| Joint friction | none | **three-part, ×1.15** | See §3. The 15% excess is the §9 twin/plant mismatch. |
| Payload | 0.05 kg cube | **0.1–3.0 kg ladder** | 0.05 kg is 0.49 N, ~1.6% of rated payload and under the residual noise floor. An estimator cannot converge on it; a detector cannot see it. The rated max is 3 kg. |
| Gripper mass | not separated | **`TOOL_MASS` = 0.93 kg** | Hand 0.73 + 2 × 0.1 fingers. An O4 estimator that fails to subtract this reports every empty gripper as carrying 0.93 kg. |

## 6. What the first run has to establish

`pal_scene.py` ends in an audit rather than a demo, because three things are assumed by
every later objective and none of them are verified yet:

1. **Which decomposed-dynamics quantities this build exposes** — `M(q)`, `g(q)`,
   `C(q,q̇)q̇`, `J(q)`, measured efforts. O1 and O2 rest entirely on these. The audit probes
   each by name and reports shape and finiteness. If any are missing they must be computed
   analytically from the URDF — and then armature has to be added by hand (§4).
2. **Whether the arm at rest reports `τ_measured = g(q)`.** With `q̇ = q̈ = 0` the dynamics
   reduce to the gravity term, so this is the cheapest possible version of the §7.2
   calibration test. A failure here is a configuration error, and every later residual
   would be measuring that bug instead of a physical effect.
3. **The spread of `c(q) = σ_min(J)/σ_max(J)` over real poses.** `CONFIDENCE_THRESHOLD` is
   a guess (0.08) until this exists. The survey runs five named poses — nominal, elbow-
   singular, wrist-singular, folded, horizontal-reach — each chosen for the condition it
   tests. Set the threshold from a full workspace sweep, not from these five.

Everything lands in `out/physics_audit.json` plus an isometric render at
`out/scene_isometric.png`.

---

## Sources

- [Isaac Lab `isaaclab_assets` Franka configs](https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab_assets/isaaclab_assets/robots/franka.py)
- [Tutorial 14: System Identification of a Franka — Isaac Sim docs](https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup_tutorials/tutorial_franka_system_identification.html)
- [System Identification Extension — Isaac Sim docs](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/robot_setup/ext_isaacsim_robot_setup_sysid.html)
- [Actuators — Isaac Lab docs](https://isaac-sim.github.io/IsaacLab/develop/source/concepts/actuators.html)
- [PhysX joint schema (friction and armature units)](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/joints/physx_joint_schema.html)
- [`PxJointFrictionParams` — PhysX 5.6 API](https://nvidia-omniverse.github.io/PhysX/physx/5.6.0/_api_build/structPxJointFrictionParams.html)
- [What is the Model of Joint Friction? — NVIDIA developer forums](https://forums.developer.nvidia.com/t/what-is-the-model-of-joint-friction/238318)
- [Torque control robotic arm — Isaac Lab issue #2355](https://github.com/isaac-sim/IsaacLab/issues/2355)
- [Setting motor friction identified from no-load — Isaac Lab discussion #3456](https://github.com/isaac-sim/IsaacLab/discussions/3456)


---

# Measured results — first audit run

Isaac Sim 6.0.1, RTX 5090 Laptop, ~45 s. Full report in `out/physics_audit.json`.

## Dynamics API: 7/7 available

```
✓ M(q)                     get_mass_matrices()                  (1, 9, 9)
✓ g(q)                     get_generalized_gravity_forces()     (1, 9)
✓ C(q,qdot)qdot            get_coriolis_and_centrifugal_forces()(1, 9)
✓ J(q)                     get_jacobians()                      (1, 10, 6, 9)
✓ tau_measured             get_measured_joint_efforts()         (1, 9)
✓ joint reaction wrenches  get_measured_joint_forces()          (1, 11, 6)
✓ tau_commanded            get_applied_joint_efforts()          (1, 9)
```

M(q) is `get_mass_matrices()`, **not** `get_generalized_mass_matrices()` — the first audit
reported it missing purely because of the wrong name. Objective O1 is fully feasible from
the simulator; nothing needs to be derived analytically from the URDF, which also means the
armature trap in §4 stays dormant.

The articulation also exposes `switch_dof_control_mode()`, `set_effort_modes()` and
`set_joint_efforts()` — the supported route into a real torque mode for the §7.7 known-push
validation, better than zeroing position gains (which leaves the drive in position mode,
still fighting the applied force).

## Static gravity check: PASS, but a weak test

| joint | g(q) | τ_measured | residual |
|---|---|---|---|
| joint2 | −1.6548 | −1.6548 | 1e-05 |
| joint4 | 20.0771 | 20.0771 | −2.5e-05 |
| joint6 | 0.8557 | 0.8557 | 7e-06 |

Agreement to ~1 part in 10⁶. **What this does and does not prove:** at rest both quantities
come from the same PhysX articulation, so they are consistent by construction. It rules out
sign flips, degree/radian errors and finger-DOF index misalignment — which is what it is
for — but it is **not** a validation of the dynamics model. That requires the §7.2 moving
calibration run, where `M(q)q̈` and `C(q,q̇)q̇` actually participate.

The torque-convention diagnostic is **inconclusive**: joints 1/3/5/7 carry ~zero gravity
load so their ratios are `nan` or noise. `TORQUE_CONVENTION` stays `"unverified"`; it needs
a loaded, moving test.

## Pose conditioning: c(q) needs rescaling before it can be used

```
pose                        sigma_min  sigma_max     c(q)  manipulability
home                          0.22245    1.80490   0.1232       8.014e-02
near_singular_extended        0.03812    1.92621   0.0198       1.189e-02
near_singular_wrist           0.18016    1.73650   0.1037       4.150e-02
folded                        0.19061    1.77960   0.1071       4.165e-02
horizontal_reach              0.14763    1.85641   0.0795       8.980e-02
```

Two findings that affect the proposal's §6 as written:

1. **`c(q)` never exceeds 0.13 anywhere, including at home.** The proposal says "a score
   near 1 means the pose is healthy." On a 7-DOF arm that never happens, because
   σ_max/σ_min on a mixed translation/rotation Jacobian is dominated by the unit mismatch
   between its linear (m/s) and angular (rad/s) rows. The raw ratio needs either
   length-scaling of the rotational rows or renormalising against the workspace maximum —
   otherwise every pose reads as near-singular and the O3 gate suppresses everything.
2. **`horizontal_reach` has the highest manipulability yet the second-lowest `c(q)`.** Not a
   bug — the two metrics genuinely disagree, which makes it a good concrete example for the
   §8 "where is the arm strong" view.

The elbow-singular pose does collapse to 0.0198, so the metric works directionally. The
0.08 threshold was a guess and is clearly misplaced; set it from a workspace sweep after
the rescaling fix.

## Confirmed directly on the stock asset prim

```
jointFriction        = 0.0
physxJoint:armature  = 0.0
```

A **fourth** independent confirmation of the zero-friction finding, and the first for zero
armature.

## Limitation found: friction cannot be split three ways

This build has **no `PhysxJointAxisAPI`**, so PhysX's three-part friction model (static
Coulomb / dynamic Coulomb / viscous) is unreachable. The articulation tensor API
(`set_friction_coefficients`) takes **one scalar per joint**.

Consequence for O5, recorded rather than hidden: a Coulomb-type fault (stiffening, resists
breakaway) and a viscous-type fault (bearing wear, grows with speed) both arrive as a change
in the same scalar and **cannot be separated from the plant model alone**. To recover the
distinction, the viscous term must be injected as an applied torque proportional to joint
velocity in the control loop.

## Housekeeping

- `panda_finger_joint2` ships `kp = kd = effort = 0`, so the finger patch fires — matching
  Isaac Lab's `panda_finger2_passive` group exactly. Not a broken asset.
- The container logs `CPU performance profile is set to powersave`, which will cost
  throughput on long runs.
- Cosmetic: the table renders light grey rather than the dark `(0.16, 0.18, 0.22)` set on
  the `FixedCuboid`; the colour is not reaching the material.


---

# Update — 2026-11-02: the asset's inertial parameters

Everything above concerns gains, friction, limits and armature. A later audit found a
larger problem underneath all of it: **the shipped `franka.usd` is not a dynamically
credible Franka.**

| quantity | asset | URDF |
|---|---|---|
| centre of mass, every body | exactly `[0,0,0]` | offsets up to 120 mm |
| inertia diagonal, every body | ~1e-6 kg·m² | e.g. `ixx = 0.703` on link1 |
| per-link mass | differs by up to 4.5× | — |
| total mass | exactly 18.0000 kg | 17.6218 kg |

Round-trip verified: writing `[0.1, 0.2, 0.3]` reads back exactly, so the API is in kg·m²
and the asset really does have no rotational inertia. Dynamically it was a stick figure of
point masses sitting on the joint axes — which is why its `g(q)` is exactly zero on joints
3 and 5 at the home pose, a detail the analysis side spotted from their own Pinocchio
output before we had looked.

**The scene now authors mass, COM and inertia from `panda_arm_hand.urdf`**
(`AUTHOR_INERTIAL_FROM_URDF` in `pal_config.py`). After that, Isaac Sim and an independent
Pinocchio model agree to rounding on all seven joints. The accidental mismatch removed was
13.65% of joint 6's torque limit, about 0.6 kg of phantom payload.

Two further corrections to this document:

- **The Jacobian row.** `get_jacobians()` excludes the fixed base, so its last row is
  `panda_rightfinger`, 2.5 cm off the hand centreline. Use row 7 (`panda_hand`): max error
  against a known payload drops from 1.177 to 0.016 N·m.
- **`world.reset()` wipes tensor-API writes.** Masses, COMs, inertias, friction and
  armature all revert. Apply after the final reset, and verify after it.
