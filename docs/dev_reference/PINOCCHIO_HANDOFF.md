# Building the independent model — a handoff to the analysis side

You are building the twin's dynamics model in Pinocchio. This document is everything the
simulator side has established about the arm, written so you can build against it without
waiting for log files, plus the traps we have already fallen into so you don't repeat them.

Everything here was measured on Isaac Sim 6.0.1 with
`Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd`, unless marked otherwise.

---

## 1. The one thing that must not be got wrong

**Your model must not know about the payload.** That is the entire reason your side exists.

We tested the alternative. Reading `g(q)` back from the simulator while it carries a 3 kg
object gives:

```
tau_measured - g(q)_from_simulator  =  [0, -0.0006, 0, 0.0001, 0, 0, 0]
```

Zero, because the simulator's gravity term already contains the payload. Comparing against
it cancels exactly the signal we are trying to measure. Your Pinocchio model, built from a
URDF that describes an empty arm, is the only thing in the system that provides an
independent prediction. Keep it that way: do not "improve" it by feeding it the payload.

The same applies to the gripper. `panda_hand` (0.73 kg) and two fingers (0.1 kg each) are
**part of the robot**, not part of the payload. They are in the URDF and your model must
include them, or every empty gripper will report 0.93 kg of phantom load.

## 2. Minimal working setup

```python
import numpy as np, pinocchio as pin

model = pin.buildModelFromUrdf("panda_arm_hand.urdf")
data  = model.createData()

model.gravity.linear = np.array([0.0, 0.0, -9.81])
model.armature       = np.full(model.nv, 0.0061)     # CONTRACT — see section 3

tau_model = pin.rnea(model, data, q, qd, qdd)        # M(q)qdd + C(q,qd)qd + g(q)
```

Split terms, if you need them for the torque-breakdown chart (proposal section 8):

```python
g   = pin.computeGeneralizedGravity(model, data, q)   # gravity alone
M   = pin.crba(model, data, q)                        # mass matrix alone
nle = pin.nonLinearEffects(model, data, q, qd)        # C·qd + g
```

`pin.crba` fills only the upper triangle. Symmetrise before using it:
`M = np.triu(M) + np.triu(M, 1).T`.

**URDF location.** `panda_arm_hand.urdf` (307 lines, 9 joints, arm + hand) currently lives
in a sibling project at `Physyk_AI-master/assets/franka_panda/`. It should be copied into
this repo — depending on a path outside it is fragile. `panda.urdf` (216 lines) is the arm
without the hand; do not use it, the 0.93 kg of gripper matters.

## 3. Contracts — values that must match on both sides

Changing either of these on one side alone silently corrupts the residual.

### 3.1 Armature = 0.0061 kg·m² on all seven joints

**DECIDED 2026-11-02.** The plant keeps the physical reflected rotor inertia; you add the
matching term in Pinocchio.

A Panda joint drives through a gearbox of ratio `N`. When the joint accelerates at `q̈` the
motor rotor accelerates at `N·q̈`, and the torque to spin it up is `N²·I_rotor·q̈`. That
`N²·I_rotor` is the armature, and it adds to the **diagonal of the mass matrix**:

```
tau = [ M(q) + diag(a) ] qdd  +  C qd  +  g
```

**URDF has no field for rotor inertia** — it describes links and joints, not motors. So
`pin.crba` from a bare URDF returns `M_rigid` only, and you must set `model.armature`
explicitly.

Size of the error if you don't: `0.0061 · q̈`, up to **0.12 N·m** at joint 6/7's 20 rad/s²
acceleration limit. Small against the 5% calibration tolerance — but it is proportional to
`q̈`, which makes it **indistinguishable from payload**, because a payload also appears as
extra inertia proportional to `q̈`. Using the lever arm we measured (0.472 m at home) it is
worth about **27 g of phantom payload**: negligible against a 3 kg load, **27% against the
0.1 kg bottom of the test ladder**, and systematic, so averaging will not remove it.

For reference, Isaac Lab's own Franka config uses `1e-3`. That is a numerical stabiliser,
not a physical claim. Do not mix it into a physical model.

### 3.2 Gravity = −9.81 m/s² in +Z

Set explicitly on both sides. Note that Isaac Lab's `FRANKA_PANDA_HIGH_PD_CFG` **disables
gravity** — it is built for task-space control where gravity compensation happens
elsewhere. We do not do that; `g(q)` is a measured physical quantity here.

## 4. Friction — Pinocchio does not model it

`rnea` is rigid-body dynamics only. Friction is yours to add:

```python
tau_model = pin.rnea(model, data, q, qd, qdd) + friction(qd)
```

### What the asset ships

**Nothing.** Four independent confirmations: the URDF declares `friction="0.0"` on every
revolute joint; NVIDIA's sysid tutorial states the packaged `franka.usda` "authors no
Newton joint friction at all"; Isaac Lab's `FRANKA_PANDA_CFG` sets viscous friction to 0.0;
and we read `jointFriction = 0.0` directly off the joint prim.

This matters for honesty of results: a frictionless plant compared against a frictionless
model gives a residual bounded only by solver noise, and any detection rate measured that
way is meaningless. The simulator side therefore injects friction deliberately.

### Which values

From Gaz et al. 2019, read directly from the supplementary PDF in `docs/references/`
(not from memory, not from a text extraction). **The paper gives two models and they are
not interchangeable.**

**Model A — Table I, the one the paper actually fits.** A sigmoid in joint velocity:

```
tau_f,j(qd) = phi1 / (1 + exp(-phi2*(qd + phi3)))  -  phi1 / (1 + exp(-phi2*phi3))
```

| | j1 | j2 | j3 | j4 | j5 | j6 | j7 | units |
|---|---|---|---|---|---|---|---|---|
| phi1 | 0.54615 | 0.87224 | 0.64068 | 1.27940 | 0.83904 | 0.30301 | 0.56489 | N·m |
| phi2 | 5.1181 | 9.0657 | 10.136 | 5.5903 | 8.3469 | 17.133 | 10.336 | s/rad |
| phi3 | 0.039533 | 0.025882 | −0.046070 | 0.036194 | 0.026226 | −0.021047 | 0.0035526 | rad/s |

The second term makes `tau_f(0) = 0` exactly. **Use this one.** A `sign()` term chatters
around zero velocity, and the arm sits near zero velocity at every waypoint of a
pick-and-place — precisely where chatter would look like a residual spike and be
classified as a fault. Implemented as `friction_sigmoid()` in `workspace/pal_config.py`.

**Model B — Table III, linear.** `tau_f = fv·qd + fc·sign(qd) + fo`

| | j1 | j2 | j3 | j4 | j5 | j6 | j7 |
|---|---|---|---|---|---|---|---|
| fv | 0.0665 | 0.1987 | 0.0399 | 0.2257 | 0.1023 | **−0.0132** | 0.0638 |
| fc | 0.2450 | 0.1523 | 0.1827 | 0.3591 | 0.2669 | 0.1658 | 0.2109 |
| fo | −0.1073 | −0.1566 | −0.0686 | −0.2522 | 0.0045 | 0.0910 | −0.0127 |

The paper adopts this form only "because of their linearity in the defining parameters" and
warns it "may suffer from sudden discontinuities in the neighbourhood of qd = 0".

**Do not omit `fo`.** It is a real identified parameter — a constant per-joint torque that
does not vanish at zero velocity — and it is easy to miss. Dropping it biases the residual
by up to 0.25 N·m on joint 4, which a payload estimator absorbs as ~25 g of phantom mass.

**Read the error bars before leaning on any of these.** `fv,6 = −0.0132` has a standard
deviation of **12.26** — the estimate is noise, and negative viscous friction is physically
impossible. We clamp it to zero. `fo,5` (std 19.46) and `fo,7` (std 6.26) are likewise not
meaningful.

### A caveat on all of it

The manufacturer publishes no dynamic parameters for the Panda. Gaz et al. is one robot's
identification from 2019. NVIDIA's sysid tutorial produces different numbers again, and
says of its own result that it is "a predictive fit, not a certificate of true joint
friction". Treat friction as the least-known thing in the model, and expect the residual to
carry the difference.

### What this means for the fault list

**DECIDED: stiction and bearing wear are kept indistinguishable.** PhysX on this build
exposes one friction scalar per joint (no `PhysxJointAxisAPI`), so a Coulomb-type fault
and a viscous-type fault arrive as the same change.

**Reporting rule, not optional:** wherever this fault class appears — fault list, O5
confusion matrix, dashboard alert panel, results tables — label it with **both** names,
`friction (stiction / bearing wear)`. Never one alone. A single label would assert a
distinction the plant cannot make.

## 5. Arm parameters

### Link masses (from the URDF; cross-check your parse against these)

| link | kg |
|---|---|
| panda_link0 (base, fixed) | 0.6297 |
| panda_link1 | 4.970684 |
| panda_link2 | 0.646926 |
| panda_link3 | 3.228604 |
| panda_link4 | 3.587895 |
| panda_link5 | 1.225946 |
| panda_link6 | 1.666555 |
| panda_link7 | 0.735522 |
| panda_hand | 0.73 |
| each finger | 0.1 |

Total ≈ 18.0 kg, matching the datasheet. Full 3×3 inertia tensors are present for all 11
links, so this is a real dynamic model and not a point-mass approximation.

### Limits (datasheet; the URDF agrees exactly)

| | j1 | j2 | j3 | j4 | j5 | j6 | j7 |
|---|---|---|---|---|---|---|---|
| q min (rad) | −2.8973 | −1.7628 | −2.8973 | −3.0718 | −2.8973 | −0.0175 | −2.8973 |
| q max (rad) | 2.8973 | 1.7628 | 2.8973 | **−0.0698** | 2.8973 | 3.7525 | 2.8973 |
| qd max (rad/s) | 2.175 | 2.175 | 2.175 | 2.175 | 2.61 | 2.61 | 2.61 |
| qdd max (rad/s²) | 15 | 7.5 | 10 | 12.5 | 15 | 20 | 20 |
| tau max (N·m) | 87 | 87 | 87 | 87 | 12 | 12 | 12 |

**Joint 4 lives entirely at negative angles** and joint 6 cannot go below −0.0175. The
reachable set is not a box centred on zero, which matters when you sample poses for the
confidence sweep.

Isaac Lab's flat asset sets velocity limits of 20/25 rad/s — roughly 10× the datasheet.
That is RL headroom, not physics. We use the datasheet values.

### Rated payload: 3 kg

The test ladder is 0.1–3.0 kg. An earlier scene used 0.05 kg cubes — that is 0.49 N, about
1.6% of rated payload, and under the residual noise floor. No estimator can converge on it.

## 6. Reading the telemetry correctly

### `tau_applied` is identically zero in position mode

Measured. `get_applied_joint_efforts()` returns whatever was last written by
`set_joint_efforts()`, and in position mode nothing ever is — the PD loop is internal to
PhysX and never routes through that call.

With a 3 kg payload at a fixed pose:

```
g(q) change      : [0, -9.573, 0, 14.688, 0, 2.730, 0]
tau_measured     : [0, -9.574, 0, 14.688, 0, 2.730, 0]   <- tracks it exactly
tau_applied      : [0,  0,     0,  0,     0, 0,     0]   <- does not move
```

**Consequence:** `r = tau_applied − tau_model` on a position-mode log reduces to
`−tau_model`. It will appear to detect the payload perfectly while containing no
measurement at all. In effort mode the two agree to within 0.44 N·m and either is fine.

**Guard for your code:** read the control mode from the run manifest and refuse a
position-mode log if the residual definition is `tau_applied`.

### The Jacobian's last row is not the end effector

`get_jacobians()` returns `(1, 10, 6, 9)` while the articulation reports **eleven** bodies.
The Jacobian excludes the fixed base, so **row i ↔ body i+1**:

```
row 6 -> panda_link7      row 7 -> panda_hand
row 8 -> panda_leftfinger row 9 -> panda_rightfinger
```

Verified by adding a known 3 kg mass to `panda_hand` and checking which row reproduces the
measured change in `g(q)` via `tau = -J_v^T W`:

| row | body | max error |
|---|---|---|
| **7** | **panda_hand** | **0.016 N·m** |
| 9 | panda_rightfinger | 1.177 N·m |

**Use row 7.** Row 9 is what a naive `[-1]` picks, and it is wrong because
`panda_rightfinger` sits ~2.5 cm off the hand centreline — resolving a centreline load at
an off-axis frame adds a spurious moment, which showed up as ~1 N·m of phantom torque on
joints 3 and 5. That is worth ~215 g of phantom payload.

Note `END_EFFECTOR_PRIM` in the scene config is `panda_rightfinger`. That is correct for the
**gripper wrapper** (used for IK targeting) and wrong for the **Jacobian**. Different
things; do not conflate them.

### Angular quantities in USD are per DEGREE

If you ever read drive gains or friction coefficients off the stage rather than through the
tensor API, remember USD expresses angular drive stiffness and viscous friction **per
degree**. The shipped asset authors `400.0` into a per-degree field, which simulates as
22918 N·m/rad — 57× what was presumably intended. NVIDIA's own tutorial warns about exactly
this. The tensor API (`get_gains`) reports per radian.

## 7. Reference values to build against

**Updated 2026-11-02 — the earlier reference vector in this document was wrong.** It gave
the *unauthored asset's* `g(q)`, which came from a robot with all centres of mass at the
link origins and no rotational inertia. Ignore it if you saw it.

### The plant is now authored from the URDF

Your cross-check was right, and it found something larger than the hand. A full per-body
dump of the shipped `franka.usd`:

| quantity | asset | URDF |
|---|---|---|
| centre of mass, **every** body | exactly `[0, 0, 0]` | offsets up to 120 mm |
| inertia diagonal, **every** body | ~1e-6 kg·m² | e.g. `ixx = 0.703` on link1 |
| per-link mass | differs by up to 4.5× | — |
| total mass | exactly 18.0000 kg | 17.6218 kg |

The inertia figure was verified by round trip — writing `[0.1, 0.2, 0.3]` reads back
`[0.1, 0.2, 0.3]` exactly, so the API is in kg·m² and the asset genuinely ships with
essentially no rotational inertia. Dynamically it was a stick figure of point masses on
the joint axes, which is precisely why our `g(q)` was exactly zero on joints 3 and 5 while
yours was not.

**The scene now overwrites mass, COM and inertia from `panda_arm_hand.urdf` at build
time.** With that in place:

```
your Pinocchio (URDF)  : [0, −4.58,  −0.64,  22.88,  0.62,  2.50,  0]
Isaac Sim (authored)   : [0, −4.663, −0.636, 22.954, 0.623, 2.441, 0.001]
```

Agreement to rounding on all seven joints, from two independent engines. **Your model is
correct and needs no change.** Use your own numbers as the reference.

The accidental mismatch we were carrying before this was **13.65% of joint 6's torque
limit** — roughly 0.6 kg of phantom payload, which the estimator would have reported as
load and the detector might have alarmed on.

### A caveat neither of us can resolve

Authoring from the URDF makes the twin and the plant agree; it does not make either one
*correct*. Franka publishes no dynamic parameters — Gaz et al. opens by saying so.
`franka_description`'s values are estimates and NVIDIA's are different estimates. We have
picked one so that the mismatch is chosen rather than inherited, which is what section 9
requires. No result should claim these are the real robot's parameters.

### Residual floor at rest, with the authored plant

```
joint         g(q)    tau_measured   residual   % of limit
panda_joint2  -4.663    -4.796        -0.133      0.15%
panda_joint3  -0.636    -0.638        -0.002      0.00%
panda_joint4  22.954    23.030         0.076      0.09%
panda_joint5   0.623     0.627         0.004      0.03%
panda_joint6   2.441     2.450         0.009      0.08%
```

The residual is no longer ~1e-6 as it was before, because joint friction is now actually
reaching the plant. What you see here is the friction signature at rest, which is the
baseline O5 must detect a *change* in.

### Payload response (measured, at home)

Gravity torque is exactly linear in payload mass: **4.6306 N·m per kg** on joint 4,
maximum departure from a straight line **0.00038 N·m** across 0–5 kg. Effective lever arm
at the tool centre: **0.472 m**.

The full estimation chain — residual → `(J_vᵀ)⁺` → vertical force → mass — recovers known
masses to **0.04%** across 0.1–3.0 kg when the arm is at rest. That is the ceiling, not the
operating point: during motion you additionally face finite-difference `q̈` noise, friction,
and the armature term.

## 8. The confidence score needs rescaling before you use it

Measured across five named poses with the corrected Jacobian row:

| pose | σ_min | σ_max | c(q) | manipulability |
|---|---|---|---|---|
| home | 0.22485 | 1.80571 | 0.1245 | 7.962e-02 |
| near_singular_extended | 0.03817 | 1.90949 | **0.0200** | 1.202e-02 |
| near_singular_wrist | 0.17780 | 1.74304 | 0.1020 | 4.115e-02 |
| folded | 0.18970 | 1.77999 | 0.1066 | 4.094e-02 |
| horizontal_reach | 0.14971 | 1.85476 | 0.0807 | 9.016e-02 |

Two problems.

**`c(q)` never exceeds 0.13 anywhere, including the home pose.** The proposal says "a score
near 1 means the pose is healthy"; on a 7-DOF arm that never happens, because σ_max/σ_min
on a mixed Jacobian is dominated by the unit mismatch between its linear (m/s) and angular
(rad/s) rows. Your plan already says "scaled so position and orientation rows are
unit-consistent" — that is exactly right and it is necessary, not cosmetic. Scale the
rotational rows by a characteristic length (the 0.472 m lever arm is a reasonable choice)
before taking the SVD.

**A scalar may be the wrong gate for O4.** We held a 1 kg object at the near-singular
extended pose and the estimate was still accurate to 0.06% — because that singularity
collapses some direction of the Jacobian but **not the vertical one**, and vertical is all
the mass estimate uses. The gate was conservative, not protective. A direction-specific
confidence — how much of the vertical axis lies in the Jacobian's well-conditioned subspace
— would gate O4 far more precisely than a single `σ_min/σ_max`. Worth raising before the
score is finalised.

The 0.08 threshold currently in the config is a guess. Set it from a workspace sweep.

## 8b. If you ever drive the simulator yourself

`world.reset()` **silently discards everything written through the tensor API** — body
masses, centres of mass, inertias, joint friction, armature. The writes succeed, the
readbacks confirm them, and a later reset reverts all of it to the asset's values.

This cost us real time: for a while the log reported URDF parameters applied while every
downstream number was the stock asset. Apply tensor-API values **after** the final reset
and verify **after** it; a readback taken immediately after the write proves nothing.
`pal_scene.py` prints a `[survive]` line for this.

## 9. What is verified, and what is not

| Claim | Status |
|---|---|
| All 7 dynamics quantities available from the sim | **verified** (note: `get_mass_matrices()`, not `get_generalized_mass_matrices()`) |
| `tau_measured = g(q)` at rest | **verified**, 1 part in 10⁶ |
| `g(q)` and `J(q)` mutually consistent | **verified**, 0.016 N·m with row 7 |
| Payload response linear, estimator chain correct | **verified**, 0.04% at rest |
| Three-term identity closes on a slow trajectory | **verified**, 0.49% of limit — but that run was 98% gravity |
| Three-term identity closes under fast motion | **FAILED at 5.98%** on the unauthored plant; needs re-measuring now that the plant has real inertia |
| Friction reaches the plant | **YES**, fixed — `world.reset()` had been wiping it |
| Plant and Pinocchio agree on g(q) | **YES** — agreement to rounding on all 7 joints |
| Telemetry at 1 kHz without dropped frames | **NO** — 0.42× real time |
| An independent model exists | **NO** — this is your job; every `tau_model` so far came out of Isaac Sim |

That last row is the important one. Until your Pinocchio model lands, this system can
verify its own arithmetic but cannot attest to anything, because the thing being measured
and the thing measuring it are the same simulator.


---

# Update — 2026-11-03

> **CORRECTION 2026-11-03:** the "time discretization, first order in dt" finding and its
> 200/500/1000/2000 Hz figures below are wrong (the sweep ignored `physics_dt`). The
> residual is rate-independent below ~50% of velocity limit and grows with rate above it.
> See `docs/RESIDUAL_FLOOR.md` and the correction in `docs/OPEN_PROBLEMS.md` (P1).


## The residual has a floor, and your model cannot remove it

The fast-motion residual we reported as an open failure is **time discretization of a
stiff position drive**, not a modelling error on either side. Measured on a 100%-of-limit
excitation trajectory, worst joint as a fraction of its torque limit:

```
200 Hz -> 50.91%     500 Hz -> 14.24%     1000 Hz -> 2.78%     2000 Hz -> 1.44%
```

Fitting RMS residual against `dt` gives an exponent of **1.049** — first order in `dt`.
Two controls rule out the alternatives: solver iteration count changes nothing (14.24% at
4 iterations and at 128), and armature changes nothing (2.3692 vs 2.3892), which also
confirms armature *is* correctly inside the reported `M`. Residual rises with drive
stiffness for the same reason — a stiffer drive is a stiffer ODE.

Five hypotheses were eliminated by measurement before this one: timing misalignment,
Coriolis sign or definition, finite-difference noise, velocity-limit saturation, and the
mass-matrix slice.

**What this means for you.** At 1000 Hz with `position_stiff` the floor is roughly 2.8% of
a joint's torque limit on fast motion, and well under 1% on slow motion. Detection
thresholds for O5 must sit above it, and it should be reported alongside any detection
rate. It is a property of the plant's integration, so a perfect model still sees it.

It is also why `PHYSICS_HZ` stays at 1000 even though a full telemetry frame costs ~1.29 ms
and therefore cannot stream in real time at that rate. Decimating `M`/`J` is the trade.

## Payload attachment — corrected, and our two numbers reconciled

Our 4.896 N·m/kg figure was wrong. We attached the payload by raising `panda_hand`'s mass
and wrote `hand_mass + 3.0` assuming the URDF's 0.730 kg hand, but the asset's hand was
0.558 kg — so that line added **3.172 kg**, not 3.0. 13.89 / 3.172 = **4.631**, which
matches our other figure. `INTEGRATION.md` has been corrected.

**The attachment point has moved to match yours.** It was at the hand origin; the payload
is now folded in as a point mass at **0.1034 m along `panda_hand`'s z axis** — combined
mass, combined COM, and the payload's parallel-axis inertia contribution about that
combined COM. Recorded in each run manifest under `payload_attachment`.

## Telemetry: your schema adopted unchanged

`schema/telemetry_v0.yaml` and `schema/validate.py` are on `dev` as-is. The writer is
`workspace/pal_telemetry.py`, driven by `workspace/pal_capture.py`.

Your four questions are answered in `docs/REPLY_TO_ANALYSIS.md` §3. In short: label
columns accepted as written; `qdd_cmd_*` and `push_force_*` accepted and written; your
`mj_fresh` decimation encoding adopted; and the timing convention is measured to be
immaterial (pre-step / post-step / midpoint give 27.99 / 27.98 / 27.98 percent), with the
row assembled after `world.step()` and that stated in the manifest.

**One deviation, in delivery only:** the Isaac Sim container has neither pandas nor
pyarrow, so the writer emits compressed `.npz` and `tools/npz_to_parquet.py` converts on
the host. You still receive Parquet in your schema.

## Corrections to section 9 of this document

| claim | now |
|---|---|
| "Three-term identity closes under fast motion — FAILED at 5.98%" | **Explained.** Time discretization, first order in `dt`. ~2.8% at 1 kHz. Not a model error. |
| "Friction reaches the plant — NO" | **Fixed.** `world.reset()` was wiping it. |
| "An independent model exists — NO" | **Yours does.** Your gravity vector is now the reference, and the plant reproduces it to rounding. |


---

# The operating envelope — this sets your detection thresholds

Mapped on 2026-11-03 (`workspace/pal_fast_random.py`, `out/residual_floor.json`). The
discretization floor is **not one number**: it has a knee at ~50% of velocity limit, and
the knee is wrist-specific.

Worst-joint RMS residual as a fraction of that joint's torque limit, 1000 Hz:

| speed | j1 | j2 | j3 | j4 | j5 | j6 | j7 | worst |
|---|---|---|---|---|---|---|---|---|
| 10% | 0.09 | 0.31 | 0.39 | 0.20 | 0.23 | 0.24 | 0.01 | **0.39%** |
| 25% | 0.24 | 0.78 | 0.96 | 0.53 | 0.54 | 0.60 | 0.03 | **0.96%** |
| 50% | 0.54 | 1.57 | 1.81 | 1.14 | 1.03 | 1.27 | 0.07 | **1.81%** |
| 75% | 1.43 | 3.15 | 3.45 | 2.20 | **8.80** | **7.79** | **8.01** | 8.80% |
| 100% | 3.87 | 5.79 | 6.59 | 4.45 | **27.79** | **25.31** | **20.54** | 27.79% |

Joints 5/6/7 have 12 N·m limits rather than 87 **and** much lower inertia, so the same
absolute discretization error is a larger fraction and the lighter links make a stiffer
ODE. Both effects land on the same three joints.

Confirmed over 24 random poses, phases and speeds (correlation with speed 0.596). The
scatter at frac 0.75 — 2.47% to 28.68% — means the floor depends on **pose as well as
speed**, so a single scalar threshold will be wrong somewhere in the workspace.

### Converted into payload terms

At 4.63 N·m/kg on joint 4:

| speed | floor at j4 | smallest detectable payload | 3-sigma threshold |
|---|---|---|---|
| 10% | 0.174 N·m | 0.038 kg | 0.113 kg |
| 25% | 0.457 | 0.099 | 0.296 |
| 50% | 0.987 | 0.213 | 0.640 |
| 75% | 1.917 | 0.414 | 1.242 |
| 100% | 3.870 | 0.836 | **2.507** |

**At full speed most of the 0.1-3.0 kg ladder is invisible.** Our recommendation, and we
think it is close to forced: run O4 and O5 at or below 50% of velocity limit.

### A conflict worth agreeing on now

O6 benchmarks trajectory styles and wants fast motion. O2/O4/O5 need slow motion to stay
below the knee. They cannot share a speed regime. O6 should report cycle time, peak torque
and energy at speed, while any residual-based claim stays below the knee — and the results
should say which regime each number came from rather than mixing them.


---

# CORRECTION 2026-11-03 — the residual floor, measured properly

**This supersedes the envelope table sent earlier in this document, and the "time
discretization, first order in dt" claim. Both were wrong.** Full detail in
`docs/RESIDUAL_FLOOR.md`.

The earlier rate sweep rebuilt the `World` once per rate inside a single process, which
does not give a clean plant — your R2.3 instinct to question it was right, and the
inconsistency you spotted (27.98% and 2.78% both at 1000 Hz) was the symptom. Re-run with
one container per rate, identical settle/warm-up/measurement windows in simulated seconds,
and trajectory amplitude a function of speed only:

| speed | 250 Hz | 500 Hz | 1000 Hz | 2000 Hz | rate-dependent? |
|---|---|---|---|---|---|
| 10% | 0.43% | 0.43% | 0.43% | 0.43% | no (x1.00) |
| 20% | 0.85% | 0.86% | 0.86% | 0.86% | no (x1.00) |
| 30% | 1.26% | 1.27% | 1.27% | 1.27% | no (x1.00) |
| 50% | **2.05%** | **2.05%** | **2.05%** | **2.05%** | no (x1.00) |
| 75% | 3.08% | 4.29% | 8.79% | 17.96% | **YES (x5.8)** |
| 100% | 7.26% | 14.85% | 30.21% | 53.76% | **YES (x7.4)** |

**Below ~50% the residual is physical** — identical to two decimals across an 8x range of
physics rate, and linear in speed. **Above ~50% a rate-dependent artifact takes over**,
roughly doubling per doubling of rate, reaching a peak of 112% of a joint's torque limit at
2000 Hz / full speed.

This is strong support for your R2.3 hypothesis. Below the knee the arm tracks well and the
drive is gentle; above it tracking falls to ~54% of commanded velocity, the drive works
hard, and the rate-dependent term appears. That is an implicit-drive signature, not the arm.

Also ruled out by measurement, so you need not re-test them: finite-difference noise
(analytic qdd gives the same answer), filtering (2%), timing alignment, Coriolis form,
solver iterations (4 vs 128), armature, velocity saturation, and mass-matrix slicing.

### For your thresholds

| speed | floor at j4 | smallest detectable payload | 3-sigma threshold |
|---|---|---|---|
| 10% | ~0.13 N*m | 0.03 kg | 0.09 kg |
| 30% | ~0.41 | 0.09 | 0.27 |
| 50% | ~0.69 | 0.15 | 0.45 |
| 100% (1 kHz) | ~3.70 | 0.80 | 2.40 |

**Recommendation: record O2/O4/O5 data at or below 50% of velocity limit.** Below the knee
the whole 0.1-3.0 kg ladder is detectable and the number does not depend on a simulator
setting.

### Your R2.3 test is still the right next step

We have not yet run effort mode. If the above-knee residual collapses there, it belongs to
the position drive and all attestation data should be recorded in effort mode.

### Confirmations from a full windowed run

- payload response **4.629 N*m/kg** at joint 4 with the TCP attachment, against your 4.60
- an unknown **2 kg** object weighed from torque alone to 0.01-0.03% at three poses
- your scaled confidence formulation reproduced in the simulator: raw c = 0.1238 at home,
  scaled c = 0.2127, c_vertical = 0.2686, force gain 4.85 — and the elbow-singular pose
  correctly gated at a force gain of 27.16
