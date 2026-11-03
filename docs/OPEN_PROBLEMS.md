# Open problems and what we work on next

Status as of 2026-11-02. Everything here was measured, not assumed; each item says what was
observed and what it blocks.

---

## Blocking problems

### P1 — The three-term identity fails under fast motion — **CORRECTED 2026-11-03: not "time discretization"**

> **Correction.** The "SOLVED, first order in dt" conclusion below is wrong. Its rate sweep
> (200/500/1000/2000 Hz -> 50.91/14.24/2.78/1.44%) came from `pal_residual_diag.py`, which
> rebuilds `World` inside one process; Isaac warns that `physics_dt` is ignored on
> subsequent constructions, so the rates were not what they claimed. Measured properly
> (one rate per process, `pal_sweep.py`, see `RESIDUAL_FLOOR.md`) the residual **grows**
> with physics rate above ~50% of velocity limit (30.21% at 1 kHz, 53.76% at 2 kHz) and is
> rate-independent below it. `pal_rate_decouple.py` agrees (27.79% at 1 kHz, 35.26% at
> 2 kHz) and also shows the **controller** rate is irrelevant: 2 kHz physics with a 1 kHz
> or 500 Hz controller gives 35.31% / 35.45%. Only the 500 Hz row (14.24%) and the
> solver-iteration / armature / mass-matrix controls from the old text still stand.
> The accepted explanation is the implicit-drive hypothesis in `RESIDUAL_FLOOR.md`.
> The text below is kept as the historical record.

**It is time discretization, not a modelling error.** Measured on a 100%-of-limit
excitation trajectory, worst joint as a fraction of its torque limit:

```
200 Hz -> 50.91%     500 Hz -> 14.24%     1000 Hz -> 2.78%     2000 Hz -> 1.44%
```

Fitting RMS residual against `dt` gives an exponent of **1.049** — first order in `dt`,
the signature of discretization. Two controls:

- **solver iterations change nothing** — 14.24% at 4, 14.22% at 128. Not convergence.
- **armature changes nothing** — 2.3692 vs 2.3892. So armature *is* correctly inside the
  reported `M`, which also closes a worry from the handoff.

And from the other side, **residual rises with drive stiffness** (kp 80 → 12.17%,
kp 5000 → 33.87%): a stiffer drive is a stiffer ODE and discretizes worse at a given `dt`.

Five hypotheses were eliminated along the way, each by measurement rather than argument:
timing misalignment (pre/post/midpoint state give 27.99/27.98/27.98%), Coriolis sign or
definition (the assumed form wins; `C` is exactly zero at rest), finite-difference noise
(filtering changes it 2%), velocity-limit saturation (the sweep never saturates), and the
mass-matrix slice (full 9×9 identical to the 7×7 block).

**Consequence, which is not a bug but a limit:** there is a residual floor set by the
physics rate and the drive stiffness. At 1000 Hz with `position_stiff` it is ~2.8% of a
joint's torque limit on fast motion and well under 1% on slow motion. O5 detection
thresholds must sit above it, and it must be reported alongside any detection rate.

Original report follows.

#### P1 (original) — the three-term identity fails under fast motion

A multi-sine excitation trajectory on all seven joints, with velocities driven toward the
datasheet limits, gives:

| joint | \|q̇\| max | \|M q̈\| | \|C q̇\| | \|g\| | RMS residual | % of limit |
|---|---|---|---|---|---|---|
| j1 | 2.18 | 2.807 | 0.491 | 0.000 | 1.3545 | 1.56% |
| j2 | 1.33 | 3.539 | 1.301 | 6.663 | 2.8976 | 3.33% |
| **j3** | 2.18 | 4.218 | 1.181 | 4.092 | **5.2021** | **5.98%** |
| j4 | 1.57 | 2.381 | 1.147 | 19.300 | 3.2504 | 3.74% |

**FAIL** against the 5% tolerance on joint 3. Note the residual (5.20 N·m) is *larger than
the acceleration term it is supposed to explain* (4.22 N·m) — this is not a small modelling
imperfection.

Filtering `q̈` changed nothing (−0%), so it is **not** finite-difference noise, which was
the suspected cause and is now ruled out.

Leading hypothesis: a half-step misalignment between `tau_measured` and the `(q, q̇)` used
to evaluate the model. We read all quantities after `world.step()`, but PhysX may report
the measured effort for the step just completed while reporting state after it. At 1 kHz
with accelerations in the tens of rad/s², half a step is significant.

Other candidates not yet excluded: a sign convention on the Coriolis vector; the measured
effort including the drive's internal damping term; armature not actually present in this
articulation.

**Blocks:** O2 calibration sign-off, and therefore everything downstream. The slow-run PASS
at 0.49% is not sufficient evidence — that run was 98% gravity.

### P2 — ~~Joint friction never reaches the plant~~ **FIXED 2026-11-02, then CORRECTED 2026-11-03: it reaches the plant in the wrong units**

> **Correction.** Friction does reach the plant, but `set_friction_coefficients` takes a dimensionless
> PhysX coefficient and we passed N*m values, so joint 2 carries ~38 N*m instead of ~0.2. The torque-valued
> API is `art._physics_view.set_dof_friction_properties`. Also, `tau_measured` is net torque and cannot
> show friction at all. See `docs/REVIEW.md` F1 and F2.
>
> **Fixed later the same day:** `pal_plant.py` uses the torque-valued API (verified, `pal_selftest.py` T2) and fault
> data is recorded in effort mode with `tau_applied`.


**Root cause was worse than friction.** `world.reset()` silently discards *everything*
written through the tensor API — body masses, centres of mass, inertias, joint friction
and armature all revert to the asset's values. The writes succeed and the readbacks
confirm them; a later reset throws it all away. For part of the session the log claimed
URDF inertial parameters were applied while every downstream number was the stock asset.

**Fix:** removed the `world.reset()` that ran after the configuration block, and added a
`[survive]` verification line that re-reads masses, friction and armature *after* the last
reset. Verified:

```
[survive] body masses kept: True   friction nonzero: True   armature: [0.0061 x7]
```

**Rule going forward:** apply tensor-API values after the final reset, and verify after it.
A readback taken immediately after the write proves nothing.

Original report follows.



The excitation test with Gaz friction applied returned **byte-identical** numbers to the
frictionless run; residual growth was exactly `[0,0,0,0,0,0,0]`.

Cause is almost certainly ordering: we call `set_friction_coefficients()` and then
`world.reset()`, which appears to restore articulation properties from USD and wipe the
tensor-API write. **`pal_scene.py` has the same ordering**, so the friction it prints as
"applied" with a successful readback may not survive into the run.

**Blocks:** every O5 number. The whole point of injecting friction is that a frictionless
plant versus a frictionless model gives a meaningless detection rate.

**Fix:** move friction and armature application to *after* the final `world.reset()`, and
verify by reading back post-reset and by confirming a residual change against the
frictionless baseline — a successful readback immediately after the write proved nothing.

### P3 — No independent model exists

Every `tau_model` produced so far has come out of Isaac Sim. The system can currently verify
its own arithmetic but cannot attest to anything, because the measurer and the measured are
the same simulator.

**Owner:** analysis side (Pinocchio). See `docs/PINOCCHIO_HANDOFF.md`.

### P4 — Telemetry cannot run at 1 kHz — **decided: keep 1 kHz, decimate M/J**

Throughput and residual pull in opposite directions, and the residual wins because it is
the measurement.

A full frame costs ~1.29 ms to read **regardless of physics rate** — the readback
dominates, not the solver — so throughput is flat at about 780 steps/s (1000 Hz → 0.76×
real time, 500 Hz → 1.52×, 400 Hz → 1.94×). Lowering the rate to buy real-time telemetry
would multiply the residual floor by five (P1).

`PHYSICS_HZ` was briefly set to 500 and has been **reverted to 1000**. The trade is made
on the readback instead: `M` (49 columns) and `J` (42) are 57% of the frame and dominate
its cost, and both change very little in a millisecond. `pal_telemetry.py` implements the
analysis side's proposed encoding — `mj_every=N` with an `mj_fresh` flag.

Original report follows.

#### P4 (original) — telemetry cannot run at 1 kHz

| | steps/s | vs real time |
|---|---|---|
| physics only | 1,769 | 1.77× |
| + full telemetry every step | **422** | **0.42×** |

Telemetry costs 319% on top of the physics. Per-step mean 2.369 ms, p99 3.519 ms, against a
1.000 ms budget — **100% of steps over budget**. O1's stated pass condition is "no dropped
frames across a full run", and at 1 kHz that is not currently met.

`M` (49 floats) and `J` (42) are 57% of the frame and almost certainly most of the cost.

Headless dataset generation is **unaffected** — it just takes 2.4× wall-clock per simulated
second, and 4.6 GB/hour at float64 is manageable. **O7's live dashboard cannot stream at
1 kHz.**

**Decision needed:** decimate `M`/`J` to every Nth step (physically defensible — `M` changes
very little per millisecond, but the residual then uses a slightly stale `M`), lower the
physics rate, or accept slower-than-real-time with a decimated live view.

---

## Correctness questions

### P5 — Torque convention unverified

NVIDIA states plainly that "motor-side, link-side, commanded, and measured efforts are not
interchangeable", and that its own reference recording declares link-side torque. These
differ by the gear ratio.

`TORQUE_CONVENTION` is still `"unverified"`. The static check cannot resolve it: the loaded
joints all gave ratio 1.000, but the unloaded ones give only noise. Needs a **loaded,
moving** test.

### P6 — ~~Hand mass discrepancy~~ **RESOLVED — it was not just the hand**

The 0.8 N·m discrepancy was the visible corner of a much larger problem, raised by the
analysis side: their Pinocchio model gave `[0, −4.58, −0.64, 22.88, 0.62, 2.50, 0]` at
home against our `[0, −1.655, 0, 20.077, 0, 0.856, 0]`, and they correctly inferred from
our exact zeros on joints 3 and 5 that the centres of mass differed, not just the hand's
mass.

**A full per-body dump confirmed the asset is not a dynamically credible Franka:**

| quantity | asset | URDF |
|---|---|---|
| centre of mass, every body | exactly `[0,0,0]` | offsets up to 120 mm |
| inertia diagonal, every body | ~1e-6 kg·m² | e.g. `ixx = 0.703` on link1 |
| per-link mass | differs by up to 4.5× | — |
| total mass | exactly 18.0000 kg | 17.6218 kg |

The inertia figure was verified by round trip — writing `[0.1, 0.2, 0.3]` reads back
`[0.1, 0.2, 0.3]` exactly, so the API is in kg·m² and the asset really does ship with
essentially no rotational inertia. Dynamically it is a stick figure of point masses
sitting on the joint axes, which is exactly why gravity produces no moment about joints 3
and 5 at the home pose.

**Fix:** the scene now authors mass, COM and inertia from `panda_arm_hand.urdf` at build
time (`AUTHOR_INERTIAL_FROM_URDF`). After authoring:

```
Pinocchio (URDF)    : [0, −4.58,  −0.64,  22.88,  0.62,  2.50,  0]
Isaac Sim (authored): [0, −4.663, −0.636, 22.954, 0.623, 2.441, 0.001]
```

Agreement to rounding across two independent engines. The accidental mismatch being
carried before this was **13.65% of joint 6's torque limit** — about 0.6 kg of phantom
payload, which would have been attributed to a payload or a fault.

### P6b — Which parameter set is physically right is still unknown

Authoring from the URDF makes the twin and plant agree, but it does not make either
*correct*. Franka publishes no dynamic parameters — Gaz et al. 2019 opens by saying so.
`franka_description`'s URDF values are estimates; NVIDIA's are different estimates. We
have chosen one so the mismatch is controlled, which is what section 9 requires, but no
claim should be made that these are the real robot's parameters.

### P6c (superseded) — original hand-mass report

At the home pose with no payload, two measurements of `g(q)` disagree:

```
unmodified asset             : [0, -1.655, 0, 20.077, 0, 0.856, 0]
after re-authoring hand mass : [0, -2.176, 0, 20.874, 0, 1.004, 0]
```

0.797 N·m on joint 4, 0.521 N·m on joint 2 — about 4% on joint 2. The asset's natural
`panda_hand` mass may differ from the URDF's 0.73 kg, or re-authoring the attribute may
change how the fingers are accounted for.

**Matters because** the handoff document gives the first vector as the reference B should
validate their Pinocchio model against. If the asset and the URDF disagree about the hand,
the twin and the plant start out mismatched in a way nobody chose.

### P7 — `c(q)` is unusable as written

Never exceeds 0.13 anywhere, including the home pose, so the proposal's "near 1 means
healthy" never occurs. Caused by the unit mismatch between the Jacobian's linear (m/s) and
angular (rad/s) rows.

Separately: a scalar `σ_min/σ_max` may be the wrong gate for O4. We held 1 kg at the
near-singular extended pose and the estimate was accurate to 0.06% — the singularity
collapses some Jacobian direction but not the vertical one, and vertical is all the mass
estimate uses. The gate was conservative, not protective.

**Blocks:** O3, which gates O4 and O5. The 0.08 threshold is a guess.

---

## Known and accepted

### A1 — Stiction and bearing wear are indistinguishable
**DECIDED.** One PhysX friction scalar per joint; no velocity-proportional injector will be
built. **Reporting rule:** always label this class `friction (stiction / bearing wear)`,
never one alone.

### A2 — A faithfully-executed tampered waypoint produces no residual
If the arm is told to go somewhere wrong and goes there correctly, the dynamics are
consistent and both residual definitions stay near zero. Needs tracking error `q_cmd − q`
as a third signal. Already identified in `WORK_SPLIT.md`; must be stated explicitly in
results as an attack class the residual does not cover.

### A3 — `tau_applied` is unusable in position mode
Identically zero. The residual definition is therefore coupled to the control mode. Record
the mode per run and have the residual code refuse a mismatched log.

---

## Cosmetic

- The table renders light grey instead of the dark `(0.16, 0.18, 0.22)` set on the
  `FixedCuboid`; the colour is not reaching the material.
- The container logs `CPU performance profile is set to powersave`, which will cost
  throughput on long headless runs.
- Isaac Sim hangs on startup roughly 1 start in 5, with empty `docker logs` and an
  `unhealthy` container. `docker rm -f pal-isaac-sim` and restart.

---

## Additional findings from 2026-11-02

### F1 — The Jacobian's last row is not the end effector (FIXED)
`get_jacobians()` returns 10 rows while the articulation reports 11 bodies; the Jacobian
excludes the fixed base, so row *i* maps to body *i+1*. A naive `[-1]` picks
`panda_rightfinger`, which sits ~2.5 cm off the hand centreline and injects a spurious
moment. Verified against a known 3 kg mass:

| row | body | max error vs Δg(q) |
|---|---|---|
| **7** | **panda_hand** | **0.016 N·m** |
| 9 | panda_rightfinger | 1.177 N·m |

That 1.18 N·m error was worth ~215 g of phantom payload. Now `EE_JACOBIAN_ROW = 7`.

Note `END_EFFECTOR_PRIM` is still `panda_rightfinger` — correct for the gripper wrapper
used in IK targeting, wrong for the Jacobian. Different things.

### F2 — `tau_applied` is identically zero in position mode
`get_applied_joint_efforts()` returns whatever `set_joint_efforts()` last wrote, and in
position mode nothing ever does. With 3 kg held, `tau_measured` tracked the payload
exactly while `tau_applied` stayed at zero on every joint.

Consequence: `WORK_SPLIT.md`'s residual `r = tau_applied − tau_model` reduces to
`−tau_model` on a position-mode log. It would appear to detect the payload perfectly while
containing no measurement. In effort mode the two agree to within 0.44 N·m.

**The residual definition is therefore coupled to the control mode.** Record the mode per
run and have the residual code refuse a mismatched log.

### F3 — The 57× gain trap is in the shipped asset
USD stores angular drive stiffness **per degree**; the tensor API reports **per radian**.
Both conventions sit on the same joint prim:

```
USD  drive:angular:physics:stiffness =   400.0   per degree
articulation.get_gains()             = 22918.3   per radian
```

So the asset simulates at 22918 N·m/rad — 57× Isaac Lab's intended 400. NVIDIA's own
sysid tutorial warns about exactly this. The scene now verifies both conventions on every
start, because a consistent 57× error satisfies either check alone.

### F4 — The slow calibration PASS was misleading
The first moving test passed at 0.49% of the torque limit, but the term breakdown shows
why that proves little: gravity averaged 3.81 N·m, acceleration 0.069 N·m, Coriolis
0.0105 N·m. The run was ~98% gravity, so it validated `g(q)` and essentially nothing about
`M` or `C`. The excitation test that does exercise them fails (P1).

### F5 — Filtering does not help the fast-motion residual
Low-pass filtering the finite-difference `q̈` changed the RMS residual by −0%. The leading
suspect for P1 was finite-difference noise; it is now ruled out.

### F6 — A scalar confidence score may be the wrong gate for O4
A 1 kg object held at the near-singular extended pose was estimated to 0.06% accuracy and
still gated out. The singularity collapses some Jacobian direction but not the vertical
one, and vertical is all the mass estimate uses. A direction-specific confidence would
gate O4 far more precisely than `σ_min/σ_max`.

### F7 — Isaac Sim boot hang, ~1 in 5
Empty `docker logs`, container `unhealthy`, GPU idle. Not a slow run — the process never
starts. `docker rm -f` and relaunch. A watchdog loop is in `CONTEXT.md`; cost about 20
minutes across this session before being scripted around.

---

## What we work on next

In order. The first two are prerequisites for trusting any later number.

**1. ~~Fix P2~~ — done.** Tensor-API writes now survive, verified by a `[survive]` line.

**2. ~~Diagnose P1~~ — solved.** Time discretization; see above. Test the half-step hypothesis by
sampling state before and after the step and evaluating the model both ways. If that is not
it, check the Coriolis sign convention and whether armature is actually present in the
articulation. **Nothing downstream is trustworthy until this closes.**

**3. ~~Settle P6~~ — done.** Resolved into the inertial authoring above.

**4. ~~Write the schema~~ — the analysis side already had one.** `schema/telemetry_v0.yaml`
and `schema/validate.py` are adopted unchanged on `dev`. The writer
(`workspace/pal_telemetry.py` + `workspace/pal_capture.py`) is implemented against it.
Remaining: validate a captured run end-to-end against `schema/validate.py`.

**5. Rescale the Jacobian and sweep the workspace for a real `c(q)` threshold.** Scale the
rotational rows by the 0.472 m characteristic length, then sample a few thousand poses to
set the threshold from a distribution instead of taste. Also prototype the direction-specific
confidence for O4.

**6. Wire effort-control mode** via `switch_control_mode("effort")` — verified to work. Note
the arm drifts within a few hundred steps with no controller holding it, so the validation
push needs either a short window or an outer gravity-compensation loop.

**7. Link-mass perturbation** for the deliberate twin/plant mismatch. Friction scaling (×1.15)
is already in place; masses are not.

Then the objectives proper: trajectory generators (O6), fault and attack injectors (O5),
dashboard (O7).


---

# Added 2026-11-03

### P8 — The Isaac container has neither pandas nor pyarrow

So the telemetry writer cannot emit Parquet directly. It writes compressed `.npz` with
numpy alone and `tools/npz_to_parquet.py` converts on the host (`.venv-tools`). The
analysis side still receives Parquet in their schema; only the conversion point moved.

### P9 — `data/` must be world-writable for the container

The container runs as **uid 1234**, not root and not the host user. A host directory
created with the usual 755 permissions is not writable by it, and the failure surfaces
only at the end of a run as `PermissionError` after all the compute is done.
`chmod 777 data` before capturing. Worth folding into the launcher.

### P10 — The boot hang is getting worse, not better

Three hangs in a row during the capture work, against roughly 1 in 5 earlier. Each costs
about 3 minutes of watchdog timeout. A long-lived container that scripts are sent into
would remove both this and the ~60 s boot cost per probe; currently every probe pays the
lottery.

### F8 — The effective-stiffness test was invalid

Section A of `pal_residual_diag.py` reported `k_eff` near zero and sometimes negative. The
cause is the method: `set_joint_positions()` appears to move the drive target as well, so
there was no tracking error to restore from. The underlying question — why a 0.14 rad
tracking error produced only ~10 N·m against a 400 N·m/rad drive — is unresolved, but it
matters much less now that discretization accounts for the residual.


---

# SUPERSEDED — read docs/RESIDUAL_FLOOR.md first

Everything below about the residual floor predates the controlled sweep of 2026-11-03 and
parts of it are **wrong**. The authoritative result is `docs/RESIDUAL_FLOOR.md`:

| speed | 250 Hz | 500 Hz | 1000 Hz | 2000 Hz | rate-dependent? |
|---|---|---|---|---|---|
| 10% | 0.43% | 0.43% | 0.43% | 0.43% | no (x1.00) |
| 20% | 0.85% | 0.86% | 0.86% | 0.86% | no (x1.00) |
| 30% | 1.26% | 1.27% | 1.27% | 1.27% | no (x1.00) |
| 50% | **2.05%** | **2.05%** | **2.05%** | **2.05%** | no (x1.00) |
| 75% | 3.08% | 4.29% | 8.79% | 17.96% | **YES (x5.8)** |
| 100% | 7.26% | 14.85% | 30.21% | 53.76% | **YES (x7.4)** |

**Below ~50% of velocity limit the residual is physical** — identical across an 8x range of
physics rate, linear in speed. **Above it a rate-dependent artifact takes over.** The
earlier "time discretization, first order in dt" conclusion came from a sweep that rebuilt
the `World` per rate inside one process; re-run properly, the trend inverts.

# Added 2026-11-03 (later)

## The operating envelope — the most useful number produced so far

P1 said the residual floor is time discretization. Mapping it (`pal_fast_random.py`,
`out/residual_floor.json`) shows the floor is **not one number**: it has a sharp knee, and
the knee is wrist-specific.

Worst-joint RMS residual as a fraction of that joint's torque limit, 1000 Hz:

| speed | j1 | j2 | j3 | j4 | j5 | j6 | j7 | worst |
|---|---|---|---|---|---|---|---|---|
| 10% | 0.09 | 0.31 | 0.39 | 0.20 | 0.23 | 0.24 | 0.01 | **0.39%** |
| 25% | 0.24 | 0.78 | 0.96 | 0.53 | 0.54 | 0.60 | 0.03 | **0.96%** |
| 50% | 0.54 | 1.57 | 1.81 | 1.14 | 1.03 | 1.27 | 0.07 | **1.81%** |
| 75% | 1.43 | 3.15 | 3.45 | 2.20 | **8.80** | **7.79** | **8.01** | 8.80% |
| 100% | 3.87 | 5.79 | 6.59 | 4.45 | **27.79** | **25.31** | **20.54** | 27.79% |

Below 50% of velocity limit every joint stays under 1.81% and growth is smooth. Above it
joints 5/6/7 collapse while 1–4 barely move. Those three have 12 N·m limits rather than
87 **and** much lower inertia, so the same absolute error is a larger fraction and the
lighter links make a stiffer ODE. Both effects land on the same three joints.

Confirmed over 24 random poses, phases and speeds (correlation with speed 0.596):

```
frac 0.15: median 0.62%   frac 0.50: median 2.31%
frac 0.30: median 1.20%   frac 0.75: median 11.01%, max 28.68%
```

The scatter at 0.75 (2.47% to 28.68%) shows the floor is a function of **speed and pose**,
not speed alone.

### What it means for O4 and O5

A payload contributes about 4.63 N·m/kg at joint 4, so the floor converts directly into a
detection limit:

| speed | floor at j4 | smallest detectable payload | 3-sigma threshold |
|---|---|---|---|
| 10% | 0.174 N·m | 0.038 kg | 0.113 kg |
| 25% | 0.457 | 0.099 | 0.296 |
| 50% | 0.987 | 0.213 | 0.640 |
| 75% | 1.917 | 0.414 | 1.242 |
| 100% | 3.870 | 0.836 | **2.507** |

**At full speed most of the 0.1–3.0 kg ladder is invisible.**

**Decision needed, and it is close to forced: run O4 and O5 at or below 50% of velocity
limit.** Below the knee the floor is under 2% everywhere and the whole payload ladder
stays detectable.

### A conflict this creates between objectives

O6 benchmarks trajectory styles and wants fast motion. O2/O4/O5 need slow motion to stay
below the knee. **They cannot share a speed regime.** O6 should report its cycle-time,
peak-torque and energy figures at speed, while any residual-based claim stays below the
knee — and the results should say so rather than quietly mixing the two.

## P10 — the boot hang, revised

The earlier "1 in 5, then three in a row" figures were produced by a watchdog that could
not distinguish a slow boot from a hang. It counted lines in `docker logs` and gave up at
200 s; Isaac's extension loading can exceed that on a cold cache. One run reported
"attempt 1–4 hung" when attempt 4 had in fact succeeded and finished after the loop
abandoned it — and because the loop only killed the previous container at the *start* of
the next attempt, abandoned runs kept holding the GPU.

**Fixed.** `tools/run_sim.sh` waits for the script's own `[PAL]` marker rather than log
volume, allows 420 s, kills what it abandons, distinguishes a crash from a hang, and
reports the real boot time. First use: "script started after 28s", one attempt.

**The true hang rate is therefore unknown** — the earlier numbers measured the detector,
not the simulator. It will be re-measured as runs accumulate.

## Motion generation — decided on paper, not yet built

See `docs/MOTION_STACK.md`. Recommendation: Lula IK plus our own trajectory generators
(which O6 needs anyway, and which give analytic `q_cmd`/`qd_cmd`/`qdd_cmd`), with
`PickPlaceController` reserved for the O7 demo run.

The reason it is an attestation decision rather than a convenience one: the motion stack
determines which attack classes are detectable. A tampered **Cartesian goal** passes
through IK into a perfectly self-consistent joint trajectory and produces **no residual**;
only a spoofed **joint setpoint** is visible to the residual. If tampered goals are in
scope for O5, that needs a geometric or intent check, and it should be scoped now rather
than discovered when the detection table has a column of zeros.
