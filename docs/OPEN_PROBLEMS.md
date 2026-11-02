# Open problems and what we work on next

Status as of 2026-11-02. Everything here was measured, not assumed; each item says what was
observed and what it blocks.

---

## Blocking problems

### P1 — The three-term identity fails under fast motion

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

### P2 — ~~Joint friction never reaches the plant~~ **FIXED 2026-11-02**

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

### P4 — Telemetry cannot run at 1 kHz

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

**2. Re-run the excitation test, then diagnose P1.** The 5.98% failure was measured on the
*unauthored* plant — point masses, no rotational inertia, wrong COMs. The plant has since
changed substantially, so the number may have moved on its own. Re-measure before
investigating.

If it persists: diagnose P1 (identity fails under fast motion). Test the half-step hypothesis by
sampling state before and after the step and evaluating the model both ways. If that is not
it, check the Coriolis sign convention and whether armature is actually present in the
articulation. **Nothing downstream is trustworthy until this closes.**

**3. ~~Settle P6~~ — done.** Resolved into the inertial authoring above.

**4. Write `schema/telemetry_v0.yaml` and the Parquet writer.** This is the week-1 handoff
artifact and the analysis side is blocked on it. Should carry `tau_measured`, `tau_applied`,
`q_cmd`, the control mode, and a per-run manifest.

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
