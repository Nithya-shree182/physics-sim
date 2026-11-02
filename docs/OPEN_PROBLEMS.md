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

### P2 — Joint friction never reaches the plant

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

### P6 — Hand mass discrepancy, ~0.8 N·m

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

## What we work on next

In order. The first two are prerequisites for trusting any later number.

**1. Fix P2 (friction not applying).** Smallest fix, largest consequence — it silently
invalidates every O5 result. Move the application after the final `world.reset()` and
verify by observed residual change, not by readback.

**2. Diagnose P1 (identity fails under fast motion).** Test the half-step hypothesis by
sampling state before and after the step and evaluating the model both ways. If that is not
it, check the Coriolis sign convention and whether armature is actually present in the
articulation. **Nothing downstream is trustworthy until this closes.**

**3. Settle P6 (hand mass).** Read `panda_hand`'s mass straight off the prim and compare
with the URDF. Cheap, and it decides whether the reference vector in the handoff is right.

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
