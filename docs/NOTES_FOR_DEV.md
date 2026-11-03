# Notes for dev (simulator side), from the analysis side

Written for: dev (Person A). Newest round on top. Index:

| Round | Written | After |
|---|---|---|
| 3 | 2026-11-03 16:56 IST | first real logs, `88adebc` |
| 2 | 2026-11-03 (earlier) | `34fac9d` and `docs/REPLY_TO_ANALYSIS.md` |
| 1 | 2026-11-02, updated 2026-11-03 | `23822b8` |

# Round 3 — complete data request (2026-11-03 16:56 IST)

Thank you for `slow_reference_0kg` and `slow_reference_1p0kg`. Both pass `schema/validate.py`
unchanged, and the first analysis is done (section A). This round is one consolidated request:
**if you can produce everything in section C in one session and send it together, we can run
every remaining test in one pass.** Priorities: **P1** blocks the main results, **P2** is
needed for the report, **P3** is nice to have.

## A. What the first two logs showed

| Check | Result |
|---|---|
| Schema | both VALID (3000 rows, 182 columns) |
| `ee_pos_*` varies, `t` aligned | yes (x moves 0.29–0.33 m; `t` starts at 0.001): R2.2 closed |
| Residual, 0 kg | RMS 0.08–0.25 N·m; under 0.26% of limit on joints 1–4, **1.4–1.8% on joints 5–7** |
| Residual, 1 kg | joints 2 and 4 jump to 3.1 and 4.6 N·m; wrench fz about -8.9 N |
| Payload, 1 kg | raw 0.972 kg; the 0 kg run reads -0.028 kg; using it as tare gives 1.000 kg |
| Detector | no false alarms on either run; no fault runs to tune on |

## B. Decisions the data will make (why each item is requested)

1. Is the fast-motion floor a property of the arm or of the position drive? (C1)
2. Does the payload estimate stay right while the arm accelerates and across poses? (C2, C3)
3. What detection delay and false-alarm rate do the detector thresholds give on real
   faults? They were tuned on synthetic data. (C4, C5)
4. Is the 1–2% residual on joints 5–7 a model gap or a unit/limit effect? (D1)

## C. Runs to send

Format for every run: Parquet + JSON manifest per `schema/telemetry_v0.yaml`, 1000 Hz,
`position_stiff` unless stated, **sent outside git** into `data/real/`. Name files
`<traj>_<payload>kg_<rate>hz[_<tag>]`. Keep seed, drive mode and `git_commit` in the manifest.

| ID | Pri | Run | Duration | Why |
|---|---|---|---|---|
| C1a | P1 | Fast trajectory at 100% of joint speed limit, **effort mode** (feedforward `M qdd + C qd + g` + light PD), 0 kg | 3–5 s | decides B1 |
| C1b | P1 | Same trajectory and seed, `position_stiff`, 0 kg | 3–5 s | direct comparison |
| C1c | P1 | Same as C1a and C1b at 50% speed | 3–5 s each | locates the knee |
| C2a | P1 | `excitation` (multi-sine), 0 kg | 6–10 s | payload under acceleration, tare |
| C2b | P1 | `excitation`, 1 kg, same seed as C2a | 6–10 s | payload under acceleration |
| C3a | P2 | Payload ladder on `slow_reference`: 0.5, 2, 3 kg (0 and 1 kg already sent) | 3 s each | linearity |
| C3b | P2 | `hold` at three poses: home, one far from home, one near-singular (elbow almost straight), each at 0 kg and 1 kg | 2 s each | confidence gating, tare per pose |
| C4 | P1 | Fault runs from half way, 0 kg, `slow_reference`: friction ×3 on joint 3, friction ×3 on joint 6, `stuck_joint` on joint 4, `delay` 50 ms | 6–8 s each | detector re-tuning |
| C4b | P2 | Same faults with 1 kg payload; and `backlash` if you can inject it | 6–8 s each | payload + fault confusion |
| C5 | P1 | One **fault-free** long run, 0 kg | 60 s or more | calibration window and noise floor |
| C6 | P2 | Payload picked up mid-run (0 → 1 kg at about half way) | 6–8 s | payload change detection |
| C7 | P3 | An impact or collision, if you can inject one | 5 s | spike detection |
| C8 | P3 | `altered_waypoint` and `spoofed_setpoint` attacks (schema `attack_type`) | 6–8 s each | proposal O6 |
| C9 | P2 | Seeds 1 and 2 of C4 and C5, if cheap | as above | spread of the results |

Labelling (all in the existing schema, no change needed): set the per-row columns
`fault_type` (`none`, `friction`, `backlash`, `stuck_joint`, `delay`), `fault_joint`
(0–6 or -1), `fault_param`, `fault_active`, `attack_type`, `attack_active`. `fault_active`
must flip at the true onset step, since detection delay is measured from it. For payload
runs put the mass in the manifest `payload_kg` (already done).

**Minimum set if time is short:** C1a, C1b, C2a, C2b, the four C4 runs, and C5.

## D. Questions and clarifications

1. **Joints 5–7 residual on the empty arm (1.4–1.8% of limit)** vs under 0.26% on joints
   1–4. Is there a known reason (wrist inertia authored as diagonal only, finger coupling,
   small limits)? Please send your own per-joint absolute RMS for the same run.
2. **Which rate, speed and stiffness gave the 27.98%** in the `timing_convention` text? It
   matches no row of the sweep table in `RESIDUAL_FLOOR.md` (30.21% at 1000 Hz, 100%).
3. **`torque_convention` is "unverified".** Does `tau_measured` include friction? Please
   fill this in the manifest once known.
4. In effort mode, which signal is logged as `tau_measured`: the applied command or the
   joint effort read back from the simulator? We need the read-back, with `tau_applied`
   kept separate, so the residual cannot become circular.

## E. Housekeeping from earlier rounds, still open

- Copy `schema/` from `nithya` (R2.5): `mj_fresh` optional, manifest optional keys declared.
- Confirm `ee_pos_*` stays correct when the arm moves fast (checked on slow motion only).
- Keep the empty-arm URDF as the only source for the twin; the model must not know the
  payload.

## F. What we do when it arrives

Validate each file; gravity check at a rest segment; residual on C1 and C2 and a decision
on `DEFAULT_CUTOFF_HZ`; payload on C2, C3, C6 with the tare procedure; detector re-tuned on
C4/C5 with detection delay and false-alarm rate reported next to the floor from C1; then the
classifier and dashboard on the real labelled runs. Results go back to you as Round 4.

---

# Round 2 — after `34fac9d` (2026-11-03)

Thank you for the reply; every round-1 item is now answered. Status:

| # | Round-1 item | Status |
|---|---|---|
| 1 | Gaz friction values | Closed |
| 2 | Arm vs URDF inertials | Closed (authored). 2a/2b follow-ups below are optional |
| 3 | Schema + writer | Writer done on your side; **end-to-end check waiting on a real log (R2.1)** |
| 4 | Residual sign | Closed |
| 5 | Confidence score ownership | Closed: analysis side |
| 6 | Payload placement | Closed: 4.6306 N·m/kg, point mass 0.1034 m along `panda_hand` z on both sides |
| 7 | Analysis pipeline | Ready (section 7), and now reads your timing convention (R2.4) |
| 8 | `data/` ignored | Closed |

New items:

**R2.1 Please send one captured run (Parquet + JSON) outside git.** `data/` is ignored on
both branches now, so a log cannot come through the repo. A short one is enough (e.g. 2–4 s
of `slow_reference`, and the same with a 1 kg payload). It closes your own next step 1 and
lets the whole analysis chain run on real data for the first time.

**R2.2 Two possible writer issues, from reading `pal_telemetry.py` (not from a run):**
- `_ee_pose()` reads the hand pose from USD with `ComputeLocalToWorldTransform(0)`. While
  physics drives the articulation through the tensor API, the USD transform may not be
  updated every step, so `ee_pos` could stay frozen at its first value. And if the read
  throws, it writes NaN, which `validate.py` rejects (non-nullable). Please check that
  `ee_pos_*` changes during a moving run; if not, read the body pose from the articulation
  instead.
- `pal_capture.py` passes `t = k * DT` before `world.step()`, but the row holds post-step
  values, so `t` lags the data by one step. `t = (k + 1) * DT` matches your timing note.

**R2.3 Before treating the ~2.8% fast-motion floor as fundamental, one cheap test.** Your
evidence (first order in `dt`, unaffected by solver iterations, **rising with drive
stiffness**) fits discretization, but it also fits a narrower cause: in position mode the
reported `tau_measured` of a stiff implicit drive may not be exactly the torque that moved
the arm during the step. Your F8 (a 0.14 rad tracking error giving ~10 N·m against a
400 N·m/rad drive) points the same way. The deciding run: the same 100%-of-limit trajectory
in **effort mode**, with torques from your own `M qdd + C qd + g` as feedforward plus a light
PD. If the floor collapses, it comes from the position drive, and O2 calibration and the O5
datasets should be recorded in effort mode; if it stays, your conclusion stands. Also, the
timing test's 27.98% does not match any rate in your sweep (2.78% at 1000 Hz): which rate
and stiffness was it run at?

**R2.4 The analysis side now reads your `timing_convention`.** Rows assembled after the step
pair `tau[k]` with `(qd[k] - qd[k-1]) / dt` (backward difference); synthetic logs keep the
forward one. We also confirmed your finding: on smooth motion a one-step pairing error is as
small as the right pairing (both under 0.1% of a limit), so it matters only where torque
jumps within a step (contact, fault onset).

**R2.5 Small additive schema change, please copy `schema/` from `nithya`.** Still v0, nothing
existing breaks:
- `mj_fresh` is now a declared optional column, so it no longer triggers a warning.
- Manifest optional keys declared: `timing_convention`, `timing`, `mj_every`,
  `payload_attachment`, `rows`.
- `validate.py` now errors if the manifest states no timing convention, if `mj_every > 1`
  without an `mj_fresh` column, or if `rows` does not match the file. Your writer already
  satisfies all three.

---

# Round 1 (2026-11-02, updated 2026-11-03)

## At a glance (as of round 1)

| # | Item | Status |
|---|---|---|
| 1 | Gaz friction values | **Closed.** Yours are right; our earlier "shifted" claim was wrong |
| 2 | Simulated arm vs URDF inertials | **Resolved** by your inertial authoring. Two small follow-ups in section 2 |
| 3 | Telemetry schema exists on `nithya`; please write the Parquet writer against it | Answered in round 2 |
| 4 | Residual sign `tau_measured - tau_model` | **Agreed** |
| 5 | Confidence score: analysis side owns it | Answered in round 2 |
| 6 | Where is the payload attached? Your two docs disagree | Answered in round 2 |
| 7 | What the analysis side has ready to wire to real logs | For your information (section 7) |
| 8 | Add `data/` to `.gitignore` | Answered in round 2 |

---

## 1. Closed: your Gaz friction values are right

An earlier review from our side said the Gaz arrays in `pal_config.py` were shifted by one
joint. That was wrong. It came from `pdftotext -layout`, which misaligns Table III's row
labels by one row; `-raw` and `-table` give exactly your values. `docs/SOURCES.md` records
the verified table and the pitfall.

## 2. Resolved: the plant is now authored from the URDF

Thank you. With `AUTHOR_INERTIAL_FROM_URDF` the two engines agree at home:

```
Pinocchio (URDF)     : [0, -4.584, -0.636, 22.881, 0.623, 2.497, 0]
Isaac Sim (authored) : [0, -4.663, -0.636, 22.954, 0.623, 2.441, 0.001]
```

Two small follow-ups, neither blocking:

**2a. The remaining gap is the arm sagging, not a model difference.** In position mode the
drive holds gravity with finite stiffness, so the arm settles slightly below home. Every
loaded joint moves the way sag predicts:

| at home | j2 | j4 | j6 | j7 |
|---|---|---|---|---|
| Pinocchio, exact home | -4.584 | 22.881 | 2.497 | 0 |
| Isaac Sim (yours) | -4.663 | 22.954 | 2.441 | 0.001 |
| Pinocchio, fully sagged at 400 N·m/rad | -5.143 | 23.069 | 2.334 | 0.004 |

So compare `g(q)` at the **logged** joint angles, not at the commanded home pose. The schema
already carries `q`; the analysis side always evaluates the model at the logged `q`.

**2b. Only the inertia diagonal is authored.** `pal_scene.py` 11b passes `ixx, iyy, izz` and
drops `ixy, ixz, iyz` (e.g. `ixz = 0.0103` on link2). We measured the effect: at most
0.05 N·m (0.11% of a torque limit) on a fast multi-sine trajectory. So it is **not** the
cause of P1, but it is a free accuracy gain: diagonalise each URDF tensor (eigenvectors give
the principal-axes quaternion for `set_body_coms`, eigenvalues go to `set_body_inertias`), or
use the 9-element form if this build accepts it.

## 3. Waiting: the schema exists; please write the writer against it

Your next-steps list (CONTEXT.md item 2, OPEN_PROBLEMS item 4) says to write
`schema/telemetry_v0.yaml` and that the analysis side is blocked on it. It already exists on
`nithya`, with `schema/validate.py`, and the whole analysis pipeline reads it. Please
implement the Parquet writer against it rather than drafting a second one.

It already encodes your findings: 1000 Hz; arm columns are `[:7]` of the 9-DOF arrays;
Jacobian `get_jacobians()[0, 7, :, :7]` (`panda_hand`); `drive_mode` per row and in the
manifest; residual sign `tau_measured - tau_model`.

**Please confirm or change:**
- the label columns and their allowed values (`traj_type`, `drive_mode`, `fault_type`,
  `attack_type`)
- our extra columns `qdd_cmd_*` (commanded acceleration) and `push_force_*` (known force for
  the validation push)
- how decimated `M`/`J` should be encoded if you decimate them for P4 (suggestion for v1:
  hold the last value and add an `mj_fresh` boolean)
- the **row timing**: which step's torque goes with which state. Synthetic logs declare
  "tau at row k moves qd[k] to qd[k+1]" in the manifest; the residual code reads it. Stating
  the real convention would also settle your P1 half-step hypothesis.

Test any writer with `python schema/validate.py <file>.parquet`.

## 4. Agreed: residual sign

`r = tau_measured - tau_model`, as in the proposal (section 6) and your code.

## 5. Waiting: please confirm the analysis side owns the confidence score

It is built (`observers/confidence.py`), using your 0.472 m lever arm and your F6 idea:

- **Scaled score** `c(q)` on `J_s = diag(1,1,1,L,L,L) J`. Raw home `c` = 0.1242 against your
  0.1245, so the Jacobians agree. Even scaled, `c` tops out near 0.22 (99th percentile of
  5000 random poses), so "near 1 means healthy" never happens on a 7-DOF arm; a normalised
  display score divides by that reference.
- **Vertical score** `c_vertical` for the payload estimate, as you proposed.
- **Gates are physical, not percentiles:** at most 10 N of force error per N·m of torque error
  (detector, `1/sigma_min`) and 10 N of vertical force error per N·m (payload). A workspace
  percentile was tried first and rejected: random poses are mostly awkward, so its 5th
  percentile let the elbow-singular probe pose through.

| pose | c_norm | force gain | detector | payload |
|---|---|---|---|---|
| home | 0.96 | 4.8 | ok | ok |
| near_singular_wrist | 0.88 | 6.0 | ok | ok |
| horizontal_reach | 0.58 | 7.2 | ok | ok |
| near_singular_extended | 0.15 | 26.7 | gated | gated |

So you can drop "rescale the Jacobian, sweep the workspace" from your list.

## 6. Waiting: where is the payload attached?

- `PINOCCHIO_HANDOFF.md` section 7: **4.6306 N·m per kg** on joint 4.
- `INTEGRATION.md` section 1, change for +3 kg: `[0, -9.573, 0, 14.688, 0, 2.730, 0]`, that
  is **4.896 N·m per kg** on joint 4.
- Synthetic plant, payload sphere 0.1034 m along `panda_hand` z (the Franka TCP):
  `[0, -3.00, 0, 4.60, 0, 0.83, 0]` N·m per kg.

Which body is it attached to, at what offset, and which of your two numbers is current? The
mass estimate at rest does not depend on it, but the estimate during motion does
(`PAYLOAD_OFFSET_M` in `observers/payload.py`).

## 7. For your information: ready to wire to real logs

All of this runs today on synthetic logs and needs only a schema-valid log from you:

| file | does | synthetic result |
|---|---|---|
| `observers/residual.py` | `r = tau_measured - tau_model`, qdd from the manifest's timing, 20 Hz zero-phase filter, external wrench `W = -(J^T)^+ r` at `panda_hand`; refuses `tau_applied` on a position-mode log | clean plant: r = 0; 1 kg held: fz = -9.810 N |
| `observers/confidence.py` | section 5 | |
| `observers/payload.py` | mass from the vertical force, corrected for acceleration, held while gated, optional tare | 1 kg held, tared: 1.0012 kg |
| `observers/detector.py` | threshold baseline on two channels: residual, and tracking error `q_cmd - q` | friction (stiction / bearing wear) on j3, stuck j4, 50 ms command delay: all detected, right joint, no false alarms on 6 fault-free runs |

The command-delay case shows your A2 in practice: the residual stays quiet because the
physics is consistent; only the tracking channel catches it.

**What the analysis side needs in a real log to use all of it:** `tau_measured`, `q`, `qd`,
`q_cmd` (the scheduled command, for the tracking channel), `drive_mode`, the row timing in the
manifest, and, for the validation push, effort mode plus `push_force_*`.

## 8. Waiting: ignore `data/`

Logs go in `data/` (the synthetic set alone is about 250 MB). Your `.gitignore` covers
`out/`, `logs/` and `__pycache__/` but not `data/`. Please add `data/`; we have not added a
`.gitignore` on `nithya` to avoid a merge conflict with yours.
