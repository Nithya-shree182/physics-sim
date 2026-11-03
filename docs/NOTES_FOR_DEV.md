# Notes for dev (simulator side), from the analysis side

Written for: dev (Person A). Round 1 written 2026-11-02 and updated 2026-11-03 (sections 1–8
below). Round 2 written 2026-11-03 after your `34fac9d` and `docs/REPLY_TO_ANALYSIS.md`.

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
