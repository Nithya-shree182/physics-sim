# Notes for dev (simulator side), from the analysis side

Written for: dev (Person A), after reading `docs/PINOCCHIO_HANDOFF.md`, `docs/OPEN_PROBLEMS.md`
and `INTEGRATION.md` on the `dev` branch (commit `c307c21`). Each item says what we found, and
what we need from you. Status as of 2026-11-02.

---

## 1. Correction: your Gaz friction values are right

An earlier review from our side said the Gaz arrays in `pal_config.py` were shifted by one
joint. **That was wrong. Ignore it.** It came from `pdftotext -layout`, which misaligns the
row labels of Table III by one row. Re-read with `-raw` and `-table`, which keep each label
with its own numbers, the values match yours exactly (`fv`, `fc`, `fo`), and all seven Coulomb
terms come out positive. `docs/SOURCES.md` now records the verified table and the pitfall.

**Nothing to do.**

## 2. Blocking: the simulated arm is not the URDF's arm

Your handoff (section 7) says that if the Pinocchio model reproduces your home-pose gravity
vector, the URDF parse is correct. It does not reproduce it, and the parse is not the cause.

Home pose `q = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]`, at rest, no payload, N·m:

| | j1 | j2 | j3 | j4 | j5 | j6 | j7 |
|---|---|---|---|---|---|---|---|
| Isaac Sim, unmodified `franka.usd` (yours) | 0 | -1.655 | **0** | 20.077 | **0** | 0.856 | 0 |
| Isaac Sim, hand mass re-authored to 0.73 (yours) | 0 | -2.176 | **0** | 20.874 | **0** | 1.004 | 0 |
| Pinocchio, `panda_arm_hand.urdf` | 0 | -4.584 | -0.636 | 22.881 | 0.623 | 2.497 | 0 |
| Pinocchio, `panda.urdf` (no hand) | 0 | -1.783 | -0.644 | 18.575 | 0.634 | 1.694 | 0 |

- The simulator gives **exactly zero on joints 3 and 5**. Both URDFs give about ±0.63 N·m
  there, because they place several link centres of mass slightly off the arm's plane.
- **No change of mass can produce an exact zero there**, so the asset's centres of mass differ
  from the URDF, not only the hand mass. Your P6 (hand mass, ~0.8 N·m) is part of a larger
  difference: up to 2.9 N·m on joint 2 and 13.7% of joint 6's 12 N·m limit.
- The Pinocchio side is verified independently of the simulator
  (`python model/gravity_check.py`, 10/10 self-tests): mass equals the URDF sum (16.992 kg
  moving), `g(q)` equals the numerical gradient of potential energy to 1e-8 N·m, armature
  reaches `M` and `rnea`, forward and inverse dynamics invert to 3e-14.

**Why it matters:** until this is resolved, the twin and the plant differ by an amount nobody
chose, and every residual carries it.

**What we need:**
1. Per-body **mass, centre of mass (in the body frame) and inertia tensor** as the
   articulation actually simulates them, for all 11 bodies (`panda_link0..7`, `panda_hand`,
   both fingers). The Isaac Sim articulation API exposes body masses, COMs and inertias; please
   check the exact method names on 6.0.1.
2. A decision, together: either **author the URDF's inertials into the plant** (our
   recommendation: the URDF values have a known source, and the deliberate mismatch is then
   added on purpose), or build the Pinocchio model from the asset's values.

## 3. The telemetry schema exists; please write the Parquet writer against it

Your `OPEN_PROBLEMS.md` "next" item 4 is to write `schema/telemetry_v0.yaml`. It is already
on `nithya`: `schema/telemetry_v0.yaml` (per-step columns + run manifest) and
`schema/validate.py` (checks a Parquet log and its manifest). Please implement the writer
against it rather than drafting a second schema. It already uses your findings:

- 1000 Hz, one row per physics step
- arm columns are `[:7]` of the 9-DOF arrays; fingers in separate gripper columns
- Jacobian from `get_jacobians()[0, 7, :, :7]` (row 7 = `panda_hand`, per your handoff)
- `drive_mode` per row and in the manifest, so the residual code can refuse a position-mode
  log when `tau_applied` would be used (your A3)
- residual sign `tau_measured - tau_model` noted in the conventions

**Please confirm or change:**
- the label columns and their allowed values (`traj_type`, `drive_mode`, `fault_type`,
  `attack_type`)
- the extra columns we added: `qdd_cmd_*` (commanded acceleration) and `push_force_*` (known
  force for the validation push)
- how decimated `M`/`J` should be encoded if you decimate them for P4. A suggestion for v1:
  hold the last value and add an `mj_fresh` boolean column.
- the **timing convention** of a row. Synthetic logs use "tau at row k moves qd[k] to
  qd[k+1]". Stating the real one in the manifest would also settle your P1 half-step
  question for both sides.

You can test a writer against the validator with any synthetic log from
`model/synth_log.py` (see section 6).

## 4. Residual sign: agreed

`r = tau_measured - tau_model`, as in the proposal (section 6) and your code. `WORK_SPLIT.md`
section 3.1 now says this. `INTEGRATION.md` section 1 argues against an older `WORK_SPLIT.md`
draft that used `tau_applied`; that draft is gone, but your position-mode guard stays useful.

## 5. Ownership: the confidence score

Your next-step 5 (rescale the Jacobian rows, sweep the workspace, set the `c(q)` threshold)
is on the analysis side's list, and it needs no simulator. **Proposal: the analysis side takes
it**, using your 0.472 m lever arm for the rotational rows and your idea of a
direction-specific (vertical) confidence for O4. Please confirm so it isn't done twice.

## 6. Payload placement: your two documents disagree

- `PINOCCHIO_HANDOFF.md` section 7: **4.6306 N·m per kg** on joint 4.
- `INTEGRATION.md` section 1, change for +3 kg: `[0, -9.573, 0, 14.688, 0, 2.730, 0]`, which
  is **4.896 N·m per kg** on joint 4.
- Synthetic plant, payload as a sphere 0.1034 m along `panda_hand` z (the Franka TCP):
  `[0, -3.00, 0, 4.60, 0, 0.83, 0]` N·m per kg.

**Please say where the payload is attached** (which body, what offset), and which of your two
numbers is current. The payload estimator's lever arm depends on it.

## 7. Done on the analysis side, for your reference

- **Armature** `model.armature = 0.0061` on all 7 joints, as your handoff section 3.1
  specifies. Verified that Pinocchio 4.1.0 applies it in both `crba` and `rnea`.
- **Friction** in the model: the Gaz Table I sigmoid, same values as your
  `friction_sigmoid()`, scalable per joint.
- **Synthetic logs** (`model/synth_log.py`): a Pinocchio plant (masses ±3%, friction ×1.15,
  sensor noise, optional payload and faults: friction, stuck joint, command delay) under a
  libfranka-gain joint impedance controller at 1 kHz, written in the schema and validated.
  With no mismatch and no noise the residual is 1e-13 N·m.
- The URDFs are now in the repo root (`panda_arm_hand.urdf`, `panda.urdf`), so nothing
  depends on the sibling `Physyk_AI-master` folder.

## 8. Small request: ignore `data/`

Logs go in `data/` (synthetic logs alone are ~250 MB). Your `.gitignore` covers `out/`,
`logs/` and `__pycache__/` but not `data/`. Please add `data/` when you next touch it; we have
not added a `.gitignore` on `nithya` to avoid a merge conflict with yours.
