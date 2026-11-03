# Changes on the `nithya` branch

Newest first. Current status and next steps are in [context.md](context.md).

## 2026-11-03 (22:27 IST) — physics and proposal test suites, dev's `44f835b` reviewed

- New `tests/test_physics_model.py` (20 tests): mass matrix symmetric and positive definite,
  `Mdot - 2C` skew-symmetric, Coriolis quadratic in velocity, torque linear in acceleration,
  base rotation about the vertical changes nothing, gravity scaling, free-fall energy
  conservation, work-energy theorem, friction only removes energy, forward and inverse
  dynamics agree, Jacobian against finite-difference FK (linear and angular), a known push
  recovered through `J^T` and the wrench conversion (proposal 7.7, maths only), the payload
  torque at home (4.6306 N*m per kg on joint 4) from first principles, and the armature check
  below.
- New `tests/test_proposal_objectives.py` (15 tests), one per row of the proposal's
  evaluation table: O1 no dropped frames (real and synthetic), O2 within 5% per joint (the
  real 0 kg run is under 2%), O3 confidence bounds, gates and no alarm on gated rows, O4 known
  masses 0.5 to 3 kg and the real 1 kg run, O5 detection and false-positive rate with and
  without the gate (8 of 8 detected, 0 false alarms on 2 healthy runs, median delay 0.05 s;
  `-s` prints the table). Full suite: 65 passed.
- **Finding:** dev's logged mass matrix does not include armature. Its J7 diagonal matches
  ours built without armature (difference 0.0000) and differs by exactly 0.0061 with it. The
  residual is unaffected (the plant's dynamics do include it). Closes the lead in `setup.md`
  section 6; raised with dev in round 5.
- Dev's `44f835b` read: O5 injectors (`friction`, `stuck_joint`, `delay`) and a 14-run sweep
  at 20% speed with the fault at 3 s. The runs are in `out/o5/` on dev's machine, not in git.
  No sensor noise or backlash yet.
- Dev's docs copied for reference into `docs/dev_reference/` (unedited copies).
- `docs/NOTES_FOR_DEV.md`: round 5 added (tests listed, armature finding, prioritised
  requests, questions on ETA).
- Set up on this laptop (RTX 3050, CPU only): Miniconda, the `pal` env, real logs from `dev`,
  and the synthetic set and confidence sweep regenerated locally (all outside git).

## 2026-11-03 (night) — setup file, dashboard plan in the repo, encoding repair

- New `setup.md`: how to continue on another machine (no GPU needed; install, environment,
  identity, checks), which files are not in git and how to restore each (real logs come from
  `origin/dev` via a temporary worktree), where we stopped, what is stale in `context.md`,
  an unverified lead on the simulator's mass matrix (J7 differs from ours by exactly the
  armature, 0.0061), a start prompt for Claude Code, and next steps.
- New `docs/DASHBOARD_PLAN.md`: the revised dashboard design (canvas URLs, decisions, theme,
  colours, 50% trusted-speed band, build order). It had only existed in Claude's local memory.
- `context.md`: banner pointing to `setup.md` and `DASHBOARD_PLAN.md`; content unchanged.
- `changes.md`: the previous entry had been saved in a legacy Windows encoding, so its dashes
  and middle dots showed as garbled characters on GitHub. Re-encoded as UTF-8 (7 bytes in 3
  lines); text unchanged.
- Dev's `cffda31` reviewed only as far as its `NOTES_FOR_NITHYA.md` sections 1 to 3: the
  2.8% time-discretization claim is retracted (the rate sweep was invalid); below about 50% of
  the velocity limit the residual is physical and rate-independent; effort mode does not remove
  the above-knee artifact. Record at 50% or less in position mode.

## 2026-11-03 (evening) — first real Isaac Sim logs

**Dev's `dev` branch (`88adebc`) and `data/real/`**
- Two 3 s `slow_reference` logs at 1 kHz (0 kg and 1 kg), `position_stiff` drive, both pass
  `schema/validate.py` unchanged (3000 rows, 182 columns).
- Empty gripper: residual RMS 0.08–0.25 N·m on joints 1–4, 1–2% of limit on joints 5–7, so
  the identity closes on slow motion. 1 kg: joints 2 and 4 jump to 3.1 and 4.6 N·m (as
  predicted), wrench fz about -8.9 N.
- Payload: 1 kg reads 0.972 kg raw; the 0 kg run reads -0.028 kg, and using that as the tare
  gives 1.000 kg. The offset is a constant bias of the plant, so the tare must come from an
  empty-gripper run on the same trajectory.
- Detector: no false alarms on either run (no fault runs yet to re-tune on).
- Tests: `tests/test_real_data.py` (schema, identity, payload, tare); skipped if `data/real/`
  is absent. 35 pass.

## 2026-11-03 (later) — dev's reply reviewed, timing and schema aligned

**Dev's commit `34fac9d` and `docs/REPLY_TO_ANALYSIS.md`, reviewed**
- Every round-1 note answered: schema and validator adopted byte-identical and a writer built
  (`pal_telemetry.py`, `.npz` in the container, converted to Parquet on the host); the
  manifest it writes covers all 21 required fields; confidence score confirmed ours; payload
  placement agreed (4.6306 N·m/kg, point mass 0.1034 m along `panda_hand` z); `data/` added
  to `.gitignore`.
- Dev's conclusion that the fast-motion residual (~2.8% at 1 kHz) is time discretization is
  plausible, but its stiffness dependence also fits a position-drive reporting effect; asked
  for one effort-mode run to decide (round 2, R2.3).
- Two possible writer issues found by reading the code: the end-effector pose read from USD
  may be stale during physics (and its NaN fallback is invalid in the schema), and `t` lags
  the post-step row by one step (R2.2).
- No real log has arrived yet; asked for one outside git (R2.1).

**Code**
- `observers/residual.py`: new `backward` timing (qdd = (qd[k] - qd[k-1]) / dt) for dev's logs,
  chosen automatically from the manifest's `timing_convention`; an unknown convention warns
  and falls back to a central difference. Checked: on smooth motion the one-step pairing
  barely matters (both under 0.1% of a limit), consistent with dev's measurement.
- `schema/telemetry_v0.yaml` (still v0, additive): `mj_fresh` declared as an optional column;
  manifest optional keys declared (`timing_convention`, `timing`, `mj_every`,
  `payload_attachment`, `rows`).
- `schema/validate.py`: errors when the manifest states no timing convention, when
  `mj_every > 1` without `mj_fresh`, or when `rows` does not match the file.
- Tests: 30 pass (new: dev timing layout, unknown-timing fallback, three schema tests).

**Docs**
- `docs/NOTES_FOR_DEV.md`: round 2 added on top (status table and items R2.1–R2.5).
- `context.md`: conventions (row timing, payload placement, log delivery), open items, next steps.

## 2026-11-03 — observers ready to wire to real logs

**Dev's commit `23822b8`, reviewed**
- The shipped `franka.usd` was dynamically a stick figure (every COM at the link origin,
  inertia ~1e-6 kg·m², masses off by up to 4.5×). Dev now authors mass, COM and inertia from
  `panda_arm_hand.urdf`, and the simulator's home-pose `g(q)` matches our model to a few
  hundredths of a N·m. Our model needed no change. Dev's URDFs are identical to ours.
- Findings sent back (`docs/NOTES_FOR_DEV.md` section 2): the small remaining gap is the arm
  sagging in position mode (every loaded joint moves in the predicted direction), so compare
  at the logged pose; and dev authors only the inertia diagonal, which we measured at
  ≤ 0.05 N·m (0.11% of a limit), so it is not the cause of dev's P1.
- Dev committed two minutes after our notes were pushed, so the schema, payload placement,
  confidence ownership, row timing and `.gitignore` requests are still open.

**New: `observers/`**
- `residual.py`: `load_run` / `run_from_frame`, `compute_residual` with
  `r = tau_measured - tau_model`, qdd by the manifest's timing, 20 Hz zero-phase filter
  (`DEFAULT_CUTOFF_HZ`), external wrench `W = -(J^T)^+ r` at `panda_hand`, model or logged
  Jacobian, and a guard refusing `tau_applied` on position-mode logs. CLI prints per-joint
  RMS and the wrench.
- `confidence.py`: rotational Jacobian rows scaled by 0.472 m; `c`, normalised `c_norm`,
  vertical score `c_vertical` (dev's F6 idea), noise gains, physical gates (≤ 10 N per N·m),
  workspace sweep. A percentile-based gate was tried first and rejected because it let the
  elbow-singular pose through.
- `payload.py`: mass from the residual force, corrected for the payload's acceleration, held
  while gated, running mean with an uncertainty band, optional tare.
- `detector.py`: threshold baseline on two channels (residual, and tracking error
  `q_cmd - q`), gated by pose, with evaluation against the run labels.

**Model**
- `model/pinocchio_model.py`: new `point_acceleration` (world acceleration of a point fixed to
  the hand), used by the payload estimator.

**Tests**
- New `tests/` (pytest, 25 tests) and `pytest.ini`. Two bugs found by them and fixed: the
  logged-Jacobian option read nonexistent columns `J_0..J_41` instead of `J_<row>_<col>`; the
  payload estimate started 3.5% low because the zero-phase filter distorts the first
  samples and the running mean was seeded with them. Fix: the residual drops the first and
  last 3 filter time constants, and the running mean starts as a plain average.

**Results on the synthetic set**
- Residual: exactly 0 on a clean plant; a 1 kg payload reads fz = -9.810 N.
- Confidence: home 0.96, elbow-singular 0.15 (gated); raw home `c` 0.1242 vs dev's 0.1245.
- Payload: exact without mismatch; with the ±3% mass mismatch, a tared 1 kg hold reads
  1.0012 kg, a 1 kg fast trajectory 0.965 kg (motion-dependent mismatch a static tare cannot
  remove).
- Detector: friction (stiction / bearing wear) on joint 3 detected in 0.64 s, stuck joint 4 in
  0.025 s, both on the right joint; a 50 ms command delay detected in 0.054 s on the tracking
  channel only (the residual cannot see it, dev's A2); no false alarms on 6 fault-free runs.
  Thresholds were set on this same data and must be re-tuned on real logs.

**Docs**
- `docs/NOTES_FOR_DEV.md` rewritten with an at-a-glance status table.
- `context.md` updated: conventions, how to run, key numbers, open items, next steps.

## 2026-11-02 — friction, synthetic logs, review of dev's update

**Model**
- `model/pinocchio_model.py`: added joint friction (Gaz et al. 2019 Table I sigmoid, same
  values as dev's `friction_sigmoid()`), scalable per joint via `friction_scale`; switch off
  with `load_panda(friction=False)`. `tau_model` now includes friction by default
  (`include_friction=False` gives rigid-body only). New `forward_dynamics` (aba with friction)
  and `ee_pose` (position and unit quaternion w, x, y, z). Docstring: residual sign is
  `tau_measured - tau_model`.
- `model/gravity_check.py`: two new self-tests, friction zero at rest, and forward dynamics
  exactly inverts `tau_model` (3e-14 N·m). 10/10 pass.

**Synthetic data**
- New `model/synth_log.py`: Pinocchio plant (masses ±3%, friction ×1.15, sensor noise,
  optional payload and faults: friction, stuck joint with exact constrained dynamics, command
  delay) under a libfranka-gain joint impedance controller with twin feedforward, 1 kHz,
  torque mode. Writes schema-valid Parquet + JSON manifest into `data/synthetic/` (not
  committed). `--batch` writes 9 standard runs.
- Fixed during testing: the first stuck-joint version froze the joint after each step, which
  let the saturated joint torque leak into the other joints (joint 6 reached 4.40 rad, past
  its 3.75 rad limit). Replaced with constrained dynamics; all joints now stay in limits.

**Schema and docs**
- `schema/telemetry_v0.yaml`: physics rate 1000 Hz (dev changed it); Jacobian row 7
  (`panda_hand`) per dev's measurement, never the last row; residual sign
  `tau_measured - tau_model`.
- `WORK_SPLIT.md` 3.1: residual is `tau_measured - tau_model`, following the proposal PDF
  (this reverses the earlier `tau_model - tau_measured` entry).
- `docs/SOURCES.md`: added the verified Gaz Table III values and the `pdftotext -layout`
  row-misalignment pitfall.
- New `docs/NOTES_FOR_DEV.md`: findings and requests for dev, including a retraction of our
  earlier claim that dev's Gaz friction arrays were shifted by one joint (they are correct).
- New `context.md` and this `changes.md`.

**Findings from dev's commit `c307c21`**
- At home, the simulator's gravity vector has exact zeros on joints 3 and 5 that no URDF
  reproduces: the simulated asset's centres of mass differ from the URDF's. Logged as the main
  open issue, with a request to dev.
- Dev planned to write the schema and the confidence sweep; both overlap with our work and
  are raised in `docs/NOTES_FOR_DEV.md`.

## 2026-11-02 — commit `7cfba9e`: independent model, gravity check, schema

- `model/pinocchio_model.py`: 7-DOF Panda from `panda_arm_hand.urdf`, fingers locked open
  (0.93 kg gripper kept as robot mass), armature 0.0061 set on `model.armature`.
- `model/gravity_check.py`: self-tests (moving mass equals URDF sum, torque limits, armature in
  `M` and `rnea`, `g(q)` equals dU/dq, `M` symmetric and positive definite) and the `--expect`
  comparison against simulator `g(q)`.
- `model/poses.py`: named poses and joint limits shared with dev's `pal_config.py`.
- `schema/telemetry_v0.yaml` and `schema/validate.py`: per-step telemetry frame, run manifest,
  and a validator; tested on a good and a deliberately broken log.
- `environment.yml`: conda env `pal` (pinocchio from conda-forge).
- `panda_arm_hand.urdf`, `panda.urdf` added to the repo.

## Earlier commits

- `e2c48f5` — `docs/SOURCES.md` and the reference PDFs (Gaz supplementary material, Franka
  Research 3 datasheet).
- `ec4df3a` — `WORK_SPLIT.md`, the two-person plan.
