# Changes

Newest first. What is still to do is in [docs/TODO.md](docs/TODO.md); read that first.

## 2026-11-08 (evening) — the Panda removed, telemetry v1, Pinocchio energy and payload work

Written for: whoever picks this up next. The one-page version is [docs/TODO.md](docs/TODO.md); the
telemetry contract is [docs/TELEMETRY.md](docs/TELEMETRY.md).

**The Franka Panda is gone from the runtime, not just unselected.**

- 24 Panda-era modules deleted from `workspace/` (the one-off probes, the finger workcell
  `pal_scene.py` / `pal_objects.py`, the rate and inertia audits). `panda.urdf`,
  `panda_arm_hand.urdf` and `assets/franka_panda/` are gone; the launchers no longer mount them.
- `workspace/pal_config.py` rewritten: **706 lines to 272**, and nothing arm-specific left in it.
  Everything about the arm is now a view onto the one `RobotSpec` in `pal_robot.py`; what stays is
  the simulation's own settings — rates, solver, contact, drive gains, camera, thresholds. The
  Panda's sourcing notes went with it; they are in git history at `b696c1c` if the provenance of a
  Franka number is ever wanted again.
- Drive gains are now a multiple of each joint's effort limit (`KP_PER_TAU = 4.6`, from the Panda's
  400 N*m/rad against its 87 N*m joint) instead of a fixed 400, which was a stiff drive on an 87
  N*m joint and a soft one on a 330 N*m one.
- `load_panda` / `PandaModel` renamed to `load_robot` / `RobotModel` everywhere, the aliases
  dropped, and `pal.sh` no longer pins `PAL_ROBOT=panda` — a spec that no longer exists.

**Sensor noise is off by default.** `pal_sensors.py` rewritten: ideal sensing unless
`PAL_SENSOR_NOISE` asks for it, with the reference sigmas kept for sweeping. Two consequences,
both stated in the file rather than discovered later: every channel is now readable without an
injected error in it, and a false-alarm *rate* needs the noise switched back on. Every run manifest
carries a `sensor_noise` block, so "no noise" and "nobody recorded whether there was noise" can no
longer look the same.

**Telemetry v1** (`schema/telemetry_v1.yaml`, `workspace/pal_telemetry.py` rewritten). Six joints,
no finger columns, and 205 columns in place of the old ~170. What is new is the physics:

- `qdd` is logged, differenced in-sim at the physics rate. The analysis side used to difference a
  velocity that had already been low-pass filtered. Measured: on a noisy synthetic run the residual
  floor is **less than half** what differencing gives (`tests/test_residual.py`), and no row has to
  be dropped to a finite difference any more.
- The torque is decomposed per joint: `g`, `coriolis`, `tau_inertial`, `tau_friction`,
  `tau_unexplained`. They sum to `tau_applied` exactly, and the validator checks it — a
  disagreement means two columns in one row came from different state, which is the failure mode a
  per-step writer has and is invisible in a plot.
- Energy and power: `E_kin` (through M, so the armature's share is in it), `E_pot_rel` (integrated
  from `dV/dt = g'qd`, because the articulation will not cheaply hand back link centres of mass),
  `E_tot_rel`, per-joint `P_joint`, `P_mech`, `P_grav`, cumulative `W_mech`.
- Headroom (`tau_frac`, `tau_frac_max`) and pose quality (`sigma_min`, `sigma_max`, `cond_J`,
  `manip`, `cond_M`). The conditioning scalars use the Jacobian's LINEAR block only: the full 6xn
  mixes m/s rows with rad/s rows, so its singular values have no single unit.
- Tool velocity, and a per-run `summary` in the manifest (energy budget, worst headroom, closest
  approach to a singularity).
- The `attack_*` labels are gone (O5 is fault injection, not attacks) and `grip_active` / `box_index`
  replace the finger columns — so a commanded grip that failed to seal is now visible as
  `payload_attached` true with `grip_active` false.

**Pinocchio.** `model/pinocchio_model.py` gained the physics the telemetry needs on the model side:
`kinetic_energy`, `potential_energy`, `total_energy`, `joint_power`, `gravity_power`, `decompose`,
`operational_space_inertia`, `manipulability`, `singular_values`, `gravity_headroom`, and
`with_payload(mass, size=...)` — a carried box as a **real rigid body** with its own
`m/12 diag(y²+z², ...)` tensor at its own centre of mass, not m·g at the wrist. A box and a point
mass of equal mass give the same gravity torque and different mass matrices, which is where a
moving payload is actually identified from.

One correction worth recording: the docstring claimed `pin.computeKineticEnergy` ignores
`model.armature`. It does not — Pinocchio 4.1 accounts for it, and the two agree to 1e-15. The test
now pins both facts, so if a future Pinocchio changes that the suite fails instead of the twin
quietly becoming lighter than the plant.

**`pal_live.py` is ported.** It builds the conveyor cell and runs the three-box cycle, where it used
to build the Panda finger workcell and pick a single cube. The cycle itself moved into
`pal_tasks.cell_cycle()` — one definition shared by the watched demo, the live server and the
recorded suites, and buildable on the host with no Isaac, which is why `tests/test_suites.py` can
check it at all. The cell's geometry split into `pal_cell_geom.py` (plain numbers, no USD) to make
that possible. `pal_suites.live_pick` became `live_cycle`; `random_pick_xy` and the dashboard's
random-cube checkbox are gone (dashboard item 5).

One real bug this caught: the dashboard's fault onset was the constant **3.02 s**, the Panda pick's
grasp + 0.4. The cycle's first grip is elsewhere and moves with style and speed, and a wrong onset
puts the detector's baseline window on the far side of the fault — so it calibrates on faulty data
and finds nothing. It is now asked of the task (`engine.pick_fault_onset_s`, 3.81 s at the default).

**A second real bug, caught by making one constant honest.** `pal_faults.ARM` was still the
literal 7, so `FaultSpec`'s range check accepted joint 6 — and `pal_suites` aimed a backlash fault
there in two suites, because 6 was the Panda's last joint. On a six-joint arm that fault was being
written to a joint that does not exist. Setting `ARM = cfg.DOF` turned it into an immediate
`ValueError`; the suites now say `LAST_JOINT = cfg.DOF - 1`, which is what "the far end of the
chain" actually meant. Same fix in `pal_sweep.py` and `pal_selftest.py`. Suite counts unchanged.

**Tests: 161 passed, 57 skipped** (was 110 / 71).

- All **fifteen** per-test Panda quarantines are gone. `conftest.PANDA_ASSUMPTION` is deleted; the
  file now records why it existed and what it was for, because that list is what made the work cheap
  to pick up.
- New `tests/test_telemetry.py` (15): the writer end to end against a Pinocchio-driven stub, every
  identity between columns, decimation holding M *and* everything derived from it, and a written run
  validating against the schema through `npz_to_parquet`.
- New `tests/test_energy.py` (16): energy conservation in a free swing, the work-energy theorem, the
  armature's share, each term of the decomposition vanishing when it should, a payload's gravity
  torque as `J_p^T(0,0,mg)` swept over three offsets, and the elbow singularity.
- The 57 skips are the four recorded-Panda-evidence modules (56) and one missing-data skip. **The
  project still has no validated detection or payload result for the arm it runs** — that needs
  `tools/pal_suite.sh` on a GPU, and it is item 1 of docs/TODO.md.

Two numbers worth having, both derived rather than quoted:

- Payload torque at home is **6.78 N*m per kg** on shoulder-lift *and* elbow, 1.18 on wrist 1, 0 on
  the base. Those two are equal because the UR ready pose stands the upper arm vertically, putting
  both axes at the same x; the lever is 0.6914 m and 9.81 x 0.6914 = 6.783. Checked three ways
  (finite difference, Pinocchio's gravity term, the lever by hand).
- The phantom mass an empty arm reads under the +-3% model mismatch is now bounded as **0.4% of the
  arm's moving mass** rather than an absolute 50 g. The old bound was 0.3% of the Panda's mass, so
  carrying it over would have been *tighter* in real terms on a heavier arm — which is how an
  unscaled tolerance misleads.

## 2026-11-03 (23:43 IST) — dashboard fix and run instructions

- **Fixed:** after a replay session the server kept its engine in replay and paused, so a freshly loaded
  page showed "Stand-in plant" while the controls were greyed out ("+" and "More controls" did nothing).
  The layout is now built per page load (`serve_layout` in `dashboard/app.py`), which resets the engine to
  the stand-in plant, running, no fault. Checked in the browser: put the server in replay, reload, header
  reads stand-in, Pause shows, "+" and "More controls" respond. Consequence: reloading the page resets
  the session, and the server still has one engine, so use one browser tab.
- Run (inside `conda activate pal`): `python dashboard/app.py`, then http://127.0.0.1:8050. Without
  activating: `& "$env:USERPROFILE\miniconda3\Scripts\conda.exe" run -n pal --no-capture-output python dashboard/app.py`.
  The replay buttons need dev's two real logs in `data/real/` (not in git; `setup.md` section 4).
- Pushed: `nithya` at `0088d5e`.

## 2026-11-03 (23:30 IST) — dashboard, first version

- New `dashboard/` (Plotly Dash, `python dashboard/app.py`, see `dashboard/README.md`), built from the
  final design canvas and `docs/DASHBOARD_PLAN.md`: header with source switch, pause, guided demo and
  alert bell; O2 Arm & residual tab (arm twin with link tint, force arrow, manipulability ellipse and
  alert rings; payload, confidence and worst-residual cards; mock load, speed, per-joint speed and
  test-fault controls; one chart with a Residual / Torque / Confidence / Payload switch; residual bars
  that select the joint; torque breakdown); O6 Benchmark tab (placeholder for dev's trajectories, session
  evaluation, residual floor table, real empty-arm RMS computed from the log); right-hand drawer (O1, O3,
  O4 with tare, alerts, about); pop-up alerts; five-step guided demo.
- Two sources: a **stand-in plant** (our Pinocchio plant stepped live, labelled "not Isaac Sim") and a
  **replay of dev's real logs** (0 kg and 1 kg). The analysis is the repo's own code (`compute_residual`,
  confidence gates, payload formula) on a sliding window; the live detector reuses `detector.py`'s
  constants and adds a payload-signature fit and a tracking-lag estimate. Alerts checked in the browser:
  pickup gives a payload alert and 1.2 kg estimate, friction on joint 3 is named on joint 3, a 50 ms delay
  is caught on the tracking channel only, near-singular pose gates and holds the estimate, over-speed holds
  alarms.
- New `tests/test_dashboard_engine.py` (5 tests). Full suite: 70 passed.
- Bugs found in the browser and fixed: a callback that read and wrote the same slider looped; re-drawn joint
  buttons fired their click callback every tick (now fixed buttons, only their contents change); merged
  click+tick triggers hid the click (now read from `ctx.triggered`); 250 ms refresh lagged clicks (now 500 ms).
- Not done (waits on dev): live Isaac Sim feed and control channel, real fault runs, O6 trajectories.
  Playwright was installed in the `pal` env only to check the page; it is not needed to run it.

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
- Dev's docs were copied into `docs/dev_reference/` (unedited copies); removed in the merge, the originals live in `docs/` and the repo root.
- `docs/NOTES_FOR_DEV.md`: round 5 added (tests listed, armature finding, prioritised
  requests, questions on ETA).
- Set up on this laptop (RTX 3050, CPU only): Miniconda, the `pal` env, real logs from `dev`,
  and the synthetic set and confidence sweep regenerated locally (all outside git).

## 2026-11-03 (night) — setup file, dashboard plan in the repo, encoding repair

- New `setup.md`: how to continue on another machine (no GPU needed; install, environment,
  identity, checks), which files are not in git and how to restore each (real logs come from
  `origin/dev` via a temporary worktree), where we stopped, what is stale in `docs/ANALYSIS_CONTEXT.md`,
  an unverified lead on the simulator's mass matrix (J7 differs from ours by exactly the
  armature, 0.0061), a start prompt for Claude Code, and next steps.
- New `docs/DASHBOARD_PLAN.md`: the revised dashboard design (canvas URLs, decisions, theme,
  colours, 50% trusted-speed band, build order). It had only existed in Claude's local memory.
- `docs/ANALYSIS_CONTEXT.md`: banner pointing to `setup.md` and `DASHBOARD_PLAN.md`; content unchanged.
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
- `docs/ANALYSIS_CONTEXT.md`: conventions (row timing, payload placement, log delivery), open items, next steps.

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
- `docs/ANALYSIS_CONTEXT.md` updated: conventions, how to run, key numbers, open items, next steps.

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
- New `docs/ANALYSIS_CONTEXT.md` and this `changes.md`.

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
