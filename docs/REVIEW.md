# External-style review: where PAL stands against the proposal

Written for: the two people on the project, and anyone who has to decide whether the numbers can
be trusted. First review 2026-11-03; updated the same day after the fixes it called for. Read
against the proposal PDF in the repo root and `docs/PROPOSAL_AMENDMENTS.md`.

## Verdict

Everything the simulator side owns is now built, and each part has a check that can fail. The
first review found two serious defects (the plant's friction was wrong and `tau_measured` could
not see faults); both are fixed and the fixes are verified. The repeated pattern in this project
is that a passing check hid a problem, so this page lists what each result does and does not show.

**Not done:** the dashboard (the analysis side's), and our websocket stream and in-simulator panel.
**Done 2026-11-04:** Pinocchio installs here (uv, Python 3.12), the analysis side's branch is merged, and their real detector has
been run on our data (`docs/DETECTION_RESULTS.md`): with the applied torque it finds 15 of 32 faulty runs and has no false
alarms; with `tau_measured` it finds 8 and never finds friction. (An earlier claim of a noise floor from their acceleration
estimate was my error and is corrected there.)

## Findings and their status

| # | Finding | Status |
|---|---|---|
| F1 | Friction was passed in N*m to a dimensionless PhysX coefficient: ~38 N*m on joint 2 instead of ~0.2 | **Fixed and verified** |
| F2 | `tau_measured` is net torque, so friction, backlash and a locked joint are invisible to it | **Fixed**: effort mode, applied torque is the signal |
| F3 | Attacks dropped; the proposal includes them | **Decided and written up** (`PROPOSAL_AMENDMENTS.md`) |
| F4 | The proposal's required plant/model mismatch was missing | **Fixed**: 5% arm-mass mismatch on every run |
| F5 | The O5 data was schema-invalid | **Fixed**; `tools/pal_suite.sh` now checks every run |
| F6 | Deterministic runs, so no false-alarm statistics | **Fixed**: seeded sensor noise |
| F7 | The 5% O2 criterion has no denominator | **Reported both ways**; the stricter reading exposes a real limit (below) |
| F8 | Reproducibility depends on files outside the repo | **Open** |
| F9 | The validation push first failed its pre-set tolerance (15.8% with simulator terms) | **Resolved**: passes (worst 9.9%) with the independent model that includes friction; both scorings in `docs/VALIDATION.md` |
| F10 | Mild backlash and delay leave no measurable signature | **Open**; confirmed at 20% and at 40% speed (`out/fast`), so not a speed effect |
| F12 | The analysis side's model subtracted friction from `tau_measured`, which is net torque and has none (bias up to 0.2 N*m on payload and wrench) | **Fixed**: manifest key `tau_measured_includes_friction`; empty-arm identity now under 0.1% of every limit |
| F13 | Windowed runs were time-warped: with the default 1/30 s rendering interval each rendered step silently advanced 33 physics steps, so the arm moved 33x as far on every rendered row and the dynamics checks broke while moving | **Fixed** (rendering interval = physics step in a window); verified: the per-row motion ratio went from 33.3 to 1.00 and a windowed run reproduces the headless one |
| F14 | The "pick" was not a pick: no IK, no gripper motion, the payload was an invisible mass and the cube a drawing | **Fixed**: IK pick-and-place in three styles, and a physical cube gripped by driven fingers (`docs/PICK_AND_GRIPPER.md`) |
| F15 | A healthy pick false-alarms the detector: the first 3 s of calibration never see the carry, and a payload pickup steps the residual | **Fixed by an opt-in**: calibrate against a healthy run of the same task on both channels (`--reference`); subtracting the payload torque alone does not fix it |
| F16 | The payload estimate on a REAL cube at 2 kg is 25-36% low (modelled mass: 1-2%) | **Open**, cause not established; got worse with a harder grip |
| F11 | Only the inertia diagonal was authored (found by the analysis side); the simulator's `M` was off by up to 0.015 kg*m^2 | **Fixed and verified** against an independent mass matrix (`pal_selftest.py` T5, 0.00004 kg*m^2) |

## What each check shows

| check | result | repeat with |
|---|---|---|
| All tests, both teams (pytest, Pinocchio env): our injectors, noise, trajectories, kinematics, mass matrix; their model physics, observers, proposal table, dashboard engine; the real-fault regression tests | 138 pass | `python3 -m unittest discover tests` |
| Plant self-test (in the sim) | 8 of 8 | `tools/run_sim.sh pal_selftest.py` |
| Friction is in N*m | 1.0 N*m injected reads 1.008 / 1.001 / 1.001 on joints 2, 4, 6 | self-test T2 |
| Zero-friction identity closes | 0.004-0.008 N*m | self-test T1 |
| Payload and mass mismatch change mid-run | gravity matches the independent URDF calculation to 0.0002 N*m at 0 kg and 3 kg | self-test T3 |
| The simulator's mass matrix (not just gravity) | simulator `M` + 0.0061 armature equals an independent numpy composite-rigid-body `M` to 0.00004 kg*m^2 at three poses, 0 and 2 kg; the old diagonal-only plant was off by 0.015 | self-test T5 |
| An independent model reproduces the plant's torques | numpy model (M, Coriolis, g) vs `tau_applied` on a clean run: residual equals the configured friction | `docs/PREVIEW_DETECTION.md` |
| Every captured run | schema, URDF-numpy gravity (0.000-0.002 N*m), identity on healthy rows, friction-scale guard | `tools/check_run.py` |
| The checker can fail | a copy of the URDF with one link 15% heavier gives a 0.56 N*m error and FAIL | `check_run.py --urdf` |
| Refactor safety | bit-identical to the pre-refactor runs | recorded 2026-11-03 |
| Payload data (O4) | 12 held runs recover 0.5-3 kg to within 0.02% (independent URDF kinematics, tare at 0 kg); the 4 moving runs are not scored | `tools/check_payload.py out/o4` |
| Validation push | see `docs/VALIDATION.md` | `tools/check_push.py out/push/*.parquet` |

## F1 and F2 in numbers

Effort mode, joint 2, mean of `|tau_cmd - (M qdd + C + g)|`, which is the friction actually acting:

| friction setting | friction acting on joint 2 |
|---|---|
| none | 0.006 N*m (the identity closes) |
| old plant: Gaz values as a dimensionless coefficient | 38.4 N*m |
| coefficient 1.4 (what "x8" meant in the first sweep) | 26.7 N*m |
| torque-valued API, 1.4 N*m | 1.388 N*m |

With the fix, healthy runs show `tau_applied - net` of 0.13-0.40 N*m RMS per joint, the Gaz scale.
In the old plant, `tau_measured - net` stayed at 0.001 N*m with friction x8 or a locked joint; with the
applied torque, friction x8 on joint 4 reads 0.39 -> 2.85 N*m and a locked joint 9.4 N*m.
`tau_measured` is still the right signal for an **external** load (a payload, a push).

## What the O5 fault set can and cannot show (F10)

26 runs, effort mode, 20% of velocity limit, fault at 3 s of 6 s, noise on every run (20 faulty, 6 healthy).

| fault | visible in the applied-torque residual? | visible in tracking? |
|---|---|---|
| friction x2 / x4 / x8, joints 2, 3, 4, 6 | yes, grows with severity (joint 4: 0.39 -> 0.78 / 1.50 / 2.85 N*m) | mostly at x4 and above |
| locked joint 4 | yes, 9.3 N*m | yes, 0.059 rad RMS |
| backlash 0.02 / 0.06 / 0.12 rad, joints 1, 7 | **no** | **no** (changes under 0.001 rad) |
| command delay 10 and 30 ms | **no** | **no** |
| command delay 50 ms | yes, 2.6 N*m: the loop oscillates | yes, 0.13 rad |

Backlash is expected to be a tracking-channel fault, not a residual one (the applied torque stays
consistent with the dynamics), and at slow, smooth motion it withholds under 2% of the torque; my
backlash model has no impact dynamics. So six of the 20 faulty runs (four backlash, two short delays) are indistinguishable from
healthy in this simulation. They stay in the set because a defined fault set should include the cases
a detector will miss, and the detection rate must be reported with them in. Faster motion or a model
with impacts would be needed to expose them.

## F7: the O2 criterion

`tools/check_run.py` prints the residual as a percentage of the torque limit (lenient) and of the
joint's own torque (strict). The clean reference runs pass both, worst 0.58% of own torque. With
0.02 N*m sensor noise the O5 runs fail the strict reading on joint 7 (81%), because that joint
carries almost no torque. So a 5%-of-own-torque criterion cannot be met on lightly loaded joints
under any realistic sensor noise; the criterion should be stated against the torque limit.

## F8: still open

- `tools/run_sim.sh` expects the Isaac cache in a sibling directory (`../Physyk_AI-master/.isaac-cache`).
- The robot USD comes from Isaac's asset server, not from the repo.
- The image needs an NVIDIA GPU and an NGC login.
- Pinocchio needs its own environment: `uv venv --python 3.12 .venv-pal` (the system Python 3.14 stalls pip on `pin`).

## Objective by objective, against the proposal

| | proposal asks | state | honest gap |
|---|---|---|---|
| O1 telemetry | every quantity each physics step, no dropped frames | all quantities per step at 1 kHz, schema-valid, row counts verified | `M` and `J` held for 10 steps between reads; recorded to file, not streamed |
| O2 residual | model within 5% on a slow unloaded run | **the independent Pinocchio model closes the clean empty-arm run to under 0.1% of every joint's limit** (`[0.007, 0.010, 0.011, 0.006, 0.076, 0.082, 0.001]` %); simulator terms 0.01%; URDF gravity matches independently | fails above ~50% speed (artifact) |
| O3 confidence | removed on 2026-11-05 (`PROPOSAL_AMENDMENTS.md` section 5, `DECISIONS.md` D6) | the 0-1 score, its panel and the payload gate are gone; the detector keeps a pose mask (`observers/pose_mask.py`) | tested before removal: the payload estimate is only wrong while moving through the straight-up elbow-extended pose; the mask matters only with the arm stretched to about 0.78 m |
| O4 payload | estimate converges, within a stated tolerance | known-mass data, 0.5-3 kg, verified to 0.02% against independent kinematics; **the analysis side's real estimator with a tare recovers all 15 loaded runs (held at three poses, and moving) to within 1.1%** (`tools/run_payload_estimator.py`); untared it reads ~0.25 kg high from the 5% mass mismatch | payload is a rigid point mass at the TCP, not a grasped object; the estimator itself is theirs |
| O5 detection | detection and false-positive rate over a defined fault set, with and without the gate | injectors, 26 labelled runs, matched baselines, the C-list in `out/extra`, noise-free twins in `out/clean`; **their real detector finds 15 of 32 faulty runs with the applied torque (8 with `tau_measured`), 0 false alarms in 9** | the gate was on and never fired; friction is found from about x4 upward, x3 with the opt-in baseline that needs the calibration window to cover the motion (`DETECTION_RESULTS.md`) |
| O6 benchmark | cycle time, peak torque, energy per pick for three styles | built twice: out-and-back (26 runs, below) and on a real IK pick cycle (18 runs, `docs/o6pick_comparison.svg`) | one pick location, two payloads |
| O7 presentation | streamed dashboard and in-sim panel | the simulator runs in a window with a physical table and cube (`tools/pal_suite.sh --window`) | the stream to the dashboard and the in-sim panel are not built |
| Validation | known force against estimate | built and run: 24 runs; **passes with the independent model (worst 9.9% against 15%)** | with simulator terms it missed by 0.8 points on one run, cause and control in `VALIDATION.md` |

### O6 on a real pick cycle (`out/o6pick`, `tools/summarize_o6pick.py`)

Shortest cycle time that keeps peak torque within a budget, home to home through the whole IK pick (the proposal's "energy per pick"):

| payload | budget | trapezoidal | S-curve | torque-aware |
|---|---|---|---|---|
| 0 kg | 55% | 5.67 s | 6.09 s | 7.54 s |
| 0 kg | 70% | 4.35 s | 5.10 s | 5.64 s |
| 2 kg | 70% | 8.03 s | 6.09 s | 7.54 s |

The 2 kg 70% row is knife-edge: the 5.67 s trapezoid peaks at 70.4%, just outside the budget. Nothing meets 40% or, at 2 kg, 55%. The fastest runs reach 96-97% of the
velocity limit, above the 50% regime, so they are for time, torque and energy only (never the residual).

### O6 out-and-back (the proposal's headline question)

Shortest cycle time that keeps peak torque within a budget (fraction of datasheet limits), out-and-back
between two joint configurations; chart in `docs/o6_comparison.svg`:

| payload | budget | trapezoidal | S-curve | torque-aware |
|---|---|---|---|---|
| 0 kg | 40% | 1.39 s | 1.60 s | 1.60 s |
| 0 kg | 55% | 1.39 s | 1.60 s | 1.30 s |
| 2 kg | 40% | none | none | none |
| 2 kg | 55% | 1.66 s | 2.03 s | 2.00 s |
| 2 kg | 70% | 1.39 s | 1.60 s | 1.60 s |

The plain trapezoid is fastest at equal peak torque; the S-curve costs about 15% more time for
smoothness and does not lower the peak torque; the quintic wins only at a loose budget. Two caveats:
the S-curve uses a 100 rad/s^3 smoothness limit (a choice; at Franka's 7500 rad/s^3 hardware figure it was
indistinguishable from the trapezoid, which is how the first run of this benchmark came out degenerate),
and at 0 kg the limit is the kinematic acceleration bound (10 rad/s^2, a choice), not torque.

## The analysis side's progress (read 2026-11-04, `origin/nithya` at `0088d5e`)

They have built: the independent Pinocchio model and schema validator; four observers (residual, confidence,
payload, detector); a synthetic-log generator; **70 tests passing**, including physics properties of the model
(skew-symmetry of `Mdot - 2C`, energy conservation, a push recovered through `J^T`) and one test per row of the
proposal's evaluation table; and a **first dashboard** (Plotly Dash: arm twin, residual/torque/confidence/payload views,
guided demo, alerts) running on a stand-in plant and on a replay of our logs. They are waiting on us for real fault runs,
sensor noise, a longer run list, the `speed_fraction` field, and answers about the armature.

Two things in their branch need to change because of what we found, both written up for them in
`docs/NOTES_FOR_NITHYA.md`: (1) their round 4 said no effort-mode datasets and asked for position-mode faults, but
`tau_measured` cannot see friction; their `residual.py` already supports `torque="applied"` for torque-mode logs, but the
detector's command-line entry calls it without that argument; (2) the dashboard replays our **old** logs (wrong friction and
diagonal-only inertia). Their synthetic "8 of 8 faults detected" should not be read as a statement about this plant.
They also found F11 (the armature lead, and the off-diagonal inertia).

## Remaining work

1. **Analysis side:** run `observers/` on the payload data `out/o4` (not yet run by anyone); consider a pose-dependent calibration
   baseline for the detector (sensitivity limit in `docs/DETECTION_RESULTS.md`); report
   detection and false-positive rates with and without the gate; run the payload estimator on `out/o4`.
2. **Ours, not done:** websocket stream of telemetry and the in-simulator panel (O7); a grasped payload
   with gripper actuation; backlash with impact dynamics; faster-motion fault sets.
3. **Cleanup:** the probe scripts written before 2026-11-03 build the old plant; their friction-related
   results are superseded (banner in `CONTEXT.md`).
