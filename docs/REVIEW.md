# External-style review: where PAL stands against the proposal

Written for: the two people on the project, and anyone who has to decide whether the numbers can
be trusted. First review 2026-11-03; updated the same day after the fixes it called for. Read
against the proposal PDF in the repo root and `docs/PROPOSAL_AMENDMENTS.md`.

## Verdict

Everything the simulator side owns is now built, and each part has a check that can fail. The
first review found two serious defects (the plant's friction was wrong and `tau_measured` could
not see faults); both are fixed and the fixes are verified. The repeated pattern in this project
is that a passing check hid a problem, so this page lists what each result does and does not show.

**Not done, and cannot be from this side:** running the analysis side's detector, estimator and
Pinocchio model on this data (Pinocchio does not install on this machine), the dashboard, and the
websocket stream and in-simulator panel.

## Findings and their status

| # | Finding | Status |
|---|---|---|
| F1 | Friction was passed in N*m to a dimensionless PhysX coefficient: ~38 N*m on joint 2 instead of ~0.2 | **Fixed and verified** |
| F2 | `tau_measured` is net torque, so friction, backlash and a locked joint are invisible to it | **Fixed**: effort mode, applied torque is the signal |
| F3 | Attacks dropped; the proposal includes them | **Decided and written up** (`PROPOSAL_AMENDMENTS.md`) |
| F4 | The proposal's required plant/model mismatch was missing | **Fixed**: 5% arm-mass mismatch on every run |
| F5 | The O5 data was schema-invalid | **Fixed**; `tools/pal_pipeline.sh` now checks every run |
| F6 | Deterministic runs, so no false-alarm statistics | **Fixed**: seeded sensor noise |
| F7 | The 5% O2 criterion has no denominator | **Reported both ways**; the stricter reading exposes a real limit (below) |
| F8 | Reproducibility depends on files outside the repo | **Open** |
| F9 | The validation push first failed its pre-set tolerance | **Diagnosed; two verdicts reported** (`docs/VALIDATION.md`) |
| F10 | Mild backlash and delay leave no measurable signature | **Open, a limit of the fault set** |

## What each check shows

| check | result | repeat with |
|---|---|---|
| Unit tests: injectors, noise, trajectories, URDF kinematics | 26 pass | `python3 -m unittest discover tests` |
| Plant self-test (in the sim) | 7 of 7 | `tools/run_sim.sh pal_selftest.py` |
| Friction is in N*m | 1.0 N*m injected reads 1.008 / 1.001 / 1.001 on joints 2, 4, 6 | self-test T2 |
| Zero-friction identity closes | 0.004-0.008 N*m | self-test T1 |
| Payload and mass mismatch change mid-run | gravity matches the independent URDF calculation to 0.0000 N*m at 0 kg and 3 kg | self-test T3 |
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

23 runs, effort mode, 20% of velocity limit, fault at 3 s of 6 s, noise on every run.

| fault | visible in the applied-torque residual? | visible in tracking? |
|---|---|---|
| friction x2 / x4 / x8, joints 2, 4, 6 | yes, grows with severity (joint 4: 0.39 -> 0.78 / 1.50 / 2.85 N*m) | mostly at x4 and above |
| locked joint 4 | yes, 9.4 N*m | yes, 0.059 rad RMS |
| backlash 0.02 / 0.06 / 0.12 rad, joints 1, 7 | **no** | **no** (changes under 0.001 rad) |
| command delay 10 and 30 ms | **no** | **no** |
| command delay 50 ms | yes, 2.6 N*m: the loop oscillates | yes, 0.13 rad |

Backlash is expected to be a tracking-channel fault, not a residual one (the applied torque stays
consistent with the dynamics), and at slow, smooth motion it withholds under 2% of the torque; my
backlash model has no impact dynamics. So six of the 17 faulty runs (four backlash, two short delays) are indistinguishable from
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
- `pip install pin` does not complete on this machine, so the analysis side's code has not been run here.

## Objective by objective, against the proposal

| | proposal asks | state | honest gap |
|---|---|---|---|
| O1 telemetry | every quantity each physics step, no dropped frames | all quantities per step at 1 kHz, schema-valid, row counts verified | `M` and `J` held for 10 steps between reads; recorded to file, not streamed |
| O2 residual | model within 5% on a slow unloaded run | simulator terms: 0.01% of limit, 0.58% of own torque (clean); URDF gravity matches independently | needs the analysis side's Pinocchio model run on `data/ref_matched_*`; fails above ~50% speed (artifact) |
| O3 confidence | 0-1 score gating O4 and O5 | analysis side's | their rescaled score differs from the proposal's raw ratio, deliberately |
| O4 payload | estimate converges, within a stated tolerance | known-mass data, 0.5-3 kg: 12 held runs at three poses verified to 0.02%; 4 moving runs recorded but not scored here | payload is a rigid point mass at the TCP, not a grasped object; the estimator itself is theirs |
| O5 detection | detection and false-positive rate over a defined fault set, with and without the gate | injectors, 23 labelled runs, 6 seeded healthy runs | rates need their detector run on these files (F10 limits what is findable) |
| O6 benchmark | cycle time, peak torque, energy for three styles | built and run: 26 runs, table below | single waypoint pair, two payloads; no pick-and-place with a gripper |
| O7 presentation | streamed dashboard and in-sim panel | not built here | dashboard is theirs; stream and in-sim panel are ours and not done |
| Validation | known force against estimate | built and run: 24 runs | strict pre-set criterion failed by 0.7 points on one run (`VALIDATION.md`) |

### O6 result (the proposal's headline question)

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

## Remaining work

1. **Analysis side** (needs Pinocchio): run `observers/` on `data/ref_matched_*`, `out/o5`, `out/o4`; report
   detection and false-positive rates with and without the gate; run the payload estimator on `out/o4`.
2. **Ours, not done:** websocket stream of telemetry and the in-simulator panel (O7); a grasped payload
   with gripper actuation; backlash with impact dynamics; faster-motion fault sets.
3. **Cleanup:** the probe scripts written before 2026-11-03 build the old plant; their friction-related
   results are superseded (banner in `CONTEXT.md`).
