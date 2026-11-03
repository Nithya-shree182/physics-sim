# Problem board (2026-11-04, after merging the two branches)

Written for: both people. One line per problem; what is fixed is listed so nobody re-investigates it.

## Open

| # | Problem | Why it matters | Where / next step |
|---|---|---|---|
| 1 | Friction at x2 (about 0.4 N*m on a joint) is not found by any variant | the proposal's maintenance case ("friction growth early") | below the calibration spread; needs a longer calibration or a model-based baseline, `DETECTION_RESULTS.md` |
| 2 | Backlash (0.02-0.12 rad) and delays up to 30 ms are not found, at 20% or 40% speed | two of the schema's fault classes are undetectable here | the residual cannot see them by construction; needs a different signal (spectral content, impact) or an honest statement in the results; REVIEW F10 |
| 3 | The friction baseline (finds x3) is opt-in because it false-alarms when the calibration window does not cover the motion | a hidden precondition | `recalibrate_baseline` docstring; a pick-and-place cycle or a repeated reference satisfies it |
| 4 | Sensor noise levels are representative, not measured | cannot claim a real-arm figure | detection barely moves with noise (noisy vs `out/clean`), so low priority; recorded in every manifest |
| 5 | The payload is a rigid point mass at the TCP; no gripper actuation, no grasp | the proposal's demo has the arm pick an object up | needs gripper control in the scene |
| 6 | No live feed or control channel from the simulator to the dashboard; no in-simulator panel | O7 as written is "live" | the dashboard replays references and runs a stand-in plant; the runner already takes payload, fault and speed as parameters |
| 7 | No impact/collision injector | the analysis side's C7, spike detection | not built |
| 8 | `TORQUE_CONVENTION` is still "unverified" | user's decision to leave it | evidence is in the manifests (`tau_measured_includes_friction: false`, net torque, no gearbox modelled) |
| 9 | Reproducibility depends on files outside the repo | a fresh clone cannot run the simulator | `../Physyk_AI-master/.isaac-cache`, Isaac's asset server, the NGC image; REVIEW F8 |
| 10 | The old probe scripts build the pre-fix plant | their friction numbers are superseded | banner in `CONTEXT.md`; the rebuilt scripts are `pal_*` listed in `README.md` |

## Decisions already made (do not reopen)
Faults, not attacks. Fault data in effort mode, with the applied torque as the fault signal. 50% speed regime for detection data.
Friction in N*m through the torque-valued API. 5% arm-mass mismatch and seeded sensor noise on every fault run.

## Fixed since the first review (all verified, see `REVIEW.md`)
| Problem | Fix |
|---|---|
| Friction passed in the wrong units (about 200x too much) | torque-valued API, `pal_selftest.py` T2 |
| `tau_measured` blind to friction and locked joints | effort mode, applied torque, `residual.fault_torque` |
| Inertia diagonal-only (mass matrix off by 0.015 kg*m^2) | full 3x3 tensors, selftest T5, 0.00004 |
| The model subtracted friction from the net `tau_measured` | manifest flag `tau_measured_includes_friction`; empty-arm identity now under 0.1% of every limit |
| Validation push missed its 15% rule by one run | scored with the independent model (friction included): worst 9.9%, passes |
| O5 data schema-invalid | `check_run.py` in every pipeline |
| My own wrong claim of a noise floor in their pipeline | I ran it without their 20 Hz filter; corrected everywhere, lesson saved |
| Tests and dashboard used the old wrong-plant logs | repointed to `data/ref_matched_*` |
| Two branches with no common history, a `context.md`/`CONTEXT.md` case clash | merged; theirs is `docs/ANALYSIS_CONTEXT.md` |
| Payload estimator never run on our data | 15 loaded runs within 1.1% (tared) |
| Real-fault findings not protected | 14 regression tests in `tests/test_real_faults.py`, 7 in `tests/test_baseline.py` |
