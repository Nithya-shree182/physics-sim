# External-style review: where PAL stands against the proposal

Written for: the two people on the project, and anyone who has to decide whether the numbers
can be trusted. Reviewed 2026-11-03, `dev` branch, against the proposal PDF in the repo root.

## Verdict

The simulation core is sound and reproducible: the arm's gravity terms match an independent
URDF calculation to three decimals, the dynamics identity closes, and a refactor reproduced
earlier runs bit for bit. **But two findings mean the fault-detection objective (O5) cannot
be met the way it is currently set up**, and one scope decision departs from the proposal.
None of this was visible from the passing checks; each was found by testing a claim directly.

| # | Finding | Severity |
|---|---|---|
| F1 | The plant's friction is not what we think it is (wrong units, up to ~200x too large on joint 2) | **Critical** |
| F2 | `tau_measured` cannot see friction or a stuck joint at all; O5 needs the *applied* torque | **Critical** |
| F3 | We dropped attacks; the proposal includes them, and its demo needs one | Scope change, needs a decision |
| F4 | The proposal's required plant/model mismatch is not in place | High |
| F5 | The O5 data sent to the analysis side was schema-invalid (now fixed) | Fixed |
| F6 | Runs are deterministic, so no false-alarm rate can be measured yet | Medium |
| F7 | The 5% O2 criterion is ambiguous and we use the lenient reading | Medium |
| F8 | Reproducibility depends on files outside the repo | Medium |

## What was checked, and how to repeat it

| check | result | repeat with |
|---|---|---|
| Gravity `g(q)` from the simulator against `g(q)` recomputed from the URDF with plain numpy (no PhysX, no Pinocchio) | worst error 0.000 N*m on all 7 joints, on the 14 O5 runs and the 0 kg reference run | `tools/check_run.py <run>.parquet` |
| The check can fail | with link 4 mass raised 15% in a copy of the URDF it reports a 0.56 N*m error and FAIL | `--urdf <modified.urdf>` |
| Identity `tau_measured` vs `M qdd + C + g` (proposal O2) | 0.00-0.01% of torque limit RMS per joint, limit 5% | same |
| Schema validator | pass on all 14 O5 runs (after the fixes in F5) | same |
| Refactor safety: shared plant builder vs the old copies | max abs difference 0.0 over every float column, two runs | compare `out/o5` parquet before/after |
| Plant and probes grep for hardcoded or fabricated results | none substitute for a measurement; constants listed in F8 | `grep -n 4.6306 workspace/pal_*.py` |

## F1. The plant's friction is in the wrong units and far too large

`pal_plant.py` (and every script before it) sets joint friction with `set_friction_coefficients`,
passing the Gaz et al. values (0.15-0.36 N*m, times 1.15). **That call takes a dimensionless
PhysX coefficient, not a torque.** The torque it produces is that coefficient times the joint's
internal reaction force, which on a loaded arm is tens of N*m.

Measured (`pal_friction_visibility.py`, effort mode, joint 2, 20% speed, mean of
`|tau_cmd - (M qdd + C + g)|`, which is the friction torque actually acting):

| friction setting | friction torque acting on joint 2 |
|---|---|
| none | **0.006 N*m** (the identity closes) |
| plant default: Gaz values passed as coefficients (0.175) | **38.4 N*m** |
| coefficient 1.4 (what "x8" meant in the first O5 sweep) | **26.7 N*m** |
| new API, 1.4 N*m torque effort | **1.388 N*m**, as intended |

Gaz et al. measured 0.15-0.36 N*m Coulomb friction on a real Panda, so joint 2 is about 200x too large. So every run on this plant, including
the reference runs in `data/`, has been running with severe unrealistic friction. It probably also
contributes to the 0.065 rad tracking error seen on healthy runs (not tested separately). The first O5 sweep's "friction x2/x4/x8"
multiplied a coefficient, not a torque, and its severities are meaningless in N*m.

The right API exists in this Isaac build: `art._physics_view.set_dof_friction_properties`
takes static effort, dynamic effort (both N*m) and a viscous coefficient. `pal_config.py`
already documents friction in those terms; the plant just never used them.

**Fix:** switch `pal_plant.py` to the effort-based API with Gaz values, then regenerate
everything. Every earlier friction-related number is superseded.

## F2. `tau_measured` is blind to friction and to a stuck joint

`tau_measured` behaves as the **net** torque on the joint, which equals `M qdd + C + g` by
Newton's law whatever produced it. Evidence from the 14 O5 runs, using simulator terms:

| run | residual `tau_measured - (M qdd + C + g)` on the faulted joint, post-fault |
|---|---|
| healthy | 0.001 N*m |
| friction x8, joint 2 | 0.002 N*m |
| friction x8, joint 4 | 0.001 N*m |
| joint 4 locked (qd ~ 0.0001 rad/s) | 0.001 N*m, and `tau_measured` stays within 0.3 N*m of gravity |

So in this simulator a residual built on `tau_measured` cannot detect friction, wear or a
locked joint. (Payload is different: a payload is an *external* load the model does not
know about, so it does show up. That is why O4 works.) The analysis side's synthetic logs
appear to put friction into `tau_measured` (their generator scales friction and detects it
through the residual); real PhysX data does not, so their synthetic detection results do not
transfer.

What does show friction is the **applied** torque: `tau_applied - (M qdd + C + g)` equals the
friction (1.388 N*m for 1.4 injected, table in F1). That signal exists only in **effort mode**,
where we command torque; in position mode `tau_applied` is identically zero.

This undoes the earlier decision to record O2/O5 in position mode, which rested on
`INTEGRATION.md`'s "the residual definition is forced by the control mode". That holds for
payload and fails for friction faults.

**Fix:** record O5 friction/stuck-joint data in effort mode with `tau_applied` logged, and
define the fault residual as `tau_applied - tau_model`. Delay and tracking-based faults can
stay in either mode. The analysis side's `residual.py` currently refuses `tau_applied` on a
position-mode log, which is correct, so it needs an effort-mode path.

## F3. We dropped attacks; the proposal has them

The proposal's O5 produces "spoofed-setpoint alerts", section 7.5 injects altered waypoints
and spoofed setpoints over ROS 2, and demo step 3 is a tampered waypoint with an alert.
The project owner decided on 2026-11-03 that nobody tampers with goals; only arm faults are
in scope. That is a legitimate descope, but it is a departure, and the demo and evaluation
text must change with it (cut order 3 in `WORK_SPLIT.md` already allows it).

One physical point the proposal gets wrong, which the descope sidesteps: it says a tampered
waypoint "looks plausible but the residual rises". Measured here, a waypoint that is
faithfully executed produces no residual (the physics is consistent). Only a command the
arm cannot follow consistently would. Anyone who restores the attack scope must state that.

## F4. The required plant/model mismatch is not in place

Proposal section 7.5 and 9: the twin's model must be deliberately different from the plant
(altered link masses, extra friction), "otherwise detection would succeed trivially". Today the
plant's masses equal the model's exactly (both authored from the URDF). Friction differs, but
by the wrong amount (F1). Mass mismatch has not been built. Until it is, detection results
would be reported against a near-perfect model, which the proposal forbids.

## F5. The O5 data we sent was schema-invalid (fixed)

The new `tools/check_run.py` ran the schema validator on the first sweep and found: seven
required manifest fields missing (`git_commit`, `machine`, `solver`, `asset`, `gravity`,
`seed`, `finger_q_ref`), and the delay runs carried `fault_joint = -1` while `fault_type` was
`delay` (the schema allows -1 only for no fault). The delay parameter was also in milliseconds
where the analysis side's logs use seconds. All fixed and re-generated; all 14 runs now pass.
`NOTES_FOR_NITHYA.md` told them the data was ready; it was not. Treat that data as superseded
anyway, because of F1.

## F6. Deterministic runs, so no false-alarm statistics

Two runs of the same scenario are bit-identical. That is good for reproducibility and bad
for the proposal's O5 metric (detection **and false-positive rate**): repeating a healthy run
gives the same answer every time. Needs sensor noise, varied trajectories or seeds.

## F7. The O2 criterion

"Within 5 percent per joint" has no stated denominator. We use percent of the joint's torque
limit, which is the lenient reading (a joint carrying 2 N*m of a 87 N*m limit can be 100% off
and still pass). Both readings should be reported. On the current data the identity is far
inside either, so this is a reporting issue, not a failure.

## F8. Reproducibility depends on things outside the repo

- `tools/run_sim.sh` expects the Isaac cache in a sibling directory
  (`../Physyk_AI-master/.isaac-cache`); a fresh clone will not have it.
- The robot USD comes from Isaac's asset server (`get_assets_root_path`), not from the repo.
- The Isaac image is `nvcr.io/nvidia/isaac-sim:6.0.1` (needs an NVIDIA GPU and an NGC login).
- Constants that look like results but are inputs worth deriving rather than typing:
  `4.6306` N*m per kg (divisor in `pal_author_plant.py`, `pal_experiment.py`,
  `pal_fast_random.py`), the payload offset `0.1034` m, drive gains 400/80, trajectory
  frequencies. They can all be computed from the URDF (`tools/check_run.py` already has the
  kinematics to do so).
- The earlier rate sweep that said the residual was "first order in dt" was invalid
  (World rebuilt in one process, so the new `physics_dt` was ignored); corrected earlier today.

## Objective by objective, against the proposal

| | proposal asks | state today | honest gap |
|---|---|---|---|
| O1 telemetry | every quantity each physics step; no dropped frames | all quantities per step at 1 kHz, 6000 rows per 6 s, schema-valid | `M` and `J` are re-read every 10th step and held; recorded to file, not streamed (0.76x real time) |
| O2 residual | model vs measured within 5% on a slow unloaded run | 0.01% on simulator terms; independent URDF gravity matches | passes only on slow motion; above ~50% speed an artifact dominates; **real test needs Pinocchio on real data, not run** |
| O3 confidence | 0-1 score flags near-singular poses | analysis side's, with a rescaled definition (the proposal's sigma_min/sigma_max mixes units) | deviation from the proposal's formula is deliberate and should be written up |
| O4 payload | estimate converges, paused at low confidence | chain verified to 0.04% at rest, 0.1-3.0 kg | payload is a rigid point mass at the TCP, not a grasped object; no grasp; real-data run not done |
| O5 detection | fault classes + spoofed-setpoint alerts, rates with/without the gate | friction/stuck/delay injectors exist | F1, F2, F3, F4, F6; backlash and noise missing; detector never run on real PhysX data |
| O6 benchmark | cycle time, peak torque, energy for 3 trajectory styles | not started | also the proposal's headline question (fastest motion for a payload) |
| O7 presentation | streamed dashboard + in-sim panel | not started here (dashboard is the analysis side's) | websocket stream and in-sim panel are ours |
| Validation | known force applied in low-gain torque mode vs estimate | effort mode works | no force is applied yet |

## How we get there

Each step has a test that decides whether it is done.

1. **Fix the friction (F1).** Switch `pal_plant.py` to the effort-based friction API with Gaz
   values. *Done when:* `pal_friction_visibility.py` shows `tau_cmd - net` equal to the
   injected N*m (within 10%) on three joints, and a zero-friction plant gives 0.006 N*m.
2. **Make faults observable (F2).** Add an effort-mode capture that logs `tau_applied`, with a
   computed-torque controller using the model's terms (as in `pal_effort_test.py`).
   *Done when:* a friction x4 run shows `tau_applied - tau_model` rising by about the injected
   torque at the faulted joint while a healthy run stays near zero.
3. **Build the missing injectors**: mass mismatch (F4), sensor noise (F6), backlash. Re-run the
   sweep through `tools/pal_pipeline.sh`; *done when* every run passes `check_run.py`.
4. **Hand over and run the detector** on the real runs (analysis side, needs Pinocchio on a
   machine that can install it). *Done when* detection and false-positive rates are reported
   with and without the gate, as section 9 requires.
5. **Payload with a grasp**, then the **validation push** (apply a known force, compare).
6. **O6** trajectory generators and benchmark; **O7** stream and in-sim panel.
7. Update the demo script and evaluation text for the descope (F3).

## Decisions needed

1. Switch the plant's friction to the effort-based API and regenerate all data (recommended;
   F1 makes the current data physically wrong).
2. Record O5 friction and stuck-joint data in effort mode (recommended; F2 shows it is the
   only way the fault is visible). This reverses the earlier position-mode decision.
3. Confirm the descope of attacks and rewrite the demo and O5 pass text to match.
