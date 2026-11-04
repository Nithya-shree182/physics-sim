# Decisions (proposed, to be validated)

Written for: the project team. Each one gets validated before we
change anything, and we keep it only if it makes the project better. New decisions go at the bottom.

---

## D1. Cap the speed control so the arm never leaves the trusted range

Status: applied 2026-11-04 (dashboard sliders, engine clamp, About text, live-pick cap, tests). 2026-11-05: in live Isaac Sim mode the speed bar now sets the pick's speed (it did nothing before); the bar's 5 to 100% gives picks of 84 s down to 7 s.

### The problem

On a healthy arm the model should match the simulator. It does at or below 50% of the joint velocity limit
(residual about 2% of the torque limit at 50%). Above 50% the simulator adds an error of its own that grows with its
update rate (about 9% of the limit at 75%, about 30% at full speed). The model is not at fault there, but the residual
becomes unreliable. See `docs/RESIDUAL_FLOOR.md`.

The dashboard has a speed slider from 5 to 100% of the velocity limit, so a viewer can push the arm into that range
and watch the residual go wrong. That shows a simulator artifact, not what the project is about.

### The fix

1. Keep the speed slider, but rescale it. The bar shows 0 to 100%, and 100% on the bar means 50% of the robot's real
   velocity limit. The bar's midpoint is 25% of the limit, and so on. The robot can never go faster than 50%.
2. Apply the same rescale to the seven per-joint sliders and to the "Reset speeds" button.
3. Remove the over-speed alarm and the "above band" notice, since that state can no longer be reached.
4. Add the About-panel text below to the dashboard (`dashboard/views.py`).

Where it lives in the code: `dashboard/app.py` (sliders, lines 134-146 and the speed callbacks) and
`dashboard/engine.py` (where the slider value becomes a joint speed). The conversion is one line:
real speed = slider value x 0.5.

### Our view

- 50% of the velocity limit is where the measurement is physical and no longer depends on a simulator setting. It was
  already a decided regime for O2, O4 and O5 (`CONTEXT.md`).
- A viewer still gets a full-range control, so the demo does not look restricted.
- O6 (the movement-style benchmark) needs speed. It stays a separate offline benchmark, reported on its own, and is
  not part of the live residual demo.

### About-panel text (draft)

> **Why the speed bar tops out at half speed.** The bar runs from 0 to 100%, and 100% on the bar is half of each
> joint's maximum velocity. In this range the torque the model predicts and the torque the arm applies agree to
> within about 2% of a joint's limit, and that agreement does not depend on any simulator setting. A clean residual
> means any change in it has a physical cause: a payload, extra friction or a stuck joint. Keeping the arm here keeps
> every reading on the dashboard trustworthy.

### Settled

- **Default speed is 60 on the bar**, which is 30% of the real limit (what the dashboard uses today).
- **The cap also applies in live Isaac Sim mode.** There the pick style sets the speeds, not the slider. The pick
  builder in `workspace/pal_tasks.py` takes speed limits (`vscale`, `ascale`, fractions of the joint limits). They
  must be capped so that no style can exceed 50% of the velocity limit, whatever the dashboard sends.

### To validate

- Does the held-alarm logic in `engine.py` (the `BAND` checks) become dead code? That is covered by D2.

---

## D2. Remove dead code

Status: the over-speed code D1 made unreachable is removed. The wider sweep is still to do (list first, then review).

### The problem

Code that nothing uses makes the project harder to read and change. Dead code also hides which parts matter. It is
likely to grow after D1, because the over-speed paths can no longer be reached.

### The fix

1. List what is unused: functions, callbacks, constants, branches, old probe scripts and old docs that nothing calls
   or refers to. Check each one with a search and the test run, not by guess.
2. Delete it in small steps, running the tests (`.venv-pal/bin/python -m pytest -q`) after each step.
3. Do not delete the probe scripts that are cited as evidence for a claim in the docs (`CONTEXT.md` lists them),
   unless the claim goes too. Anything superseded and already marked as such (for example `preview_detect.py`,
   `PREVIEW_DETECTION.md`) is a candidate.
4. Do it after D1 is applied, so the code made dead by D1 goes in the same sweep.

### Our view

Less code means fewer places for a mistake and a faster read for anyone new. The git history keeps the removed code.

### To validate

- Which items are really unused. The list comes first and gets reviewed before anything is deleted.

---

## D3. Run the live simulator at 250 Hz

Status: validated 2026-11-04 and made the default for the live view only (`pal.sh` sets `PAL_PHYSICS_HZ=250`). Data runs and O1 recording stay at 1000 Hz.

### The problem

The live dashboard needs the simulator to keep up with the clock. At 1000 Hz it ran at 0.30 of real time, so the
graphs moved about three times too slowly. The cost is per physics step (reading the arm and the mass matrix for the
controller), not the picture.

### The fix

Run the live server at 250 Hz. At 250 Hz it sends one row per step (250 rows a second). The dashboard reads the row rate
from the simulator and times its analysis window by it. O1 file recording stays at 1000 Hz.

| Physics rate | Picture rate | Speed vs real time |
|---|---|---|
| 1000 Hz | 30/s | 0.30 |
| 500 Hz | 30/s | 0.58 |
| 500 Hz | off | 0.72 |
| **250 Hz** | 30/s | **0.99** |

### Our view

- At 50% speed or below the residual does not depend on the physics rate (2.05% of limit at 250, 500, 1000 and 2000 Hz,
  `docs/RESIDUAL_FLOOR.md`). With D1 the arm never goes above 50%, so the rate that hurt before no longer matters.
- The old reason for keeping 1000 Hz (`docs/OPEN_PROBLEMS.md` P4: "lower rate multiplies the residual floor by five")
  came from the sweep later found wrong.

### Validation results (15 real-cube picks at 250 Hz against the same 15 at 1000 Hz)

| Check | 1000 Hz | 250 Hz |
|---|---|---|
| Cube lifted and placed (all 15 runs) | 15 of 15 | 15 of 15, placement error the same to 1 mm for healthy runs |
| Payload, 0.5 kg and 1 kg | within 4% | within 2.4% |
| Payload, real 2 kg cube | 25 to 36% low | **3 to 6% low** |
| Faults found with a healthy reference | 5 of 5, 0.03 to 0.11 s | 5 of 5, 0.03 to 0.10 s |
| Faults found with the 3 s self-calibration | 5 of 5, 0.04 to 0.17 s | 5 of 5, 0.03 to 0.16 s |
| False alarms with a healthy reference | none | none |
| Check 3b (applied minus net torque against friction) | passes | passes |

Two things first looked like losses and were not:
- **Check 3b failed on the trapezoidal style.** The check smoothed over a fixed 20 rows, which is 20 ms at 1000 Hz and
  80 ms at 250 Hz. It now smooths over 20 ms at any rate (`tools/check_run.py`). The 1000 Hz numbers did not change.
- **Detection looked 2 to 5 times slower.** The detector needs 50 rows in a row, written as "50 ms at 1 kHz". At 250 Hz that
  is 200 ms. Scored as 50 ms (`tools/run_pick_analysis.py`), the delays match 1000 Hz. The analysis side's default
  (`CONSECUTIVE = 50` in `observers/detector.py`) should become a duration; we left their file alone.

The joint reported for a lock on joint 4 changed from 4 to 6 in one case. Joints 4 and 6 are both over the threshold
with nearly equal scores at both rates (17 and 19, 17 and 17), and the report names whichever is ahead at the first
alarm row. It is a near tie, not an effect of the step.

The 2 kg real-cube underestimate (STATUS problem 11) mostly disappears at 250 Hz, and it repeats: nine runs (three
styles, three sensor-noise seeds) read 3.2 to 6.2% low, against 25 to 36% low at 1000 Hz. The seeds change only the
sensor noise (the physics is deterministic), so this shows the result is not a noise accident. Why the step size matters
is still unexplained.

### Still to validate

- Why the 2 kg cube estimate improves at 250 Hz (nine runs agree; the cause is the open question).
- The delay fault (50 ms) and short delays below 30 ms at the 4 ms step.
- How smooth the dashboard graphs are at 250 Hz in a real session.

### Integration (end-to-end check, 2026-11-04)

- `pal.sh` starts the live simulator at 250 Hz, drawing 10 frames a second. It holds 1.0 of real time while the
  dashboard's analysis runs against it (0.30 before; 0.85 with 30 frames a second).
- The dashboard reads the row rate from the simulator (`row_hz`) and times its window, calibration and alert streaks by
  it. Before this fix the "3 s" calibration lasted 3.75 s at 250 Hz and swallowed the start of a fault.
- Not caused by 250 Hz and still open: the dashboard's live detector raises no friction alert on a live pick at either
  rate (STATUS problem 15).

---

## D4. Is the confidence gate needed? (tested, nothing removed)

Status: tested 2026-11-05. The gate stays.

| Case | Gate on | Gate really off |
|---|---|---|
| Arm held still at the extended-elbow singular pose, 1 kg (`out/o3`) | paused | 0.989 kg (correct) |
| Moving through that pose, 1 kg: final estimate | 1.001 kg | 0.782 kg (22% low) |
| Moving through that pose: spread of single readings | 0.41 kg | 31 kg on the rows the gate would close |
| Real cube, 0.5 / 1 / 2 kg, hand 0.78 m from the base at the grasp (`out/pickfar`) | +0.023 / +0.025 / +0.030 kg | identical (the payload gate never closed) |

- Stretching the arm to the end of its reach (0.78 m) closes the detector's gate during the grasp (100% of the grasp and
  lift window) but not the payload gate (vertical noise gain about 3 against a limit of 10). The payload estimate stays
  accurate there, within 0.03 kg.
- The payload gate closes at a different pose (the elbow-extended singular pose), which a cube on the table cannot
  reach. There, while the arm moves, it prevents a 22% error and single readings of tens of kilograms.
- The earlier check labelled "gate off" never switched the gate off (`tools/run_confidence_check.py` showed nan). Fixed.
- Detector gate removed, tested on far picks (hand 0.78 m out, healthy reference, `out/pickfar`; 4 faults):

| | Gate on | Gate off |
|---|---|---|
| Faults found, on the right joint | 4 of 4, after 0.74 to 1.10 s | no: it alarms on joint 2 in every run, whichever joint has the fault |
| Alarm before the fault starts (false alarm) | none in the 4 fault runs | in all 4 fault runs (joint 2, about 0.4 s before the fault) |

  At the fixed demo spot the two are identical (the gate never closes). With the gate off, the detector raises a
  false alarm on joint 2 as the arm stretches for the grasp (3.52 s, the fault starts at 3.95 s), and that, not the fault, is the
  "fast detection" (0.04 to 0.30 s). With the gate on, the real fault is found on the right joint (4 or 6) after about 0.8 s. One open item: the healthy far run also false-alarms against its reference
  with the gate on (joint 4), so far picks are not clean even with it.

---

## D5. Random cube position in the live demo

Status: applied 2026-11-05.

The dashboard's task card has a "random cube position" option. Each pick (and each repeat) puts the cube at a random spot
0.53 to 0.65 m from the base (the fixed demo spot is 0.51 m), on the side away from the place point, kept only if the IK
solves (`pal_suites.random_pick_xy`). It is kept short of the arm's reach on purpose: a test checks 60 random spots over
the whole pick and neither confidence gate ever closes. The picture overlay shows where the cube is. Speed, mass and fault
options work as before, and the speed cap still holds. A first version reached out to 0.77 m, where the detector's gate
closes (see D4), and was shortened.

---

## D6. Remove the confidence system (proposed, not yet applied)

Status: applied 2026-11-05. The score, its dashboard panel, the confidence chart, the stat card, the hand ellipse, the guided-demo step and the payload gate are removed. The detector keeps a pose mask (`observers/pose_mask.py`). Alarms will be a separate system.

What the evidence says breaks with both gates off:

| Case | Payload gate off | Detector gate off |
|---|---|---|
| Fixed demo spot, random close cube (both gates stay open anyway) | nothing | nothing |
| Cube 0.78 m out (arm stretched at the grasp) | nothing (estimate within 0.03 kg) | **false alarm on joint 2 before the fault in 4 of 4 runs**; the fault itself is not found on its own joint |
| Arm held still at the extended-elbow singular pose | nothing (0.989 kg for 1 kg) | not tested |
| Healthy arm moving through the extended-elbow singular pose | **payload 22% low, single readings off by tens of kg** | nothing (0 alarms in 7300 rows, with and without) |

So removing the gates only breaks two things, and the demo reaches neither: a moving payload estimate near the elbow-extended
pose, and the detector's alarms with the arm stretched to about 0.78 m.

What removal means: the payload estimator loses its pause (`estimate_payload`, `vertical_gain_max`), the detector loses its
mask (`force_gain_max`), the dashboard loses the O3 panel and the "held, low trust" state, and O3 leaves the project's claims
(`PROPOSAL_AMENDMENTS.md`). Both the demo's fixed spot and the random cube stay inside the open range, which a test enforces.
Without the gates, the stated limit is: do not use the estimator or the detector with the arm stretched far out.

#### Decision (2026-11-05)

- **Payload gate: remove.** A payload estimate means something only while the gripper holds a cube, and the demo never takes the
  arm to the one pose where it fails. An estimate that is off while the gripper is empty is not shown or used.
- **Detector gate: keep, as a plain pose mask** (no 0-to-1 score and no O3 panel). The far-pick test shows it matters: with it the
  fault is found on the right joint and no alarm comes before the fault; without it a false alarm on joint 2 fires as the arm
  stretches (earlier notes called that fast detection; it was not). Confirmed.
- **The confidence score and its dashboard panel** are removed from the demo. O3 leaves the claims, replaced by the one rule
  the detector needs: no alarms with the arm stretched to about 0.78 m.

What "elbow extended" means: the pose `near_singular_extended` has joint 4 (the elbow) at -0.20 rad, almost straight (its limit is
-0.07), with the shoulder tilted 0.30 rad forward. The arm is almost one straight line pointing up and slightly forward. In a
straight arm, a force along the arm cannot be told apart from a torque error; here that direction is vertical, which is where
the payload is measured. A cube far out on the table gives a straight arm pointing outward and down, whose blind direction
is horizontal: it closes the detector's gate, not the payload's.

Applied: `observers/confidence.py` and `tests/test_confidence.py` are deleted; `observers/pose_mask.py` holds the one remaining rule;
`estimate_payload` no longer takes `vertical_gain_max` and `PayloadEstimate` has no `gated` field; the evidence tools
`run_confidence_check.py` and `run_far_check.py` are deleted (their results are in D4); the guided demo has four steps.
Tests: all 163 tests pass. Live check: random-cube picks at 1.0 of real time with no engine errors. After a restart, refresh the dashboard
tab: a tab loaded before the change asks for outputs that no longer exist and fills the log with errors.
