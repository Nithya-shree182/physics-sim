# Proposal amendments

Written for: anyone reading the proposal PDF who needs to know what this project now claims.
Decided 2026-11-03 by the project owner. The PDF is not edited; read it with this page beside it.

## 1. Attacks are out of scope. Faults are in.

Nobody tampers with goals, waypoints or setpoints in this project. What changes are the arm's own
properties (friction grows, a joint locks, a gearbox loosens, the command path slows), and the
question is how the arm copes and whether the residual catches it.

| Proposal text | Now reads |
|---|---|
| s3 Motivation, Security: "Detects tampered or spoofed commands that pass authentication" | Not claimed. The physics-versus-model argument is kept for **faults**; command-level attacks are future work. |
| s4 O5: "Fault classes (friction, looseness, stuck joint, delay) and spoofed-setpoint alerts" | "Fault classes (friction, looseness, stuck joint, delay)". No spoofed-setpoint alerts. |
| s7.5 "Inject command attacks: altered waypoints and spoofed setpoints over the ROS 2 interface" | Removed. Mechanical faults only. |
| s8 demo step 3: "A tampered waypoint is sent over ROS 2... an alert appears" | Replaced by step 3 below. |
| s9 O5 pass: "Detection rate and false-positive rate over a defined fault set, with and without the confidence gate" | Unchanged in form; the fault set is the one in section 3 below. |

One physical note that justifies the descope: a waypoint that is faithfully executed produces no
residual (the physics stays consistent), so the proposal's claim that a tampered waypoint "looks
plausible but the residual rises" does not hold for that case. Restoring attacks would need a
separate intent or geometry check. `attack_type` stays `none` in all data.

## 2. Revised demonstration scenario (replaces s8)

1. The arm carries an unknown object and the payload estimate converges on the dashboard.
2. It carries the object through a near-singular pose: the confidence score dips and the estimate
   is visibly paused rather than wrong.
3. **A joint's friction is raised mid-run.** The applied-torque residual on that joint rises above
   its calibrated band and an alert names the joint and the class "friction (stiction / bearing
   wear)".
4. A **command delay** is then injected. The residual stays quiet (the physics is consistent) while
   the tracking-error channel catches it. This is the honest limit of the residual, shown on purpose.
5. The three trajectory styles are replayed side by side with their time, torque and energy.

## 3. The O5 fault set (what "defined fault set" means)

Friction x2, x4, x8 of the Gaz et al. values on joints 2, 3, 4 and 6; a locked joint 4; backlash of
0.02, 0.06 and 0.12 rad on joint 1 and 0.12 rad on joint 7 (backlash only acts where torque reverses); command delay of 10, 30 and 50 ms; plus six
healthy runs with different sensor-noise seeds, from which the false-positive rate is measured.
Faults start at 3 s into each 6 s run. Speed is 20% of the velocity limit.

## 4. Changes that follow from what was measured

| Change | Why | Where |
|---|---|---|
| The fault residual uses the **applied** torque, in effort mode | `tau_measured` is the net joint torque and cannot show friction, backlash or a locked joint | `docs/REVIEW.md` F2 |
| Friction is specified in N*m through the torque-valued friction API | the old call took a dimensionless coefficient and gave ~200x too much friction | `docs/REVIEW.md` F1 |
| The plant carries a 5% arm-mass mismatch against the model | s7.5 and s9 require the twin's model to differ from the plant | `workspace/pal_plant.py` |
| Sensor noise on logged values, seeded per run | false-positive rates need runs that differ | `workspace/pal_sensors.py` |
| O3 confidence is the analysis side's rescaled score, not raw sigma_min / sigma_max | the proposal's ratio mixes linear and angular rows (units) | analysis side, `observers/confidence.py` |
| O2 pass is reported both as % of torque limit and as % of the joint's own torque | s9 gives no denominator | `tools/check_run.py` |
