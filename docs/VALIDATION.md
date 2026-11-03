# Independent validation: push with a known force (proposal 7.7)

Written for: anyone who has to say whether the residual mathematics is right.

## The test

The arm is switched to a low-gain torque mode (computed torque, Kp = 40, Kd = 12, ten times softer than
normal) and held at a pose. A constant world-frame force is applied at the gripper (`panda_hand` origin,
the same point the Jacobian row refers to). The force is then estimated from the residual of the
applied torque, with no trained model:

    r = tau_applied - (M qdd + C + g)         F_est = -(J^T)^+ r

and compared with the logged `push_force_*`. Runs: 3 poses (home, horizontal_reach, folded) x 3 axes x
2 magnitudes (10 N, 20 N) = 18, plus 6 control runs (home and folded, 3 axes, 10 N) with the plant's
friction zeroed. Scored by `tools/check_push.py` from the written Parquet files; `pal_selftest.py` T4 is
a smaller version inside the simulator.

## What happened, in order

| attempt | what | result |
|---|---|---|
| 1 | Kp = Kd = 4, push 0.4-1.0 s, measure 0.6-0.95 s | **failed**: errors 6-29%, off-axis up to 4.3 N |
| diagnosis | time series of one run | the arm accelerated to its velocity limit (2.1-2.6 rad/s, up to 37 rad/s^2); the estimate was 9.75 N of 10 N early in the push and degraded as speed grew. That is the known above-50%-speed artifact, so the test, not the mathematics, was at fault |
| 2 | Kp = 40, Kd = 12, push 0.4-1.5 s, measure the settled window (0.6 s after onset to 0.05 s before the end); checker refuses any run above 50% of the velocity limit | see below |

The tolerance (15% of the applied magnitude, 1.5 N off-axis) was set before the first run and was not
changed. Only the experiment was.

## Result (attempt 2)

| group | runs | worst magnitude error | worst off-axis | peak speed in window |
|---|---|---|---|---|
| friction ON, realistic plant | 18 | **15.8%** (limit 15%) | 1.29 N | 49% of limit |
| friction OFF, control | 6 | **0.2%** | 0.01 N | 20% of limit |

Median error with friction on is 5.5%; 17 of 18 runs are within 10.4%. The one run over the line is
`push_folded_y_10N` at 15.8% (1.58 N of 10 N). **The strict, pre-registered criterion therefore FAILS by 0.7
points on one run.**

## Why, and what that means

The control group answers it: with friction off the force is recovered to 0.0-0.2%, so the residual
mathematics is exact. With friction on, the error is a fixed-size offset (relative error is larger at
10 N than at 20 N): static friction holds a joint back by up to its static friction torque, and the model
has none. That sets a floor on the accuracy of any residual-based force estimate on this arm.

The floor can be computed from the plant, not from the results: the largest force static friction can hide
is `|| |(J^T)^+| . tau_static ||`, using the manifest's static friction values and the Jacobian at the pose.
It is 2.5-4.1 N across these runs, a conservative worst case, and every observed error (at most 1.58 N) is
inside it. `tools/check_push.py` prints this as the AMENDED verdict (error within max(15%, dead band)),
which passes.

## How to read the two verdicts

- STRICT (15% only, as pre-registered): **FAIL**, by one run, by 0.7 points.
- AMENDED (15% or the friction dead band): PASS, but the dead band is a loose upper bound, so this verdict
  alone says little. The control group is the real evidence.

An earlier draft of the amended rule used a flat 1.5 N floor; that number was read off the data (the worst
error was 1.57 N), so it was dropped in favour of the parameter-derived dead band above.

## Re-scored with the independent model (2026-11-04): the strict rule passes

After the analysis side's branch was merged, the same 18 realistic-friction runs were scored through their residual and wrench
conversion (`tools/check_push_model.py`): Pinocchio model **with friction**, applied torque, 20 Hz filter, `W_ext = -(J^T)^+ r`.
Same window, same force columns, same pre-registered rule (15%, 1.5 N off-axis, under 50% speed).

| | worst magnitude error | worst off-axis | speed |
|---|---|---|---|
| simulator terms (no friction in them), `check_push.py` | 15.8% (one run over) | 1.29 N | 49% |
| independent model, friction included, `check_push_model.py` | **9.9%** | **1.36 N** | 49% |

All 18 runs are within 15%, so **the pre-registered criterion passes with the estimator the proposal means** (a residual against an
independent model). The near-miss above came from scoring with simulator terms, which contain no friction. The two scorings are
reported side by side because the second was chosen after seeing the first; the control group (friction off, 0.2%) shows the
mathematics was never the problem.

## Conclusion for the proposal

The residual-to-force conversion is mathematically exact. With a model that includes friction, a push of 10-20 N is recovered
to within 10% (worst case 9.9%, 18 runs, three poses, three axes). With a model that ignores friction the error grows to about 16%,
limited by static friction. The proposal's
"agreement within tolerance" should be stated with that tolerance.
