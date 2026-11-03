# Preview: can the injected faults be detected? (SUPERSEDED)

> **Superseded 2026-11-04 by `docs/DETECTION_RESULTS.md`, which uses the analysis side's real code and Pinocchio.** This stand-in
> was optimistic (it caught friction x8 on all four joints; their pipeline catches none of them with the same signal), because
> it smoothed the acceleration differently. Kept as a record and for `tools/preview_detect.py`, which still runs without Pinocchio.

Written for: both people, and anyone who wants to know whether the O5 dataset is fit for purpose before the
analysis side runs its own code.

**What this is.** `tools/preview_detect.py` re-implements the analysis side's detector *rule* (observers/detector.py
on `nithya`: z-score against the first 3 s, alarm after |z| > 8 for 50 ms, two channels, floors 0.02 N*m and
2e-4 rad) on top of an **independent numpy model** of the arm (`workspace/pal_kinematics.py`: mass matrix with full
inertia tensors and 0.0061 armature, Coriolis from the Christoffel symbols, gravity; nominal URDF, no payload, no mass
mismatch). It is **not their code and not Pinocchio** (Pinocchio does not install on this machine) and it has no
pose-confidence gate. Their real run supersedes it. It exists so we can see, before hand-over, whether the faults rise
above the mass mismatch and the sensor noise.

**The model checks out.** On the clean reference run the numpy model's residual against `tau_applied` is
`[0.25, 0.19, 0.18, 0.39, 0.26, 0.16, 0.13]` N*m per joint, equal to the plant's configured friction
(`tools/check_run.py` check 3b: `[0.25, 0.19, 0.19, 0.39, 0.27, 0.16, 0.13]`). So a model built only from the URDF
reproduces the simulator's dynamics, Coriolis included, not just gravity.

## Result on the 26-run O5 sweep (`out/o5`, applied torque)

| fault | detected | how |
|---|---|---|
| friction x8, joints 2, 3, 4, 6 | **4 of 4** | residual channel, on the right joint; delay 0.2-1.4 s |
| friction x4 and x2 (same joints) | **0 of 8** | below the threshold |
| locked joint 4 | **1 of 1** | residual, joint 4, 0.06 s |
| delay 50 ms | **1 of 1** | tracking channel, 0.68 s |
| delay 10 and 30 ms | 0 of 2 | invisible (see REVIEW F10) |
| backlash 0.02-0.12 rad | 0 of 4 | invisible (see REVIEW F10) |
| **overall** | **6 of 20 faulty runs** | |
| healthy runs, six noise seeds | **0 false alarms in 6** | |
| before-onset false alarms on the 20 faulty runs | 0 | |

## Result on the requested C4 set (`out/extra`, applied torque, three seeds)

| fault | 0 kg | 1 kg |
|---|---|---|
| locked joint 4 | 3 of 3 (residual, 0.06 s) | 1 of 1 |
| 50 ms delay | 3 of 3 (tracking, 0.68 s) | 1 of 1 (residual j4, 0.56 s) |
| friction x3, joint 3 | **0 of 3** | 0 of 1 |
| friction x3, joint 6 | **0 of 3** | 0 of 1 |
| healthy twins | 0 false alarms in 4 | |

## What changing the signal buys (same rule, same runs, `tau_measured` instead)

| run | `tau_measured` | `tau_applied` |
|---|---|---|
| friction x8 on joints 2, 3, 4, 6 | **0 of 4** | **4 of 4** |
| locked joint 4 | 1 of 1, late, via the tracking channel (0.28 s) | 1 of 1 on the residual (0.06 s) |
| 50 ms delay | 1 of 1 | 1 of 1 |

This is the evidence behind the effort-mode decision (`REVIEW.md` F2), on the data itself.

## What it says about the analysis side's constants

The rule as written (8 sigma against a 3 s calibration window) **misses friction at x4 and below, and the x3 friction
faults the analysis side asked for.** The reason is visible in the data: during calibration the healthy residual is
not quiet, because the plant carries a 5% arm-mass mismatch and Coulomb friction that changes sign, so its standard
deviation is a few tenths of a N*m, and 8 sigma sits above a 1 N*m friction step. Two things follow, both for the
analysis side:

1. Their constants were set on synthetic data and need re-tuning on these runs (they already plan this).
2. A residual that subtracts a pose- and velocity-dependent baseline learned in the calibration window (rather than a
   single mean and standard deviation) would shrink that standard deviation; friction is the part that should remain.

Treat the numbers above as the shape of the answer, not the answer.
