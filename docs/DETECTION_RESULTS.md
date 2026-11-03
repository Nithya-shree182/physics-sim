# Detection results with the analysis side's real code

Written for: both people, and whoever reports O5. Supersedes `docs/PREVIEW_DETECTION.md`, which was a stand-in
written before Pinocchio could be installed here.

## How this was run

Pinocchio 4.1.0 installs here with `uv` (Python 3.12, as in the analysis side's `environment.yml`); the earlier failure was
pip stalling on the Python 3.14 environment:

```bash
uv venv --python 3.12 .venv-pal
uv pip install --python .venv-pal/bin/python pin numpy scipy pandas pyarrow pyyaml pytest
git worktree add --detach /tmp/nithya origin/nithya        # their branch, read-only; nothing of theirs is edited
.venv-pal/bin/python tools/run_analysis_side.py --nithya /tmp/nithya --torque applied out/o5/*.parquet
```

Their code is imported unmodified (`observers/residual.py`, `detector.py`, `confidence.py`, `model/pinocchio_model.py`).
Their full test suite passes in this environment (the skips are tests that need their `data/real/`), and their synthetic
O5 result reproduces (8 of 8 faults, 0 false alarms). The pose-confidence gate is on throughout; it never fired.

## Result 1: their detector on our noisy runs (`out/o5` plus the C4 set, 32 faulty and 9 healthy runs)

Sensor noise: q 2e-5 rad, qd 1e-3 rad/s, torque 0.02 N*m. Four configurations: which torque the residual uses, and which
acceleration goes into the model (their default, a 20 Hz filtered difference of `qd`, or our logged analytic `qdd_cmd`).

| fault (runs) | applied, differenced qd | applied, commanded qdd | measured, differenced qd | measured, commanded qdd |
|---|---|---|---|---|
| locked joint 4 (4) | 4 of 4, 0.33 s, **tracking channel** | 4 of 4, 0.05 s, residual | 4 of 4, 0.33 s (tracking) | 4 of 4, 0.33 s (tracking) |
| delay 50 ms (4) | 4 of 4, 0.7 s (tracking) | 4 of 4, 0.4 s | 4 of 4, 0.7 s | 4 of 4, 0.4 s |
| friction x8, joints 2, 3, 4, 6 (4) | **0 of 4** | **2 of 4** (joints 4 and 6) | 0 of 4 | 0 of 4 |
| friction x4 and x2 (8) | 0 of 8 | 0 of 8 | 0 of 8 | 0 of 8 |
| friction x3, joints 3 and 6, three seeds (6) | **0 of 6** | 0 of 6 | 0 of 6 | 0 of 6 |
| delay 10 and 30 ms (2); backlash (4) | 0 of 6 | 0 of 6 | 0 of 6 | 0 of 6 |
| **all faulty runs** | **8 of 32** | **10 of 32** | 8 of 32 | 8 of 32 |
| false alarms on healthy runs, and before onset on faulty runs | 0 of 9, 0 of 32 | 0 of 9, 0 of 32 | 0 of 9, 0 of 32 | 0 of 9, 0 of 32 |

Reading it:
- Locked joint and 50 ms delay are found whatever is done. They are large effects. But with their default pipeline even a
  locked joint (about 9 N*m) is caught only by the tracking channel, at 0.33 s: the noise floor hides it from the residual.
  With the commanded acceleration the residual catches it at 0.05 s.
- **Friction is found only with the applied torque and a cleaner acceleration, and then only at x8 on two of four joints.**
  With `tau_measured` it is never found, which is the evidence for the effort-mode decision, now on their real code.
- Nothing finds friction at x4 or below, including the x3 faults the analysis side asked for.
- Zero false alarms in 9 healthy runs, so the rule is safe, not sensitive.

## Why: the residual floor is sensor noise amplified by the acceleration estimate

Healthy residual RMS per joint, `tau_applied - tau_model`, through their pipeline:

| data | joints 1-7, N*m |
|---|---|
| clean reference (no noise, no mismatch) | 0.16, 0.08, 0.09, 0.19, 0.11, 0.07, 0.12 |
| the same clean reference with our noise added offline (no mismatch) | **1.05, 2.42, 1.57, 1.75**, 0.19, 0.24, 0.13 |
| our healthy run (noise and 5% mass mismatch) | 1.08, 2.45, 1.60, 1.95, 0.19, 0.25, 0.13 |
| **their own synthetic healthy run, their default noise** (qd 2e-3, torque 0.05) | **1.90, 3.91, 2.38, 3.10**, 0.31, 0.37, 0.05 |
| their synthetic healthy run with noise off | 0.01, 0.11, 0.03, 0.12, 0.06, 0.02, 0.02 |
| our healthy run, `qdd_cmd` instead of a differenced `qd` | 0.17, 0.17, 0.13, 0.84, 0.12, 0.09, 0.14 |

So the 1-3 N*m floor on joints 1-4 is **not** the mass mismatch (rows 2 and 3 are the same) and **not** specific to our data (row
4 is theirs, and worse). It is white noise on `qd`, differentiated and multiplied by the mass matrix. Friction x2 to x4 is 0.4
to 1.1 N*m and sits inside it. Using the noise-free commanded acceleration lowers the floor ten-fold on joints 1-3 (last row).

## Result 2: the same faults without sensor noise (`out/clean`, applied torque)

| fault | their pipeline (differenced qd) | with commanded qdd |
|---|---|---|
| friction x8, joint 2 | **yes**, 0.05 s | no |
| friction x4, joint 2 | no | no |
| friction x8 and x4, joint 4 | **yes** (x8 0.05 s, x4 0.78 s) | x8 yes (0.10 s), x4 no |
| friction x2, both joints | no | no |
| locked joint 4 | yes, 0.05 s | yes, 0.05 s |
| delay 50 ms | yes, 0.61 s | yes, 0.39 s |
| healthy | no false alarm | no false alarm |

Without noise their pipeline finds friction down to x4 on the loaded joint. So the ceiling on this detector is roughly "friction
of a few N*m"; sensor noise at our level moves it up to roughly "x8 or larger". The commanded acceleration is not a cure: it
removes noise but brings in tracking error, so it helps in the noisy set and hurts in the noise-free one.

## What follows

For the analysis side (these are findings, not instructions):
1. Use `torque="applied"` on `drive_mode == "torque"` logs. Without it friction is invisible. (`detector.py`'s command line
   defaults to `measured`.)
2. The acceleration estimate is where the sensitivity goes. A smoother or model-based estimate (fitting `q`, not
   differencing a noisy `qd`) is worth more than any threshold change. The 8-sigma rule against a 3 s window is otherwise
   safe (no false alarms) but only finds large faults.
3. Their own synthetic data has the same floor, so the "8 of 8 detected" there means the synthetic faults were large.

For us: the noise level is a [CHOICE] and not identified. It decides how much of the fault set is findable. Two stances
are defensible and both are in the data: the noisy runs (stress case) and `out/clean` plus `ref_matched_*` (noise-free). We
should ask which noise level the analysis side wants as the main set.

## Limits of this result

- Our noise sigmas are representative, not measured on a real arm.
- Their detector constants were set on synthetic data and are untuned for this plant; they plan to re-tune.
- Backlash and short delays are not findable here by either channel (REVIEW F10); the rate has to include them.
- One trajectory family at 20% speed, onset at 3 s, which is also the end of their 3 s calibration window.
