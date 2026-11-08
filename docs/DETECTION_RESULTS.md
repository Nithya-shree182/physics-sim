# Detection results — UR10e (2026-11-09)

Written for: whoever picks this up next. **Everything below this section is Panda-era and describes a different robot.**
The UR10e numbers are here; the method is in `tools/tune_detector.py`.

## What was done

- Data: `tools/pal_suite.sh detect,detectfast,detectlimits` with `PAL_SENSOR_NOISE=1` (the reference sigmas, 0.076 N*m on torque)
  and the 5% mass mismatch. Every fault on every joint: friction x2/x4/x8, stuck joint, backlash 0.04 and 0.12 rad, delay 10/30/50 ms.
- A split, decided before any scoring: **`dtune_*`** (fault seed 1000 + eight healthy seeds) is the only data a setting was fitted to;
  **`dtest_*`** (other seeds) and **`dfast_*`** (40% speed) were scored afterwards.
- Selection rule: no alarm on any healthy tuning run, and still none with the threshold raised 1.5x; then the most faults found.
- Grid: baseline (none / gain / friction), z 4-8, hold time 20-100 ms, per-joint floor as a fraction of torque limit, residual cutoff 10 / 20 Hz.

**Before any of this, a controller bug hid the wrists.** In every earlier run joints 5 and 6 did not move, so no fault on them could show.
Fixed in `pal_runner.py` (friction feed-forward, armature in the controller's M); see `changes.md` 2026-11-09. All numbers below are after the fix.

## Result

Chosen: baseline `friction`, z = 6, hold 20 ms, 10 Hz residual filter, floors unchanged (`observers/detector.py` defaults).

| | untuned (z 8, 50 ms, no baseline, 20 Hz) | tuned, tuning split | tuned, **held out** | tuned, 40% speed |
|---|---|---|---|---|
| faults found | 17 / 39 | 30 / 39 | **31 / 39** | 27 / 39 |
| healthy runs alarmed | 0 / 9 | 0 / 9 | 0 / 9 | 0 / 4 |
| mean delay | 0.34 s | 0.07 s | 0.07 s | 0.09 s |
| named the right joint | all | all | all | all |
| friction (18: 3 sizes x 6 joints) | 10 | 18 | **18** | 18 |
| stuck joint (6) | 6 | 6 | **6** | 6 |
| delay (3: 10/30/50 ms) | 1 | 2 | **2** | 2 |
| backlash (12) | 0 | 4 | **5** | 1 |

The held-out score is not lower than the tuning score, so the settings are not fitted to the nine tuning noise seeds.
Zero false alarms in 22 healthy runs only bounds the true rate below about 13% (95%); more healthy runs would tighten it.

## Limits (`detectlimits`, noise seed 4000, 20% speed)

- **Friction, stuck joint: solved.** Every size, every joint, wrists included, with no false alarm and under 0.1 s.
- **Delay: visible from about 20-30 ms.** 15 ms missed, 20 ms found after 0.62 s, 40 ms after 0.29 s. A 10 ms delay is below what the
  controller's own tracking noise hides.
- **Backlash: not reliably detectable.** At 0.2-0.5 rad (far more play than a real UR10e has) it is found on the forearm and wrist joints
  but missed on joint 1 at 0.2 and 0.35 rad and on joint 4 at 0.35 rad, and when found on joint 1 it is blamed on joint 6 after 0.8 s.
  The controller compensates the play, so what remains is small. Treat a backlash alarm as "something is wrong", not as a location.
- **Faster motion costs backlash, not friction**: at 40% speed friction and stuck joints are still found, backlash drops to 1 of 12.

## What this is not

- Not a result on real hardware: the sensors are ideal plus white noise, the friction values are authored (1% of each joint's effort limit),
  and the fault injectors are the simulator's.
- The thresholds hold for this task (a repeating multisine; the friction baseline needs the 3 s calibration window to cover the motion).
  A pick cycle uses the reference-run method instead (`against_reference`), which these numbers do not cover.
- The dashboard's own alarm rule (`dashboard/engine.py`: |z| > 8 for 150 ms and 0.5% of limit) is separate and has NOT been retuned.

---

# Detection results with the analysis side's real code

> **2026-11-05:** the detector's "50 consecutive steps" is now 50 ms (`observers/detector.py`, `CONSECUTIVE_S`); results at 1000 Hz are unchanged. Detection on live picks: `DECISIONS.md` D7.

Written for: both people, and whoever reports O5. Supersedes `docs/PREVIEW_DETECTION.md`.

> **Correction, 2026-11-04.** An earlier version of this page, and the notes sent to the analysis side in `e7b25a3`, said the
> detector was limited by a 1-3 N*m "noise floor" created by their acceleration estimate, and that it found friction only
> at x8 on two joints. **That was wrong.** My driver called `compute_residual` without the 20 Hz low-pass that their
> detector applies by default, so it ran their pipeline unfiltered. With their own settings the floor is small and the
> results below hold. The driver now uses their `DEFAULT_CUTOFF_HZ` unless told otherwise. The "commanded acceleration"
> option I tried to work around the supposed floor is removed: it was not needed.

## How this was run

Pinocchio 4.1.0 installs with `uv` (Python 3.12, as in the analysis side's `environment.yml`); the earlier failure was pip
stalling on the system Python 3.14. After the merge their code is in this repo:

```bash
uv venv --python 3.12 .venv-pal
uv pip install --python .venv-pal/bin/python pin numpy scipy pandas pyarrow pyyaml pytest
.venv-pal/bin/python -m pytest -q --ignore=tests/test_dashboard_engine.py       # both teams' tests
.venv-pal/bin/python observers/detector.py out/o5/*.parquet                      # their detector, torque auto
.venv-pal/bin/python tools/run_analysis_side.py --nithya . --torque applied out/o5/*.parquet
```

Their code is used as written: 20 Hz zero-phase filter, 3 s calibration window, alarm after |z| > 8 for 50 ms, pose gate on
(it never fired). The only change since the merge is that the detector now picks the **applied** torque on effort-mode logs
(`residual.fault_torque`), because `tau_measured` cannot show friction (REVIEW F2).

## Result 1: noisy runs (`out/o5` plus the C4 set: 32 faulty, 9 healthy)

Sensor noise: q 2e-5 rad, qd 1e-3 rad/s, torque 0.02 N*m.

| fault (runs) | applied torque (now the default) | `tau_measured` (what the detector used before) |
|---|---|---|
| locked joint 4 (4) | **4 of 4**, 0.05 s, residual channel | 4 of 4, 0.33 s, tracking channel only |
| delay 50 ms (4) | **4 of 4**, 0.60 s, residual | 4 of 4, 0.70 s, tracking |
| friction x8, joints 2, 3, 4, 6 (4) | **4 of 4**, 0.05-0.47 s, right joint | **0 of 4** |
| friction x4, joints 2, 3, 4, 6 (4) | **3 of 4** (joints 3, 4, 6; delays 0.12-0.46 s) | 0 of 4 |
| friction x2, joints 2, 3, 4, 6 (4) | 0 of 4 | 0 of 4 |
| friction x3, joints 3 and 6, three seeds (6) | 0 of 6 | 0 of 6 |
| delay 10 and 30 ms (2); backlash 0.02-0.12 rad (4) | 0 of 6 | 0 of 6 |
| **all faulty runs** | **15 of 32** | **8 of 32** |
| false alarms: healthy runs; before onset on faulty runs | 0 of 9; 0 of 32 | 0 of 9; 0 of 32 |

Reading it:
- **With `tau_measured` friction is never found, whatever its size.** That is the evidence for the effort-mode decision, now
  on their correctly configured pipeline. With the applied torque, friction is found from about x4 upward.
- The threshold sits between x3 and x4: x4 on joints 3, 4, 6 is found, x3 on joints 3 and 6 is not. Friction x4 on joint 2 is
  not found at all (its base friction is the smallest, so x4 is only ~0.7 N*m).
- Zero false alarms in 9 healthy runs: the rule is safe. Sensitivity is the limit, not noise (Result 2).
- A locked joint is caught on the residual in 0.05 s with the applied torque; with `tau_measured` only the slower tracking
  channel sees it.

## Result 2: the same faults without sensor noise (`out/clean`, applied torque)

| fault | detected |
|---|---|
| friction x8, joint 2 | yes, 0.06 s |
| friction x4 and x2, joint 2 | no |
| friction x8 and x4, joint 4 | yes (0.05 s, 0.07 s) |
| friction x2, joint 4 | no |
| locked joint 4 | yes, 0.05 s |
| delay 50 ms | yes, 0.60 s |
| healthy | no false alarm |

Noise changes little: the detection boundary is about the same with and without it (joint 4 x4 is found at 0.07 s noise-free and
at 0.25 s with noise). So the noise level is a weaker lever than I first said.

## The healthy residual (applied torque, joints 1-7, N*m)

| data | RMS |
|---|---|
| clean reference, 20 Hz filter | 0.16, 0.08, 0.08, 0.18, 0.11, 0.07, 0.12 |
| our healthy run (noise and 5% mass mismatch), 20 Hz filter | 0.16, 0.17, 0.13, **0.84**, 0.11, 0.08, 0.13 |
| the same healthy run **without** the filter | 1.08, 2.45, 1.60, 1.95, 0.19, 0.25, 0.13 |

The filter matters: unfiltered, the differenced `qd` gives a 1-3 N*m floor on joints 1-4 (this is what I mistakenly reported as
their pipeline's behaviour). Filtered, the floor is the clean one plus joint 4's mass-mismatch term (0.84 N*m, the plant's arm
is 5% heavier than the model).

## Result 3: a calibration-window baseline finds friction down to x3 (opt-in, with a condition)

The healthy residual is not white: the 5% mass mismatch makes a part proportional to the model torque, and Coulomb friction that the
model only approximates makes a sign(qd) part. `detector.recalibrate_baseline` fits those per joint on the calibration window only
(before any fault, so a later fault cannot leak into the fit) and removes them before the z-score. `tools/try_recalibration.py`
compares the variants on all 40 faulty runs (applied torque):

| fault | original rule | gain only | **friction baseline** |
|---|---|---|---|
| friction x8 (6 runs) | 6 of 6 | 6 of 6 | 6 of 6 |
| friction x4 (6) | 4 of 6 | 4 of 6 | **6 of 6** |
| friction x3, the C4 faults (6) | 0 of 6 | 0 of 6 | **6 of 6** |
| friction x2 (6) | 0 of 6 | 0 of 6 | 0 of 6 |
| locked joint (5) | 5 of 5 | 5 of 5 | 5 of 5 |
| delay, incl. 50 ms (7) | 5 of 7 | 5 of 7 | 5 of 7 |
| backlash (4) | 0 of 4 | 0 of 4 | 0 of 4 |
| **all faulty** | **20 of 40** | 20 of 40 | **28 of 40** |
| false alarms, any run | 0 | 0 | 0 |

No parameter is tuned on the runs (the tanh width of 0.05 rad/s is a plain choice).

**The condition, found by testing it on their data.** The fit extrapolates, so it is valid only when the 3 s calibration window covers
the motion. Our reference motion repeats about every 2 s, so it does. The analysis side's synthetic `slow_reference` does not (autocorrelation
0.4 at 2 s, -0.06 at 3 s), and there the friction baseline **false-alarms on joint 5 in five of six healthy runs** with a 3 s window and is
clean with 6 s or more. Hence it is **opt-in** (`--baseline friction`, `detect(..., baseline="friction")`), off by default, and it is
the right choice for a repeated reference or pick-and-place cycle, not for motion that does not repeat.

## Result 4: faster motion (`out/fast`, 40% of the velocity limit, applied torque)

Does moving faster expose backlash and short delays? No:

| fault | detected |
|---|---|
| friction x8 and x4, joint 4 | yes, 0.05 s and 0.06 s |
| friction x2, joint 4 | no |
| locked joint 4 | yes, 0.05 s |
| backlash 0.02 and 0.12 rad (joints 1 and 7) | **no** |
| delay 10 and 30 ms | **no** |
| healthy twin | no false alarm |

So backlash and short delays are not just a low-speed problem; the applied torque stays consistent with the dynamics and the tracking error stays small.

**A trap worth recording.** A first version of this set ran at 50% speed. The healthy run peaked at 51.7% of the limit, just past the knee, and
it raised a joint-6 alarm at 5.775 s; every faulty run, backlash and delay included, raised the same alarm at the same moment, and read as
"detected". They were the healthy run's own false alarm. The set was re-run at 40%.

## Result 5: picks (`out/pick` modelled payload, `out/pickphys` real cube)

A pick does not repeat, and it picks something up. Both break the default calibration (the first 3 s):

| calibration | healthy 1 kg pick | faults after the grasp |
|---|---|---|
| first 3 s (the default) | **false alarm** (joint 1, residual) | all found |
| first 3 s, payload torque removed (`compensate_payload`) | **still a false alarm** (joint 5) | all found |
| a healthy run of the same task, different noise seed (`against_reference`, `--reference`) | **no false alarm** | all found |

Findings, in the order they were made:
1. The healthy pick false-alarms because the model has no payload. Subtracting the estimated payload's torque did **not** remove it: the cause
   was elsewhere.
2. The cause is the same precondition as the friction baseline: the first 3 s (home to grasp, unloaded) never sees the carry or the place, and the
   tracking channel sees a loaded carry sag that the unloaded start did not. A non-repeating task needs a calibration that covers it: a healthy
   reference run of the same task, subtracted on **both** channels.
3. Two bugs in my own first version of that reference, each caught by a test: a zero-padded smoothing turned a constant 0.1 rad tracking offset into a
   false alarm in the last half second; and zero-filling the filter's NaN edge rows dragged the smoothed reference to zero near the ends.

With the reference, every fault is found on the right joint (physical cube: friction x8 on joint 6 in 0.05 s, locked joint 1 in 0.03 s, a locked joint 4
and friction x8 on joint 4 in 0.09 s, 50 ms delay in 0.11 s; modelled payload: 0.06-0.12 s, except friction and a lock on joint 4 at 3.27 s because that
joint hardly moves on this path). The reference must be a healthy run of the same task: it is a requirement, not a free improvement.

**Payload estimate on a pick** (the estimator's own tare: its reading on the empty stretch before the grasp):

| payload | 0.5 kg | 1 kg | 2 kg |
|---|---|---|---|
| modelled mass at the TCP | -1.6 to -4.0% | -2.5% | -1.0 to -1.7% |
| loaded ellipse (tare: the empty ellipse) | | -0.3% | |
| **real cube held by the fingers** | -2.4 to -4.5% | -2.1 to -3.1% | **-25 to -36%** |

The real cube at 2 kg is badly under-estimated, and **got worse when the squeeze force was doubled** (-14 to -18% at 17.5 N per finger, -25 to -36% at
33 N), so friction slip is not the explanation. The cube's mass was read back and is 2.000 kg. The per-row estimate has a standard deviation near 1 kg
and the carry window is short, but the modelled 2 kg shows none of this. Not understood; recorded as open.

## What limits detection now

1. **Sensitivity of the 8-sigma rule**: friction below roughly 1 N*m on a joint is not found (x3 and x4 with the opt-in baseline, x4 and up without). The calibration window's own
   spread (mass mismatch on joint 4, friction that changes sign) sets the band.
2. **Faults the residual cannot see by construction**: backlash and delays up to 30 ms leave the applied torque consistent
   with the dynamics, and the tracking error is too small at 20% speed (REVIEW F10).
3. The mass-mismatch term on joint 4 inflates that joint's band; a pose-dependent baseline learned during calibration
   would shrink it.

## Limits of this result

- Our noise sigmas are representative, not measured on a real arm.
- Their detector constants were set on synthetic data and are untuned for this plant.
- One trajectory family at 20% speed, fault onset at 3 s (also the end of the 3 s calibration window).
