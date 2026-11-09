# What is true now, and what is left

Written for: whoever picks this up next. Read this first. State as of **2026-11-09**. Everything here is **simulation evidence**: ideal sensors plus white noise, authored friction
values, faults injected by the simulator. No real hardware has been involved. History is in [../changes.md](../changes.md); decisions with their evidence in [../DECISIONS.md](../DECISIONS.md).

## What has been shown

| claim | evidence | where |
|---|---|---|
| The independent model matches the simulator | gravity error 0.0 N*m on every joint, 0 to 12.5 kg; torque identity within 0.1% of the limit; 177 tests pass, 57 skip with a reason each | `tools/check_run.py`, `tests/` |
| The residual floor is small and has no knee | 0.04% of the limit at 10% speed to 0.36% at 100%, identical at 250, 500 and 1000 Hz | [RESIDUAL_FLOOR.md](RESIDUAL_FLOOR.md) |
| Payload estimation works | live dashboard, noise on: within about 3.5% from 0.5 to 8 kg (8 kg reads 7.72, 4 kg 3.89, 2 kg 1.95, 0.5 kg 0.49); a real box carried by suction within 4% after removing the 0.35 kg empty-arm bias | `out/payload`, `out/pickphys`, [DETECTION_RESULTS.md](DETECTION_RESULTS.md) |
| The fault detector works on repeating motion | friction x2 to x8 and a stuck joint on all six joints, 0 false alarms in 22 healthy runs, the right joint named every time, held-out score equal to the tuning score; delay from about 20-30 ms | [DETECTION_RESULTS.md](DETECTION_RESULTS.md) |
| The dashboard finds faults on the pick cycle | noise on, 5 kg held out: 0 of 6 healthy runs alarmed (the rule as shipped alarmed 5 of 6), 11 of 13 faults found and named correctly; the 2 misses are joint 5 | [DETECTION_RESULTS.md](DETECTION_RESULTS.md), `tools/tune_dashboard_rule.py` |
| The dashboard runs end to end | live sim, three boxes placed on every run, healthy runs clean, friction on joints 4 and 6, a stuck joint 1 and 4 and a 50 ms delay all flagged | `docs/screens/` |

## Open, in the order worth doing

1. **Faults the detector cannot see.** Backlash is not reliably found even at 0.2 to 0.5 rad (far more play than a real UR10e has), and a delay under about 20 ms is missed. These are
   measured limits of a controller that compensates the play, not bugs. Decide whether to state them as limits (the current choice) or to inject something the detector can see.
2. **The dashboard rule is tuned on one cycle.** 38 recorded runs: one style (S-curve), one speed (60% of the bar), two box masses (2 kg to tune, 5 kg held out), friction x8 only.
   Re-record with other styles, speeds and friction sizes (`tools/record_live_runs.py`) and re-run `tools/tune_dashboard_rule.py` before quoting a rate. The held-out sample is 13 faults and 6 healthy runs.
3. **Joint 5 is invisible in the pick cycle.** It moves 0.11 rad in total (the others 1 to 1.8 rad), so a friction or stuck fault there has almost nothing to act on. A limit of the cycle, not of the rule.
4. **Extra and mislabelled alerts.** A friction fault on a wrist is also called "stuck joint" (it seizes), a big fault on a heavy joint adds "command delay", and a 50 ms delay adds a friction alert on joint 3. The
   right fault is always among them; the extras are a nuisance, not a lie.
5. **The offline detector on the pick cycle.** `observers/detector.py` with `against_reference` false-alarms on healthy suction picks (a 100-sigma spike at the grasp that one reference does not line up). The dashboard
   uses its own rule, so the demo is not affected. The `pickdetect` suite (`workspace/pal_suites.py`) was built to tune it and was stopped half recorded.
6. **Heavy moving loads.** The slider stops at 8 kg, the top of the validated ladder. The arm is rated 12.5 kg (tool included); a stretched-out move above about 9 kg exceeds the joint torque limits on this cycle, so 8 to 12.5 kg
   moving has not been validated, only held.
7. **The analysis model carries the rotor inertia, the measured torque does not** ([DECISIONS.md](../DECISIONS.md) D11): a bias of up to 0.1 N*m, proportional to acceleration. A possible improvement, not a fix.
8. **Friction and armature values are authored**, not identified (1% of each joint's effort limit; `J_motor * N^2` for a 100:1 drive). They set the scale a friction fault is measured against.
9. **Stale data.** `out/_stale_pre_controller_fix/` (recorded when the wrists did not move) and `out/_archive_panda/`. Delete them, or regenerate with `tools/pal_suite.sh`.
10. **Commit.** Work after the commit `e9a58d6` (the docs reorganisation, `docs/archive/`, the tuning scripts, the dashboard rule values) is not committed.

## Traps found this session (all in the code or docs now)

- The wrists never moved until the controller carried friction and the rotor inertia ([DECISIONS.md](../DECISIONS.md) D10).
- The dashboard sliced live rows for a 7-joint arm; every live channel was read from the wrong columns until 2026-11-09.
- The live server counted boxes still on the belt, and the belt carries each one off the end: "1 of 3 placed" was a miscount.
- The dashboard's command-lag reading was not compared with the reference run, so a healthy carry read as a 43 ms delay. With sensor noise on, the old rule alarmed on every healthy run; a check with noise off hid it.
- Two programs share the container name `pal-pal-live` and port 8765 (`./pal.sh` and a headless `pal_live.py`). One at a time.
- Editing `tools/pal_suite.sh` while it runs breaks the run; bash reads a script as it executes it.
- With sensor noise on, the exact identities in the schema validator fail by design; it now allows 6 sigma from the manifest.
