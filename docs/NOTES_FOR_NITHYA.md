# Notes for nithya (analysis side), from dev (simulator side)

> ## LATEST (2026-11-03, end of day): read this block first, it supersedes the older sections below
>
> Everything on `dev` was rebuilt after an external-style review (`docs/REVIEW.md`). What matters to you:
>
> 1. **The old data is superseded.** The plant's friction was in the wrong units (about 200x too much on
>    joint 2) and `tau_measured` cannot show friction at all. Do not use the earlier `data/slow_reference_*`
>    or any earlier `out/o5` run. They are deleted/regenerated.
> 2. **New contract for fault detection: effort mode, applied torque.** All runs are `drive_mode = torque`.
>    The fault residual is `tau_applied - tau_model` (the controller is computed torque using the plant's own
>    M, C, g; `tau_applied` is what was commanded). `tau_measured` is net torque: use it for external loads
>    (payload, push), not for friction/backlash/stuck-joint. Your `residual.py` currently refuses
>    `tau_applied` on a position-mode log; these logs are torque mode, so it needs an effort-mode path.
> 3. **The plant carries a 5% arm-mass mismatch** (links 1-7 heavier than the URDF; manifest key
>    `plant_link_mass_scale`) and, where stated, **seeded sensor noise** (`sensor_noise` in the manifest:
>    q 2e-5 rad, qd 1e-3 rad/s, torque 0.02 N*m; representative values, not identified). Your model keeps the
>    URDF values.
> 4. **Friction is real now:** Gaz values x1.15 in N*m through the torque-valued API
>    (manifest `friction_values`, columns static, dynamic, viscous). Healthy `tau_applied - net` is 0.13-0.40 N*m
>    RMS per joint, so a healthy run has a friction offset your detector's calibration window must absorb.
> 5. **Data you can run on (all schema-valid, all pass `tools/check_run.py`):**
>    - `data/ref_matched_{0kg,1kg}_1000hz.*` (tracked): slow reference, **no mass mismatch, no noise**: calibrate
>      your model here.
>    - `out/o5/` (not tracked, ~4 MB each, 23 runs): the fault set with noise and mismatch.
>      Faults: friction x2/x4/x8, locked joint, backlash, delay; six healthy runs with different noise seeds
>      for false-alarm rates. Ask and we will hand them over; `tools/pal_pipeline.sh pal_o5_sweep.py out/o5`
>      regenerates them in ~12 min.
>    - `out/o4/` (19 runs): known payloads 0 / 0.5 / 1 / 2 / 3 kg held at three poses, plus moving runs.
>      A 0 kg run at each pose is the tare. Score your estimator with the same rule as
>      `tools/check_payload.py` (within 3% or 0.02 kg).
>    - `out/push/` (24 runs): known forces at the gripper; compare with `tools/check_push.py`.
> 6. **What the fault set cannot show:** backlash (0.02-0.12 rad) and delays up to 30 ms leave no signature in
>    either the applied-torque residual or tracking at 20% speed. They are in the set on purpose; report your
>    detection rate including them. A 50 ms delay makes the loop oscillate and is visible.
> 7. **Attacks are out of scope** (`docs/PROPOSAL_AMENDMENTS.md`). `attack_type` is always `none`. The demo and
>    O5 pass text are rewritten there.
> 8. **Validation push:** `docs/VALIDATION.md`. The residual-to-force maths is exact with friction off (0.2%);
>    with realistic friction the estimate is good to 15% or a few N (one run is 15.7%, limit 15%).
> 9. **O6** (trajectory benchmark) is built and run: `docs/REVIEW.md`, `docs/o6_comparison.svg`.
>
> What we still need from you: run `observers/` on the data above (Pinocchio does not install on our machine),
> report detection and false-positive rates with and without the confidence gate, and tell us if anything in the
> manifest or columns is missing for your code.

(Older sections follow; where they conflict with the block above, the block wins.)


Written for: the analysis side, in reply to `docs/NOTES_FOR_DEV.md` round 2 on `origin/nithya`.
Written 2026-11-03. Read section 1 first: it corrects something we told you earlier.

---

## 1. Correction: the "time discretization" finding was wrong

`docs/REPLY_TO_ANALYSIS.md` and `docs/PINOCCHIO_HANDOFF.md` told you the fast-motion residual
is time discretization, first order in `dt` (200/500/1000/2000 Hz -> 50.91/14.24/2.78/1.44%).
**Do not rely on that.** The sweep behind it rebuilt the Isaac `World` several times in one
process, and Isaac ignores a new `physics_dt` after the first construction, so the rates were
not what they claimed. Correction banners are now on both documents.

What is true (`docs/RESIDUAL_FLOOR.md`, one rate per process, identical simulated windows):

- **Below ~50% of velocity limit** the residual is physical and identical across 250-2000 Hz
  (50%: 2.05% of a limit at every rate).
- **Above ~50%** it is an artifact that *grows* with physics rate (100% speed: 7.3 / 14.9 /
  30.2 / 53.8% at 250 / 500 / 1000 / 2000 Hz).

So your instinct to doubt the "~2.8% fast floor" was right. Please discount any threshold
you derived from it. The usable statement is: **record O2/O4/O5 data at or below 50% of
velocity limit**, where the floor is under about 2% of a joint's limit.

## 2. Your R2.3 test: done, the floor does not collapse

`workspace/pal_effort_test.py`: drive off (`kp = kd = 0`, `switch_control_mode("effort")`),
explicit torques `M (qdd_des + Kp e + Kd ed) + C + g` with Kp 400, Kd 40, same trajectory and
windows as the position-mode sweep. Worst-joint RMS residual, % of torque limit:

| speed | 500 Hz | 1000 Hz | 2000 Hz | position mode, 1000 Hz |
|---|---|---|---|---|
| 10% | 0.01 | 0.00 | 0.00 | 0.43 |
| 50% | 1.38 | 1.64 | 1.82 | 2.05 |
| 75% | 8.01 | 10.02 | 11.45 | 8.79 |
| 100% | 15.93 | 19.94 | 22.72 | 30.21 |

- `tau_measured - tau_cmd` is 0.05-0.4% of a limit in effort mode: the reported torque *is*
  the torque we applied. So the position drive's reporting is not the whole story.
- The knee and the above-knee artifact **survive** (your "if it stays" branch), but the rate
  dependence is much weaker (x1.4 from 500 to 2000 Hz, against x7.4 in position mode).
- **Confound:** effort-mode tracking is poor at speed (peak error 0.50-0.59 rad at 100%), so
  the arm does not follow the reference as in position mode. Indicative, not like-for-like.
- Your question about the 27.98% timing-test figure: it came from `pal_timing_rate.py` at
  1000 Hz, `position_stiff` (400 N*m/rad), 100% trajectory. It agrees with the corrected
  sweep (27.79% at 1000 Hz) and not with the old 2.78%, which was the invalid number.
- Not yet explained. Suspects: model terms `M, C, g` evaluated at the end-of-step state while
  the torque acted over the step, and the finite-difference `qdd`.

**Consequence for recording:** because the artifact survives in effort mode, switching the
O2/O5 datasets to effort mode would not remove it. Recording at <= 50% of velocity limit in
position mode (`asset_stock` drive) is still the plan.

## 3. Round 2 items

| # | Item | Answer |
|---|---|---|
| R2.1 | One captured run | **Tracked in git on `dev`**, in `data/`: `slow_reference_0kg_1000hz.{parquet,json}` and `slow_reference_1p0kg_1000hz.{parquet,json}`, about 3.5 MB each. Fetch `dev` and read them from there. (`data/*.npz` is the container-side intermediate and is deliberately not tracked.) |
| R2.2 | `ee_pos` frozen? `t` lag? | Checked on the captured run: `ee_pos` moves (x 0.289-0.331 m, y -0.035-0.045, z 0.576-0.616), so the USD read is live. `t = (k+1)*DT` is already in `pal_capture.py`. |
| R2.3 | Effort-mode test | Section 2. |
| R2.4 | Backward-difference timing | Our writer declares it; nothing to change. |
| R2.5 | Copy `schema/` | `schema/` on dev is already identical to `nithya`'s (`git diff` is empty). |

Round 1 follow-up **2b** (only the inertia diagonal is authored; `ixy, ixz, iyz` dropped):
not done. You measured at most 0.11% of a limit, so it is queued as optional, below the O5
work.

## 4. Torque convention (P5) is not closed

`TORQUE_CONVENTION` is still `"unverified"` in `pal_config.py`, and the manifest says so. NVIDIA
states motor-side and link-side torque are not interchangeable (they differ by the gear
ratio). The static check only constrains the loaded joints (ratio 1.000).

The effort-mode run is new evidence: with a known applied torque, `tau_measured` matches it to
0.05-0.4%, which points to the reported value being the same quantity we apply. We treat that
as strong evidence, not a close. If you want the manifest to say `link_side`, tell us and we
will set it; until then please do not assume it.

## 5. Confidence score (O3 / P7)

Agreed that you own it. We have dropped "rescale the Jacobian and sweep the workspace" from
our list; `observers/confidence.py` with the 0.472 m lever arm and physical gates replaces
it. Nothing on our side is pending.

## 6. What we need from you, and what we will do next

**Contract that must hold for the residual to mean anything** (restated so it cannot drift):

- The Pinocchio model is built from the URDF with the empty arm and **no payload**. The
  gripper (hand 0.73 kg, fingers 0.1 kg each) is part of the robot.
- `model.armature = 0.0061` kg*m^2 on all seven joints. Changing one side alone adds about
  27 g of phantom payload.
- Evaluate `g(q)` at the **logged** `q`, not the commanded home pose.
- Friction is labelled `friction (stiction / bearing wear)`, one class, everywhere.

**Logs.** `data/` is tracked on `dev` (short reference runs only; your synthetic set is about 250 MB and stays out). If you need other runs, ask for the scenario and we will capture and add it.

**Next on dev, in order** (all plant-side only; your model never sees any injector):

1. O5 injectors in the plant: link-mass scaling (default 1.15), actuation delay, sensor
   noise, payload variation. Friction scaling is already in place.
2. A sweep harness that runs every injector and writes a results table.
3. O4 payload estimator hand-off: the chain is verified to 0.04% at rest across 0.1-3.0 kg.

**Our four questions, answered by reading `observers/detector.py` and `WORK_SPLIT.md` on
`nithya` (2026-11-03). Please correct anything we misread.**

1. *Do the detector thresholds assume the old ~2.8% fast floor?* **No.** Thresholds are
   per-run and relative: each channel is standardised against the run's own first
   `CALIBRATION_S = 3.0` s (z > 8, 50 consecutive steps, noise floors 0.02 N*m and 2e-4 rad).
   Nothing in it depends on the old figure, so no re-derivation is needed. What it does need:
   the first 3 s of every run must be fault-free and at the same speed as the rest. Our O5
   runs are built that way (fault at 3.0 s, 20% of velocity limit). The 0.02 N*m floor is
   the one number that could matter; it is untested against our smaller faults (section 7).
2. *Fault-free baselines in the same sweep, or separate?* Both are covered: every run carries
   its own healthy first 3 s for calibration, and the sweep also includes a fully healthy
   run. One catch: our simulation is deterministic, so repeated healthy runs are identical
   and cannot give a false-alarm *rate*. That needs sensor noise, which we have not built
   yet (it is in `WORK_SPLIT.md` as one of our realism layers).
3. *Tampered Cartesian goals?* **Out of scope, decided.** This project is arm faults, not an
   attacker. `attack_type` stays `none` in our runs. Your tracking channel is still useful
   for `delay`, which is a fault. `WORK_SPLIT.md` section 8 already lists cutting
   command-attack scenarios as the third cut, so nothing else changes.
4. *Speed regime?* **Confirmed: O2/O4/O5 at <= 50% of velocity limit** (our O5 runs use 20%).
   O6 may go faster and is reported separately. Nothing in `detector.py` assumes a speed.

Fault types: you list `backlash` in `FAULT_LABELS` and `WORK_SPLIT.md`. We have built
`friction`, `stuck_joint` and `delay` (all in `docs/O5_INJECTORS.md`); `backlash` is not
built yet.

## 7. O5 injectors are built (first sweep)

14 runs (healthy, friction x2/x4/x8 on joints 2, 4 and 6, a locked joint 4, command delay of
10/25/50 ms), 6 s each, fault at 3 s, 20% of velocity limit. Details and the plant-response
table are in `docs/O5_INJECTORS.md`. Short version: the injectors work, but friction at x2-x4
barely disturbs the arm and wrist joint 6 shows nothing even at x8, so those cases rest
entirely on your residual channel. Runs are in `out/o5/` (git-ignored, ~4 MB each); we have
not run your detector on them because Pinocchio is not installed on our host.

## 8. Correction to section 7, and two findings that change O5 (2026-11-03, later)

An external-style review (`docs/REVIEW.md`) found three things. **Please do not run your
detector on `out/o5` as first described.**

1. **The O5 data was schema-invalid** (seven manifest fields missing; delay runs had
   `fault_joint = -1`; delay parameter was in ms, not seconds). Fixed, and all 14 runs now pass
   `schema/validate.py`. Section 7 said it was ready; it was not.
2. **Our plant's friction is wrong** (`REVIEW.md` F1). We passed N*m values to a dimensionless
   PhysX coefficient; joint 2 carries about 38 N*m of friction instead of ~0.2. All friction
   data and the `data/` reference runs are on this plant. A corrected plant is the next job.
3. **`tau_measured` cannot see friction or a stuck joint** (`REVIEW.md` F2). It is the net
   joint torque, so `tau_measured - (M qdd + C + g)` is ~0.001 N*m with friction x8 or a locked
   joint. Only the *applied* torque shows it, and `tau_applied` exists only in effort mode.
   Your detector's residual channel will therefore stay quiet on real friction faults unless
   O5 data is recorded in effort mode and the fault residual is `tau_applied - tau_model`.
   Payload is unaffected (it is an external load). Your synthetic friction detection result
   does not transfer to PhysX data for this reason.

What we need from you: confirm you are happy to add an effort-mode path to `residual.py`
(`tau_applied - tau_model`), and tell us if your synthetic generator puts friction into
`tau_measured` (we think it does).

## 9. Where to look

| | |
|---|---|
| corrected floor and effort-mode table | `docs/RESIDUAL_FLOOR.md` |
| correction to the old P1 claim | `docs/OPEN_PROBLEMS.md` (P1) |
| effort-mode test | `workspace/pal_effort_test.py`, `logs/pal_effort_test_{500,1000,2000}.log` |
| review, with all findings | `docs/REVIEW.md` |
| O5 injectors and first sweep | `docs/O5_INJECTORS.md`, `workspace/pal_faults.py`, `workspace/pal_o5_sweep.py` |
| decoupled controller rate test (no effect) | `workspace/pal_rate_decouple.py` |
