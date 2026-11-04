# Notes for nithya (analysis side), from dev (simulator side)

> ## NEWEST (2026-11-04, evening): a real pick, a windowed simulator, and three changes to your `observers/`
>
> 1. **The simulator now picks up a real cube** (`docs/PICK_AND_GRIPPER.md`): IK pick-and-place in three styles, a physical table and cube, fingers
>    that close on it. `tools/pal_suite.sh pickphys --window` shows it. Data: `out/pickphys` (15 runs), `out/pick` (modelled payload, 14), `out/o6pick`
>    (18), `out/o3` (12, confidence poses). The cube's weight is NOT in the simulator's `M` or `g`; it arrives through contact, as in a real cell.
> 2. **Changes I made to your code** (all in `observers/detector.py`, `observers/payload.py`; tests in `tests/test_baseline.py`, 138+ tests pass):
>    `payload_torque_per_kg` (payload.py); `compensate_payload`, `against_reference` (healthy-run calibration on both channels, `--reference`),
>    and an edge-safe smoothing (detector.py). **Please pull before editing `observers/`.**
> 3. **Findings for you:** (a) a healthy pick false-alarms the default detector (the first 3 s never see the carry); subtracting the payload torque does
>    **not** fix it; a healthy reference run of the same task does, and every fault is then found. (b) Your payload estimator is within 1-4% on the
>    modelled payload but **25-36% low on a real 2 kg cube** (and 2-5% low at 0.5 and 1 kg); the cube's mass is verified, and a harder grip made it worse.
>    I have not found the cause. (c) O3 passes on simulator data: the gate closes at the extended-elbow singular pose only, 44% of a pass through it.
>    Details: `docs/DETECTION_RESULTS.md` Result 5, `docs/REVIEW.md`.
> 4. A bug of mine you should know about: the first windowed runs were time-warped (33x motion on rendered rows). Fixed; windowed data matches
>    headless to ~0.04 N*m.

> ## MERGED (2026-11-04): your branch is now inside `dev`; this is what changed in your files
>
> The two branches had no common history; they are merged (`git merge --allow-unrelated-histories`). Your code is where it was
> (`model/`, `observers/`, `dashboard/`, `tests/`). **Edits I made to files you own, so you are not surprised:**
>
> | file | change | why |
> |---|---|---|
> | `observers/residual.py` | new `fault_torque(run)`; the model's friction term is now used only when the compared torque contains friction (`tau_applied`, or a log whose manifest says `tau_measured_includes_friction`; default True, so your synthetic logs are unchanged) | the simulator's `tau_measured` is net torque with no friction in it; subtracting modelled friction biased it by up to 0.2 N*m. The empty-arm residual is now under 0.1% of every limit |
> | `observers/detector.py` | `--torque auto` default (applied on effort-mode logs); new opt-in `recalibrate_baseline` / `--baseline friction` | friction cannot appear in `tau_measured`; the baseline finds friction x3, valid only when the calibration window covers the motion |
> | `tests/test_real_data.py`, `tests/test_proposal_objectives.py` | read `data/ref_matched_{0kg,1kg}` instead of the untracked `data/real/` | the old logs were on a wrong-friction plant; these tests used to skip, now they run and pass |
> | `dashboard/engine.py`, `dashboard/app.py` | the replay reads `data/ref_matched_*`; the empty-arm RMS constants are recomputed from it | same |
> | `context.md` | moved to `docs/ANALYSIS_CONTEXT.md` (links updated) | `context.md` and our `CONTEXT.md` collide on Windows and macOS |
> | `docs/dev_reference/` | removed | stale copies of docs that live in `docs/` and the repo root |
> | `.gitignore`, `README.md` | ours kept; `data/synthetic/` and `data/real/` added to the ignore list; your README was a one-line title | |
>
> New tests: `tests/test_real_faults.py` (the O5 findings on our runs) and `tests/test_baseline.py`. The whole suite is 138 tests; run it with
> `.venv-pal/bin/python -m pytest -q` (environment in `README.md`). **Please pull `dev` before changing `observers/` or `dashboard/` so we do not diverge again.**
>
> New findings for you, all in `docs/DETECTION_RESULTS.md`: the baseline's precondition (above), backlash and short delays are not found even at 40% speed,
> and your payload estimator recovers all 15 of our known-mass runs to within 1.1% once tared (untared it reads about 0.25 kg high, the mass mismatch).
> Open on the dashboard: it still shows a stand-in plant or replays reference runs; it does not yet show our fault runs.

> ## LATEST (2026-11-04): read this block first, it supersedes everything below
>
> Written after reading your rounds 3, 4 and 5 and your `0088d5e`. Short version: **one decision of yours
> from round 4 has to change, one of your findings was right and is now fixed, and your run list is built.**
>
> ### 1. The decision that has to change: fault data is effort mode, and the fault signal is `tau_applied`
>
> Round 4 said "we will not ask for effort-mode datasets" and asked for position-mode fault runs. That was
> reasonable on what we had told you then. It is wrong, because of something we found afterwards
> (`docs/REVIEW.md` F2): **`tau_measured` is the net joint torque and obeys Newton's law whatever produced it,
> so it cannot show friction, backlash or a locked joint.** Measured on 14 position-mode runs:
>
> | run | `tau_measured - (M qdd + C + g)` on the faulted joint, after onset |
> |---|---|
> | healthy | 0.001 N*m |
> | friction x8 on joint 2 or 4 | 0.002 / 0.001 N*m |
> | joint 4 locked (it did not move) | 0.001 N*m; `tau_measured` stayed within 0.3 N*m of gravity |
>
> The torque we **apply** does show them (`tau_applied - net` = the friction: 1.388 N*m for 1.4 injected).
> `tau_applied` exists only in effort mode (it is zero in position mode, as you already know). So:
>
> - **All our runs are now `drive_mode = "torque"`**, a computed-torque controller using the plant's own M, C, g.
> - **For friction, stuck joint and backlash, use `compute_residual(run, model, torque="applied")`.** Your
>   `residual.py` already supports this for `drive_mode == "torque"`. **`observers/detector.py`'s command-line
>   entry (line 149) calls it without `torque=`, so it defaults to `measured` and will stay silent on friction faults
>   in our logs; check that every other caller (the dashboard engine included) passes `torque="applied"` for these logs.**
> - Keep `tau_measured` for **external loads** (payload, push): it is right for those.
> - **Your synthetic "8 of 8 detected" does not transfer** to this plant unless your synthetic generator puts
>   friction into `tau_measured` (we think it does; please check). The same goes for the dashboard's replay:
>   `data/real/` holds our OLD logs (position mode, wrong friction, see 2); please replace them.
>
> ### 2. The old logs are superseded
>
> `slow_reference_0kg/1p0kg` were taken on a plant whose friction was passed in the wrong units (about 200x too
> much on joint 2) and whose inertia was authored on the diagonal only. Do not use them or the 14-run `out/o5`.
>
> ### 3. Your armature finding (round 5 B) was right; it is closed
>
> - **Yes**, `get_mass_matrices()` excludes the 0.0061 armature; the plant's dynamics include it.
> - **Yes**, our effort-mode feedforward used that `M`. Effect: `0.0061 * qdd`, about 0.01-0.02 N*m at 20% speed,
>   absorbed by the feedback. `tau_applied - (M_full qdd + C + g)` is the friction exactly (0.004-0.007 N*m on a
>   zero-friction plant, `pal_selftest.py` T1). It is not the above-knee artifact: the old ablation (armature on
>   or off) moved the mean RMS residual by 0.02 N*m out of 2.4.
> - **Your off-diagonal lead was also right, and bigger than 0.009.** We wrote only the inertia diagonal. We now
>   check the simulator's `M` against an independent numpy composite-rigid-body calculation (`pal_selftest.py`
>   T5): with the diagonal-only plant they differed by up to **0.015 kg*m^2**; after writing the full 3x3 tensor
>   (the API takes 9 numbers in the link frame; a principal-axes quaternion is the wrong way, we tried it) the
>   difference is **0.00004 kg*m^2**, with and without payload, at three poses. `M_logged + 0.0061 I` now equals
>   an independent `M`.
>
> ### 4. Your questions (round 3 D, round 4 D, round 5)
>
> | question | answer |
> |---|---|
> | Does `tau_measured` include friction? | **No.** It is net torque (table in section 1). |
> | In effort mode, which signal is `tau_measured`? | The read-back from PhysX (`get_measured_joint_efforts`). `tau_applied` is the command (`get_applied_joint_efforts`). They are separate columns and differ by friction and external loads, so the residual is not circular. |
> | Which rate/stiffness gave 27.98%? | `pal_timing_rate.py`, 1000 Hz, 400 N*m/rad, 100% speed. It matches the corrected sweep (27.79%); the old 2.78% was the invalid figure. |
> | Joints 5-7 residual 1.4-1.8% on the empty arm? | Not re-measured. Probable cause was the diagonal-only inertia (now fixed); please re-measure on `data/ref_matched_0kg`. |
> | `speed_fraction` in the manifest? | Done: measured peak speed over the run, `speed_fraction` and `speed_fraction_per_joint` (fractions of the datasheet limit). |
> | Log the analytic `qdd`? | Already logged: `qdd_cmd_*` are analytic. |
> | Sensor noise? | Done. q 2e-5 rad, qd 1e-3 rad/s, torque 0.02 N*m, seeded per run, recorded under `sensor_noise` in the manifest. Yours were qd 2e-3 and torque 0.05; say if you want ours raised to match. |
>
> ### 5. Your run list (round 4 C and round 5 C), all built and checked
>
> Same-seed matched baselines are provided throughout: a healthy run with the same trajectory and the same noise
> realisation as each faulty run.
>
> | your ID | what | where |
> |---|---|---|
> | C2a / C2b | excitation, 50% speed, 0 and 1 kg, same noise seed | `out/extra/c2_excitation_*` |
> | C4 | friction x3 joint 3, friction x3 joint 6, locked joint 4, 50 ms delay, from 3 s of 6 s, 0 kg | `out/extra/c4_*_s1000`, healthy twin `c4_healthy_0kg_s1000` |
> | C4b | the same with 1 kg held, and the 1 kg healthy twin | `out/extra/c4b_*` |
> | C5 | one fault-free run, 60 s | `out/extra/c5_healthy_60s` |
> | C6 | payload picked up mid-run, 0 to 1 kg at 3 s (`payload_kg` per row flips; `payload_schedule` in the manifest) | `out/extra/c6_pickup_0to1kg` |
> | C9 | seeds 2001 and 2002 of C4, each with its twin | `out/extra/c4_*_s2001`, `*_s2002` |
> | C3a / C3b | payload ladder 0.5-3 kg; holds at home, horizontal reach, folded (near-singular is not included), each with a 0 kg tare | `out/o4/` |
> | beyond your list | noise-free twins of the key faults (`out/clean`, 9 runs); 26-run fault sweep (friction x2/x4/x8 on joints 2, 3, 4, 6; locked joint; backlash; delay; six healthy seeds), validation push (24 runs), O6 trajectory benchmark | `out/o5`, `out/push`, `out/o6` |
> | not built | C7 impact (no injector); C8 attacks (out of scope by decision, see 7) | |
>
> Calibration data for your model: `data/ref_matched_{0kg,1kg}_1000hz.*` (tracked in git): no mass mismatch, no
> noise. Everything else carries a 5% arm-mass mismatch (`plant_link_mass_scale`) and noise.
>
> **How to get the runs:** about 120 Parquet files, too big for git. Run `tools/package_handover.sh` on our side;
> it writes one tarball with a sha256 manifest. (It still has to be sent to you by hand.)
>
> ### 6. What the fault set can and cannot show
>
> Backlash (0.02-0.12 rad) and delays up to 30 ms leave no signature in either channel at 20% speed; they are in
> the set on purpose, so report your rate including them. A 50 ms delay makes the loop oscillate and is visible.
> **We installed Pinocchio here (uv, Python 3.12) and ran your real code on these runs: `docs/DETECTION_RESULTS.md`.** Your test
> suite passes, and your synthetic O5 result reproduces. On our runs (noise on, 5% mass mismatch) your detector, with the applied
> torque and your own 20 Hz filter, finds **15 of 32** faulty runs and gives **0 false alarms in 9** healthy ones:
>
> | | applied torque | `tau_measured` |
> |---|---|---|
> | locked joint | 4 of 4, 0.05 s, residual channel | 4 of 4, 0.33 s, tracking only |
> | 50 ms delay | 4 of 4, 0.6 s | 4 of 4, 0.7 s |
> | friction x8 (joints 2, 3, 4, 6) | **4 of 4** | **0 of 4** |
> | friction x4 | 3 of 4 (not joint 2) | 0 of 4 |
> | friction x2, and the x3 you asked for | 0 | 0 |
> | backlash, delays of 10 and 30 ms | 0 | 0 |
>
> **Correction of something we sent you earlier today:** a first version of these notes said your acceleration estimate
> creates a 1-3 N*m noise floor. That was our mistake: our driver ran your `compute_residual` without your 20 Hz cutoff. With
> your own settings the healthy floor is small (0.1-0.2 N*m, except joint 4 at 0.84 N*m from the mass mismatch). Please disregard
> the earlier claim. Your detector needs no change except the torque: **`torque="applied"` for effort-mode logs** (we added
> `residual.fault_torque` and made `detector.py` use it by default after merging the branches).
>
> Where it stops: friction below about 1 N*m on a joint is inside the calibration spread. A pose-dependent baseline learned in
> the calibration window (instead of one mean and one standard deviation) is the obvious next step; it would also shrink joint
> 4's band.
>
> ### 7. Attacks are out of scope
>
> `attack_type` is always `none`. Your plan to treat tampered goals as a separate geometric check is moot;
> `docs/PROPOSAL_AMENDMENTS.md` rewrites the demo and the O5 pass text for faults only.
>
> ### 8. What we still need from you
>
> 1. (Done in the merge: the detector uses the applied torque on effort-mode logs.) Look at sensitivity: friction below ~1 N*m
>    is not found. A noise-free set is in `out/clean`.
> 2. Run `observers/` on `data/ref_matched_*` and report the residual per joint (your D1 question).
> 3. Replace `data/real/` in the dashboard with the new reference runs.
> 4. Tell us which noise levels you want, and the shape of the live feed you need (we have not built one; our runner
>    takes payload, fault and speed as scenario parameters, so a control channel is a small step once the shape is
>    agreed).
> 5. Validation push: `docs/VALIDATION.md` and `tools/check_push.py`. The conversion is exact with friction off
>    (0.2%); with realistic friction it is good to 15% or a few N, because static friction is not in the model.

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
| O5 injectors and first sweep | `docs/O5_INJECTORS.md`, `workspace/pal_faults.py`, `workspace/pal_suites.py` |
| decoupled controller rate test (no effect) | `workspace/pal_rate_decouple.py` |
