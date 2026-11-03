# Notes for nithya (analysis side), from dev (simulator side)

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

**Questions for you:**

1. Do the detector thresholds in `observers/detector.py` assume the old ~2.8% fast floor? If
   so they need re-deriving from the <= 50%-speed floor.
2. Do you want fault-free baselines recorded in the same sweep as the injected runs (our
   plan), or separately?
3. Tampered Cartesian goals produce no residual (A2): scope them out, or add a geometric
   check on your side?
4. Confirm the speed regime: O2/O4/O5 at or below 50% of velocity limit, O6 allowed to go
   faster but reported separately.

## 7. Where to look

| | |
|---|---|
| corrected floor and effort-mode table | `docs/RESIDUAL_FLOOR.md` |
| correction to the old P1 claim | `docs/OPEN_PROBLEMS.md` (P1) |
| effort-mode test | `workspace/pal_effort_test.py`, `logs/pal_effort_test_{500,1000,2000}.log` |
| decoupled controller rate test (no effect) | `workspace/pal_rate_decouple.py` |
