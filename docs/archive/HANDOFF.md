# Handoff: where the project is (2026-11-05)

> **OUT OF DATE since 2026-11-08. Read `docs/TODO.md`, then `docs/UR10E_PORT.md` and
> `docs/TELEMETRY.md`.**
>
> This page describes the project as it was with a **Franka Panda**. The arm is now a **UR10e**,
> the Panda has been removed, the pick is done with a **suction tool** in a conveyor cell, and
> sim-to-real fidelity is no longer a goal. Everything below about the arm, the gripper, the cube,
> the speed regime and the recorded results refers to the Panda and does **not** carry over.
> What is still true: the method (plant versus independent Pinocchio model, residual as the
> signal), the architecture, and most of the traps.

Written for: whoever picks the project up next (a person or a Claude Code session). Read
`UR10E_PORT.md`, then this, then `DECISIONS.md` (repo root) for the evidence behind each choice,
then `STATUS.md` for the open problems.

## In one paragraph

A Franka Panda digital twin in Isaac Sim. The arm picks a physical cube; an independent Pinocchio model predicts the joint torques; the gap
(the residual) is read as a payload or a fault. The simulator side, the analysis side and a dashboard (branded NTT DATA, titled "Physics
Attestation Layer") are all in one repository on branch `dev`. The live demo runs end to end: `./pal.sh` starts Isaac Sim (headless, 250 Hz,
real time) and the dashboard; you press Run pick and watch the picture, the payload estimate, a run summary and any alarms.

## Run it

```bash
./pal.sh                       # Isaac Sim (headless, 250 Hz) + dashboard at http://127.0.0.1:8050  (first start: a few minutes)
./pal.sh --window              # same with an Isaac Sim window;  --status, --logs, --stop;  --dash-only = stand-in plant, no GPU
.venv-pal/bin/python -m pytest -q          # 165 tests, no simulator needed
tools/pal_suite.sh pickphys    # real-cube picks at 1000 Hz (data);  PAL_PHYSICS_HZ=250 tools/pal_suite.sh pickphys  for the live rate
tools/check_all.sh             # every host-side check on existing outputs
```

Isaac Sim hangs at boot about one start in five; `pal.sh` and `tools/run_sim.sh` kill the hung container and retry. After any restart,
**refresh the browser tab**: a tab loaded before the change asks for outputs that no longer exist and fills `logs/pal_dashboard.log`
with errors.

## What the demo does now

- **Speed.** The speed bar reads 0 to 100%; 100% is half of each joint's velocity limit (the range where the residual is trustworthy).
  The bar sets the live pick's speed (5% to 100% gives picks of 84 s down to 7 s). Default 60 (30% of the limit). (D1)
- **Cube.** Fixed spot, or "random cube position": 0.53 to 0.65 m from the base, never stretched far (D5).
- **Payload.** Estimated from the measured (net) torque while the cube is held, zeroed against the empty gripper before the grasp. Within
  about 2 to 8% for 0.5 to 2 kg at 250 Hz (D3).
- **Faults** (friction x8, locked joint, 50 ms delay) start 0.4 s after the grasp. They are found by comparing the pick with a healthy run
  of the same task, recorded automatically the first time; choosing a fault with no reference first runs a healthy pick (D7). Random cube
  positions have no reference, so no fault alarms there.
- **Run summary** card: task, cube position, fault, progress, set versus estimated payload, typical peak residual, alarms, whether the pick
  was recorded as or compared with a reference, result. It stays after the pick ends (D9).
- **Benchmark tab:** the real movement-style results (`docs/o6pick_summary.json`).
- **No confidence score.** O3 was removed. The detector keeps one pose rule (`observers/pose_mask.py`): no alarms with the arm stretched to
  about 0.78 m (D4, D6).

## What changed between 2026-11-04 and 2026-11-05 (so older pages can be read in context)

1. The two branches were merged (analysis side `nithya`, simulator side `dev`): `docs/ANALYSIS_CONTEXT.md`, `docs/STATUS.md`.
2. Real IK pick-and-place with a physical cube and gripper, three styles; live dashboard fed by `workspace/pal_live.py` over HTTP.
3. Speed capped at 50% of the velocity limit (D1). Live simulator moved to 250 Hz: 0.30 of real time became 1.0, with the same residual
   and better payload estimates (D3).
4. Random cube option (D5). Confidence system removed (D4, D6).
5. Detector's "50 rows in a row" became "50 ms" (`observers/detector.py`, `CONSECUTIVE_S`) so it is the same at any row rate.
6. Dashboard friction alert fixed for picks: faults use the applied torque, a healthy reference run is the baseline, alarms need a
   physical size, the payload fit is clipped (D7).
7. Wrist-joint residual explained: static friction at rest plus the deliberate 5% mass mismatch; nothing to fix (D8).
8. Dashboard redesigned, developer names removed, summary and compact alerts added, benchmark filled in (D9).
9. Fixes found on the way: a 3 s calibration that really lasted 3.75 s at 250 Hz and swallowed the fault; the speed bars did nothing in
   live mode; the `tau_measured_includes_friction` flag was missing for the live feed; a check that smoothed over a fixed 20 rows (80 ms at
   250 Hz).

## Open items, in the order I would take them

1. **Dead-code sweep (D2).** Not started beyond the over-speed paths. Candidates, each to be checked with a search and the tests before
   deleting: the stand-in's `reach` control (only the removed demo step used it), `workspace/pal_experiment.py` (has its own copy of the
   conditioning maths), `tools/preview_detect.py` and `docs/PREVIEW_DETECTION.md` (superseded), unused imports (`json` in
   `tools/run_pick_analysis.py`, `ET` in `tools/check_run.py`), probe scripts nothing cites, the `o3` suite (kept as evidence).
2. **Fault picks.** The payload estimate is unreliable while a fault is active (0.43 kg for a 1 kg cube in one run; the summary says so).
   One fault flags its dominant joint plus a coupled neighbour, and sometimes "stuck" when the joint barely moves (joint 4 on this path).
   One live pass showed a stray alert before the fault starts; not explained.
3. **Faults at random cube positions.** They need a baseline without a reference run (a model-based baseline, or a reference per region).
4. **Why 250 Hz fixes the real 2 kg cube** (25 to 36% low at 1000 Hz, 3 to 6% at 250 Hz, nine runs): try 500 Hz. Also untested at 250 Hz:
   delays shorter than 50 ms.
5. **In-simulator panel** (O7) is the one objective part not built. Backlash and short delays are invisible to the residual at these speeds
   (REVIEW F10); friction x2 is not found.
6. **Reproducibility.** A fresh clone cannot run the simulator: it needs files outside the repo (`../Physyk_AI-master/.isaac-cache`,
   Isaac's asset server, the NGC image). STATUS problem 9.
7. **Brand.** The header logo is the NTT DATA PNG from `~/Downloads`, recoloured white (`dashboard/assets/ntt_data_logo.png`). Replace it
   if there is an approved white version.
8. **Old docs** (`NOTES_FOR_DEV.md`, `NOTES_FOR_NITHYA.md`, `WORK_SPLIT.md`, `INTEGRATION.md`, ...) name the two authors; they are history
   and were left alone. The dashboard itself no longer names anyone.

## Traps (new ones from this stretch)

- Every `PAL_*` setting the simulator needs must be in the allow-list in `tools/run_sim.sh` or the container never sees it.
- A page load used to reset the dashboard engine; it now keeps its references and run summary (`Engine.reset_controls`).
- The dashboard's first block of a run can arrive with the previous run's id; `Engine._loop` only starts a run when the simulator says
  `running`.
- Never use a fixed row count for a time rule: rows are 1000, 250 or 200 a second depending on the source.
- A check that "turns the gate off" must really turn it off (`vertical_gain_max=inf` did; an older tool printed nan).
- Report a loss only after checking the checker: two apparent 250 Hz losses were a fixed 20-row smoothing window and a fixed 50-row rule.

## Where the evidence lives

| What | Where |
|---|---|
| Decisions and their evidence (D1 to D9) | `DECISIONS.md` |
| Real-cube picks at 1000 and 250 Hz, extra noise seeds | `out/pickphys/` (git-ignored; regenerate with `tools/pal_suite.sh pickphys`) |
| Far cube (arm stretched) and its faults | `out/pickfar/`, suite `pickfar` |
| Scoring picks | `tools/run_pick_analysis.py out/pickphys 250` |
| Screens before and after the redesign | `docs/screens/` |
| Live recordings used to design the reference detector | not kept; re-record with a pick loop and `Engine`/`IsaacSource` |
| Residual floor against speed and physics rate | `docs/RESIDUAL_FLOOR.md` |
