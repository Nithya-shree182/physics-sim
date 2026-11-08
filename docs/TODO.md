# What is left to do

Written for: whoever picks this up next. Read this first. It supersedes the "What is NOT done"
section of [UR10E_PORT.md](UR10E_PORT.md), which describes the port itself.

State as of **2026-11-08, evening**: the arm is a UR10e, the Panda is gone from the runtime,
telemetry is at schema v1 with energy and torque-decomposition channels, `pal_live.py` drives the
conveyor cell, and `pytest` reports **161 passed, 57 skipped**. The history is in
[../changes.md](../changes.md).

---

## 1. Regenerate the data suites on the UR10e — **the one that blocks everything**

**The project has no validated detection or payload result for the arm it runs.** Every recorded
run in `out/` (1.2 GB) and `data/ref_matched_*` is Franka Panda data. That is not a presentation
problem; it means any number this project currently quotes about detection, payload accuracy or the
residual floor is a number about a different robot.

Needs a GPU session, not a code change:

```bash
tools/pal_suite.sh                     # all suites; see NEEDS_CELL — the cell suites run separately
```

What it unblocks, in order of how much it matters:

- **The 56 skipped tests.** Four whole modules whose every assertion is a measurement of the Panda
  plant: `test_baseline` (19), `test_proposal_objectives` (18), `test_real_faults` (14),
  `test_real_data` (5). They cannot be ported — they need the runs captured first.
- **The ≤50% speed band (D1).** `docs/RESIDUAL_FLOOR.md`'s table was measured on the Panda. The
  cell currently *assumes* the band transfers. The UR10e has ~4x the effort limits, different
  velocity limits, and a friction set that was authored rather than identified, so 50% could be
  conservative here or not conservative enough. Re-measure with `pal_sweep.py` and rewrite the
  table. The code side of D1 is done and has been since 2026-11-04 — it is only the evidence that
  is missing.
- **250 Hz (D3).** Chosen from Panda payload accuracy. Unverified on this arm.
- **The trajectory-style comparison.** `docs/o6pick_summary.json` is Panda data, and dashboard item
  8 below is blocked on re-running it (`tools/summarize_o6pick.py` after the `o6pick` suite).

While capturing, two things are now worth recording that were not before: the per-run `summary`
block the telemetry writer puts in every manifest (energy budget, worst headroom, closest approach
to a singularity), and a noise sweep — `PAL_SENSOR_NOISE=0, 0.5, 1, 2` over the same scenario — which
is the measurement the new sensor model exists to make and which answers "how good must the sensing
be before detection fails?".

## 2. Run the ported live server end to end

`pal_live.py` now builds the cell and runs `pal_tasks.cell_cycle()`, and the dashboard's control
path, status text and reference keying follow it. **None of that has been run against Isaac yet** —
it is ported, not verified.

```bash
./pal.sh                               # the live simulator and the dashboard together
```

What to watch for specifically:

- The three-box cycle completing from the dashboard's Run button, with `boxes_placed` reaching 3.
- The fault onset. It is now derived from the task (`engine.pick_fault_onset_s`, 3.81 s at the
  default) instead of the old hard-coded 3.02 s. A wrong onset puts the detector's baseline window
  on the far side of the fault, so it calibrates on faulty data and reports nothing — it fails
  quietly, which is why this is worth checking deliberately.
- Isaac dropping the articulation's physics view, which it does at random (twice in nine runs).
  `pal_cell_demo` has mid-cycle recovery and the launcher retries; `pal_live` does not yet have the
  equivalent of `_ensure_view()`. **Worth adding** — see item 6.

## 3. The dashboard items the project owner asked for

Item 5 is done (the random-cube checkbox and `random_pick_xy` are gone). The rest:

4. **Schematic twin side-by-side with the Isaac picture**, instead of a separate toggle.
6. **Delete both Replay buttons**; move Pick style and speed/payload under the Isaac Sim view.
7. **One run of three boxes → per-box facts → an analysis tab with an offline narrator.** The
   project owner chose offline, not an API-backed LLM. Keep the existing summary card. The telemetry
   now carries most of what a narrator would want to say — the energy budget, the worst headroom and
   the closest approach to a singularity are in every manifest's `summary`.
8. **Plain-language explanation of the three trajectory styles.** Blocked on item 1: the comparison
   is Panda data, and explaining it from those numbers would be describing a different robot.

## 4. Smaller things, each worth doing on its own

- **`model/poses.py`** still exists and is nearly empty of purpose now that `PROBE_POSES` is just
  `{"home": ...}`. Fold it into `pal_robot` or delete it.
- **`tools/` analysis scripts** (`check_payload.py`, `check_run.py`, `preview_detect.py`,
  `try_recalibration.py`, `run_pick_analysis.py`) now read the active arm's URDF instead of the
  deleted Panda one, so they import — but none has been run since. Expect a few Panda-era column
  names and tolerances inside them.
- **`docs/HANDOFF.md`, `docs/STATUS.md`, `docs/PINOCCHIO_HANDOFF.md`, `docs/NOTES_FOR_*.md`,
  `ARM_CONFIG.md`, `CONTEXT.md`, `INTEGRATION.md`** all still describe the Panda era. They carry
  out-of-date banners; nothing reads them automatically.
- **`TORQUE_CONVENTION` is still `"unverified"`** (DECISIONS.md P5), left open on purpose. The
  static gravity check passes at the identity, which is consistent with link-side but does not
  prove it. Settling it needs one deliberate measurement.
- **The friction and armature values are `[CHOICE]`**, not identified — see the provenance table in
  [UR10E_PORT.md](UR10E_PORT.md). They set the scale a friction fault is measured against, so they
  decide what the detector can see. If an absolute number ever has to mean something, identify them
  from healthy data.

## 5. Nothing is committed

Branch `dev`. The whole UR10e port and everything in the 2026-11-08 `changes.md` entry is in the
working tree. Worth committing before anything else starts.

## 6. Known and worked around, not fixed

**Isaac drops the articulation's physics view at random** — twice in nine runs, the same class of
problem `tools/run_sim.sh` already works around at boot. Symptom: the next read of a joint torque
raises an opaque `cannot reshape array of size 1 into shape (6,6)` and the run dies.
`pal_cell_demo._ensure_view()` re-initialises mid-cycle and the launcher retries. It is a
workaround. `pal_live.py` has no equivalent yet.
