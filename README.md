# Physics Attestation Layer (PAL)

A Franka Panda digital twin in NVIDIA Isaac Sim 6.0.1. The arm moves; we record what its joints actually did; an independent model
(Pinocchio, built from the URDF) predicts what the torques should have been. The gap, `r = tau_applied - tau_model`, is the signal.
Full proposal: the PDF in this folder; what the project now claims: `docs/PROPOSAL_AMENDMENTS.md`.
**Where the project stands today: `docs/HANDOFF.md`. Why each choice was made, with the evidence: `DECISIONS.md`.**

The repository holds both halves, merged on 2026-11-04 from two branches that had no common history:

| half | owns | where |
|---|---|---|
| **simulator side** | the plant (the simulated arm), telemetry, injected faults, data generation, benchmarks | `workspace/`, `tools/`, `data/`, `assets/` |
| **analysis side** | the Pinocchio model, residual, payload estimator, detector (with its pose mask), dashboard | `model/`, `observers/`, `dashboard/` |
| shared | the telemetry schema and both teams' tests | `schema/`, `tests/` |

## Run something

```bash
./pal.sh                               # the live demo: Isaac Sim (250 Hz) and the dashboard, http://127.0.0.1:8050 (see docs/HANDOFF.md)
tools/pal_suite.sh o5                  # one objective's tests, headless, then convert and check every run
tools/pal_suite.sh o3,o4 --window      # several, drawn in an Isaac Sim window (slower: rendering) so you can watch the arm
tools/pal_suite.sh pickphys --window   # a REAL pick: the fingers close on a physical cube and carry it across the table
tools/pal_suite.sh all                 # every objective's tests (about an hour headless); add --window to watch them
tools/run_sim.sh pal_selftest.py       # plant self-test; run after ANY change to the plant
tools/check_all.sh                     # every host-side check on existing outputs, no sim
tools/package_handover.sh              # one tarball of every run for the analysis side (outside git)
.venv-pal/bin/python -m pytest -q      # all tests, both teams (Pinocchio env); `unittest discover tests` runs ours without it
```

Suites (`workspace/pal_suites.py`): `reference` (O1/O2), `o3` (probe poses, kept as evidence), `o4` (known payloads), `o5` (26 fault runs), `clean`
(noise-free), `fast` (40% speed), `extra` (the analysis side's run list), `push` (validation), `o6` (styles, out-and-back), `pick` and `o6pick`
(IK pick-and-place with a modelled payload), `pickphys` (a real cube). Their definitions are plain data and unit-tested.

**"Plant"** is control-engineering for the thing being controlled: the simulated Franka arm with its masses,
friction and motors, standing in for the real robot. The analysis side's Pinocchio model is the separate
prediction it is compared with; the residual is plant versus model.

`pal_suite.sh` runs the sim (with a boot-hang watchdog), converts the runs to Parquet, then runs
`tools/check_run.py` on each one. That check does not trust the simulator's own numbers:

1. the schema validator passes;
2. gravity `g(q)` from the simulator matches `g(q)` recomputed from the URDF with plain numpy;
3. `tau_measured` matches `M qdd + C + g` within the proposal's 5% (healthy rows), reported against the torque
   limit and against the joint's own torque;
3b. in effort mode, `tau_applied - net` on healthy rows is at the configured friction (the guard against the
   friction-units bug of 2026-11-03).

All runs are **effort mode**: the arm is driven by explicit torques and `tau_applied` is logged, because
`tau_measured` is net torque and cannot show friction or a locked joint (`docs/REVIEW.md`).

## Where things are

| path | what |
|---|---|
| `model/pinocchio_model.py`, `observers/` | the analysis side: independent model, residual, payload, detector and its pose mask (`docs/ANALYSIS_CONTEXT.md`, `setup.md`) |
| `pal.sh` | **start everything**: `./pal.sh` (headless Isaac Sim at 250 Hz, so it keeps up with real time, + dashboard with the simulator's picture inside it, press Run pick), `--window`, `--dash-only`, `--stop`, `--status`, `--logs`, `--test [suite]` |
| `dashboard/` | the Plotly Dash dashboard (`dashboard/README.md`); runs on a stand-in plant or replays `data/ref_matched_*` |
| `workspace/pal_plant.py` | **the one place the plant is built** (inertials, friction, armature, gains) |
| `workspace/pal_config.py` | every parameter, each tagged with its source |
| `workspace/pal_runner.py` | effort-mode controller and scenario runner |
| `workspace/pal_faults.py`, `pal_sensors.py` | O5 injectors, seeded sensor noise |
| `workspace/pal_suites.py`, `pal_run_suite.py` | every objective's scenarios as data, and the runner (headless or windowed) |
| `workspace/pal_ik.py`, `pal_tasks.py` | inverse kinematics; pick-and-place and ellipse tasks built from it |
| `workspace/pal_objects.py`, `pal_view.py` | the physical table and cube; the drawn cube for modelled picks |
| `workspace/pal_trajectories.py` | reference motions and the three movement styles |
| `workspace/pal_kinematics.py` | independent URDF kinematics and gravity (numpy), used by the checks |
| `workspace/pal_telemetry.py` | O1 telemetry writer in `schema/telemetry_v0.yaml` format |
| `workspace/pal_*_test.py` and similar | one-off probes; each is the evidence for one claim in the docs |
| `tools/` | pipeline, run watchdog, Parquet converter, `check_run.py`, `check_payload.py`, `check_push.py`, `check_all.sh`, `package_handover.sh`, `run_analysis_side.py` (their real code on our runs), `preview_detect.py` (superseded stand-in) |
| `tests/` | unit tests (no simulator) |
| `data/` | short reference runs, tracked on purpose |
| `out/` | regenerated output, git-ignored |

## Docs index

`docs/README.md` lists every document and what it is for. `DECISIONS.md` (this folder) holds the decisions made since the first review and the evidence for each.

## Read in this order

0. `docs/HANDOFF.md`, then `DECISIONS.md`: the current state, the open items, and every decision since the first review (D1 to D9).
1. `docs/REVIEW.md`: where the project stands against the proposal, what is proven and what is not.
   Then `docs/PROPOSAL_AMENDMENTS.md` (what the project now claims), `docs/VALIDATION.md` (the push test) and
   `docs/DETECTION_RESULTS.md` (can the faults be found, with the analysis side's real code).
2. `CONTEXT.md`: state, decisions already made, traps (read before touching the sim).
3. `docs/RESIDUAL_FLOOR.md`: where the residual can and cannot be trusted (<= 50% of velocity limit).
4. `docs/O5_INJECTORS.md`: the fault sweep.
5. `docs/NOTES_FOR_NITHYA.md`: what the analysis side needs from us.
6. `docs/OPEN_PROBLEMS.md`: history of findings; later corrections are marked at the top of an item.

## Environments

- `.venv-tools` (host Python): pandas, pyarrow, pyyaml. Runs the checks.
- `.venv-pal` (Python 3.12 with Pinocchio): `uv venv --python 3.12 .venv-pal && uv pip install --python .venv-pal/bin/python pin numpy scipy pandas pyarrow pyyaml pytest`.
  Runs the analysis side's code. Both are git-ignored.

## Three traps

- `world.reset()` discards tensor-API writes. Apply them after the last reset (`pal_plant.py` does).
- One `World` per process: Isaac ignores a changed `physics_dt` on a second construction.
- The container runs as uid 1234. Host output directories must be world-writable (`run_sim.sh` does it).
- Friction goes through the torque-valued API only (`pal_plant.py`). `set_friction_coefficients` is a dimensionless
  coefficient and gave ~200x too much friction.
