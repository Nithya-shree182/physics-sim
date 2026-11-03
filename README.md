# Physics Attestation Layer (PAL), simulator side

A Franka Panda digital twin in NVIDIA Isaac Sim 6.0.1. The arm moves; we record what its
joints actually did; an independent model (built on the `nithya` branch with Pinocchio)
predicts what the torques *should* have been. The gap, `r = tau_measured - tau_model`, is the
signal. Full proposal: the PDF in this folder.

**This branch (`dev`) owns the plant**: the simulated arm, its telemetry, the injected faults
and the data generation. The model, detector, payload estimator, confidence score and dashboard
are on `nithya`.

## Run something

```bash
tools/pal_pipeline.sh pal_o5_sweep.py out/o5        # fault sweep: sim -> Parquet -> checks     (~12 min)
tools/pal_pipeline.sh pal_o4_payload.py out/o4      # known payloads                            (~6 min)
tools/pal_pipeline.sh pal_validation_push.py out/push  # push with a known force               (~4 min)
tools/pal_pipeline.sh pal_o6_benchmark.py out/o6    # trajectory styles                         (~4 min)
tools/pal_pipeline.sh pal_capture.py data           # reference captures for model calibration  (~2 min)
tools/run_sim.sh pal_selftest.py                    # plant self-test; run after ANY change to the plant
tools/pal_pipeline.sh pal_extra_runs.py out/extra   # the analysis side's requested run list       (~15 min)
tools/check_all.sh                                  # every host-side check on existing outputs, no sim
tools/package_handover.sh                           # one tarball of every run for the analysis side (outside git)
git worktree add --detach /tmp/nithya origin/nithya   # then, in the Pinocchio env (see below):
.venv-pal/bin/python tools/run_analysis_side.py --nithya /tmp/nithya --torque applied out/o5/*.parquet   # their real detector
python3 -m unittest discover tests                  # 30 unit tests
```

**"Plant"** is control-engineering for the thing being controlled: the simulated Franka arm with its masses,
friction and motors, standing in for the real robot. The analysis side's Pinocchio model is the separate
prediction it is compared with; the residual is plant versus model.

`pal_pipeline.sh` runs the sim (with a boot-hang watchdog), converts the runs to Parquet, then runs
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
| `workspace/pal_plant.py` | **the one place the plant is built** (inertials, friction, armature, gains) |
| `workspace/pal_config.py` | every parameter, each tagged with its source |
| `workspace/pal_runner.py` | effort-mode controller and scenario runner |
| `workspace/pal_faults.py`, `pal_sensors.py`, `pal_o5_sweep.py` | O5 injectors, seeded sensor noise, the sweep |
| `workspace/pal_trajectories.py`, `pal_o6_benchmark.py` | reference motions and the O6 benchmark |
| `workspace/pal_kinematics.py` | independent URDF kinematics and gravity (numpy), used by the checks |
| `workspace/pal_telemetry.py` | O1 telemetry writer in `schema/telemetry_v0.yaml` format |
| `workspace/pal_*_test.py` and similar | one-off probes; each is the evidence for one claim in the docs |
| `tools/` | pipeline, run watchdog, Parquet converter, `check_run.py`, `check_payload.py`, `check_push.py`, `check_all.sh`, `package_handover.sh`, `run_analysis_side.py` (their real code on our runs), `preview_detect.py` (superseded stand-in) |
| `tests/` | unit tests (no simulator) |
| `data/` | short reference runs, tracked on purpose |
| `out/` | regenerated output, git-ignored |

## Read in this order

1. `docs/REVIEW.md`: where the project stands against the proposal, what is proven and what is not. Start here.
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
