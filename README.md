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
tools/pal_pipeline.sh pal_o5_sweep.py out/o5    # 14-scenario fault sweep -> Parquet -> checked
tools/pal_pipeline.sh pal_capture.py data       # one reference capture   -> Parquet -> checked
tools/run_sim.sh pal_effort_test.py             # any single probe (log in logs/)
.venv-tools/bin/python tools/check_run.py data/slow_reference_0kg_1000hz.parquet
```

`pal_pipeline.sh` runs the sim (with a boot-hang watchdog), converts the runs to Parquet, then
runs `tools/check_run.py` on each one. That check does not trust the simulator's own numbers:

1. the schema validator passes;
2. gravity `g(q)` from the simulator matches `g(q)` recomputed from the URDF with plain numpy;
3. `tau_measured` matches `M qdd + C + g` within the proposal's 5% per joint.

## Where things are

| path | what |
|---|---|
| `workspace/pal_plant.py` | **the one place the plant is built** (inertials, friction, armature, gains) |
| `workspace/pal_config.py` | every parameter, each tagged with its source |
| `workspace/pal_faults.py`, `pal_o5_sweep.py` | O5 fault injectors and the sweep |
| `workspace/pal_capture.py`, `pal_telemetry.py` | O1 telemetry in `schema/telemetry_v0.yaml` format |
| `workspace/pal_*_test.py` and similar | one-off probes; each is the evidence for one claim in the docs |
| `tools/` | pipeline, run watchdog, Parquet converter, `check_run.py` |
| `data/` | short reference runs, tracked on purpose |
| `out/` | regenerated output, git-ignored |

## Read in this order

1. `docs/REVIEW.md`: where the project stands against the proposal, and what is and is not
   proven. Start here.
2. `CONTEXT.md`: state, decisions already made, traps (read before touching the sim).
3. `docs/RESIDUAL_FLOOR.md`: where the residual can and cannot be trusted (<= 50% of velocity limit).
4. `docs/O5_INJECTORS.md`: the fault sweep.
5. `docs/NOTES_FOR_NITHYA.md`: what the analysis side needs from us.
6. `docs/OPEN_PROBLEMS.md`: history of findings; later corrections are marked at the top of an item.

## Three traps

- `world.reset()` discards tensor-API writes. Apply them after the last reset (`pal_plant.py` does).
- One `World` per process: Isaac ignores a changed `physics_dt` on a second construction.
- The container runs as uid 1234. Host output directories must be world-writable (`run_sim.sh` does it).
