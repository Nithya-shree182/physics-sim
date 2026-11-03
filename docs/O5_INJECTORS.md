# O5 fault injectors: what exists, how to run it, what it showed

Written for: whoever generates or reads the O5 datasets (both sides).

Scope was decided on 2026-11-03: **faults, not attackers.** The plant changes; the analysis
side's Pinocchio model never does. Data is recorded at 20% of velocity limit, inside the
<= 50% regime where the residual floor is physical (`docs/RESIDUAL_FLOOR.md`).

## What is built

| file | role |
|---|---|
| `workspace/pal_faults.py` | `FaultSpec` + `FaultInjector`. Plant-side only; no Isaac import, so it is testable on its own. |
| `workspace/pal_o5_sweep.py` | One World build, 14 scenarios, one telemetry run each, plus `summary.json`. |

Injectors, using the fault names `schema/telemetry_v0.yaml` already allows:

| `fault_type` | `fault_param` | how it is applied |
|---|---|---|
| `friction` | multiplier on that joint's Coulomb friction (base = Gaz x 1.15) | `set_friction_coefficients` at onset |
| `stuck_joint` | friction in N*m (200, far above the drive) | same call, huge value |
| `delay` | milliseconds | commanded position targets pass through a FIFO of that many physics steps |

Each run is 6 s with the fault switched on at 3 s, so `fault_active` flips mid-run and every
run carries its own healthy stretch (the analysis side's detector calibrates on the first 3 s).

```bash
tools/run_sim.sh pal_o5_sweep.py         # ~5 min; runs land in out/o5/ (git-ignored)
.venv-tools/bin/python tools/npz_to_parquet.py out/o5/*.npz
```

Note: the container creates `out/o5` as uid 1234; run `chmod -R 777 out/o5` through the
container (or `docker run --entrypoint /bin/chmod`) before converting on the host.

## Not built yet

- `backlash` (in the schema and in the analysis side's `FAULT_LABELS`)
- sensor noise and quantization, model mismatch (mass scaling): both are in `WORK_SPLIT.md`
  as Person A's "realism layers". Noise matters most: the simulation is deterministic, so
  repeated healthy runs are identical and cannot give a false-alarm rate without it.
- attacks (altered waypoint, spoofed setpoint): out of scope by decision.

## What the plant did (first sweep, 2026-11-03)

Peak tracking error `|q - q_cmd|` on the faulted joint before and after onset, rad. For
`delay` the joint shown is the one with the largest post-fault error.

| scenario | before | after | verdict |
|---|---|---|---|
| healthy | 0.0646 | 0.0647 | baseline |
| friction x2, j1 / j3 / j5 | 0.065 / 0.063 / 0.062 | 0.065 / 0.063 / 0.062 | invisible in tracking |
| friction x4, j1 / j3 / j5 | same | 0.0655 / 0.0639 / 0.0623 | barely visible |
| friction x8, j1 | 0.0646 | 0.1205 | clear |
| friction x8, j3 | 0.0631 | 0.1337 | clear |
| friction x8, j5 | 0.0623 | 0.0624 | invisible |
| stuck joint j3 | 0.0631 | 0.1052 | joint stops: post-onset travel 0.007 rad vs ~0.1 rad on the others |
| delay 10 / 25 / 50 ms | 0.0646 | 0.0679 / 0.0727 / 0.0807 | grows with delay |

Read this as: the injectors work, but **small friction changes barely disturb the arm**, because
the position drive (400 N*m/rad) absorbs them. Joint 5 (a 12 N*m wrist joint with ~0.19 N*m
base friction) shows nothing even at x8. Whether those faults are detectable is the residual
channel's question, not the tracking channel's, and belongs to the analysis side's detector.

The before/after torque RMS in `out/o5/summary.json` is confounded by the trajectory phase
(the healthy run itself moves by +0.32 N*m between the two windows), so compare against the
healthy row, not against zero.

## Next

1. Run the analysis side's detector on `out/o5/*.parquet` (needs Pinocchio on the host; not
   installed here). That is the first real end-to-end O5 number and also closes P3.
2. Add sensor noise (needed for false-alarm rates) and backlash; add mass scaling as the
   deliberate model-mismatch layer.
3. Re-sweep friction at larger multipliers on the small wrist joints if x8 stays invisible.
