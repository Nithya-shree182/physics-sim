# Data suites and the `out/` folders

Written for: whoever has to record, find or delete a run. Suites are plain data in `workspace/pal_suites.py` (unit-tested in
`tests/test_suites.py`, which pins each suite's run count). `tools/pal_suite.sh <suite>[,<suite>]` records one on the GPU, converts the runs to Parquet and
runs `tools/check_run.py` on every one. It **deletes the suite's output folder first**, so a folder only ever holds one recording.

```bash
PAL_SENSOR_NOISE=1 tools/pal_suite.sh detect        # noise on: the reference sigmas (pal_sensors.REFERENCE_SIGMA)
PAL_PHYSICS_HZ=250 tools/pal_suite.sh speedband     # another physics rate
```

Suites that need the conveyor cell and suction tool (`pickphys`, `pickfar`, `pickdetect`) run in their own process. Never edit `pal_suite.sh`
while it is running.

## Current: recorded after the controller fix of 2026-11-09

These are the data the numbers in the docs come from.

| suite | runs | folder | what it answers |
|---|---|---|---|
| `reference` | 2 | `data/` | the model against the simulator, 0 and 1 kg, no noise |
| `payload` | 30 | `out/payload` | payload estimate over 0-12.5 kg and 20/35/50% speed |
| `detect` | 96 | `out/detect` | tuning (`dtune_*`) and held-out (`dtest_*`) fault detection, every fault on every joint, noise on |
| `detectfast` | 43 | `out/detectfast` | the same faults at 40% speed |
| `detectlimits` | 22 | `out/detectlimits` | how large backlash and delay must be to be seen |
| `speedband` | 8 | `out/speedband` | residual floor against speed (copies for 1000/500/250 Hz in `out/speedband_<hz>`) |
| `pick`, `o6`, `o6pick` | 14, 26, 18 | `out/pick`, `out/o6`, `out/o6pick` | modelled-payload picks; the three movement styles (feeds `docs/o6pick_summary.json`) |
| `pickphys`, `pickfar` | 15, 9 | `out/pickphys`, `out/pickfar` | a real box carried by suction |
| (live recordings) | 38 planned | `out/livedet` | raw live rows for tuning the dashboard's own alarm rule (`tools/record_live_runs.py`) |

## Superseded: recorded before the controller fix

The wrists did not move in these runs, so anything about joints 5 and 6 in them is wrong. They are kept only because regenerating costs GPU time,
and live in `out/_stale_pre_controller_fix/`: `o3`, `o4`, `o5`, `clean`, `fast`, `extra`, `push`, and the partial `pickdetect`. The `detect` and `payload` suites replace
what they were for. Delete the folder when you no longer need it.

`out/_archive_panda/` holds the Franka Panda era's loose files and frames. Nothing reads them.

## Reading a run

Each run is `<name>_<rate>hz.parquet` plus a JSON manifest (the schema is `schema/telemetry_v1.yaml`, explained in [TELEMETRY.md](TELEMETRY.md)). The manifest
records the sensor-noise sigmas, the controller, the fault and the git commit.
