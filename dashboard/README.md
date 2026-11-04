# Dashboard: Physics Attestation Layer

Plotly Dash, one page, NTT DATA branded. Started with the rest of the system by `./pal.sh` (http://127.0.0.1:8050), or alone on its
stand-in plant with `python dashboard/app.py` (`.venv-pal`). Design record: `DECISIONS.md` D9; screens in `docs/screens/`.

## Sources (the switch on the toolbar)

| Source | What it is |
|---|---|
| **Isaac Sim (live)** | The live simulator (`workspace/pal_live.py`, 250 Hz physics): rows come over HTTP, picks are sent back. Run pick, stop, repeat, random cube position, cube mass, speed bar, test fault. |
| **Stand-in plant** | A Pinocchio plant (link masses +-3%, friction x1.15, sensor noise) under a joint-impedance controller. Not Isaac Sim; the toolbar says so. |
| **Replay · empty arm / 1 kg** | Recorded Isaac Sim slow-reference logs from `data/ref_matched_*`, looped. Controls are off. |

## The page

- **Run controls** (top): pick style, Run / Stop, repeat, random cube position; cube mass and speed bars (0 to 100%, where 100% is half of each
  joint's velocity limit); "More controls" for per-joint speeds and a test fault.
- **Arm twin**: the Isaac Sim picture (or a schematic), with the key numbers: payload estimate (zeroed before the grasp, shown while a pick
  runs) and the largest residual.
- **Run summary**: task, cube position, fault, progress, set versus estimated payload, typical peak residual, alarms, whether the pick was
  recorded as or compared with a healthy reference, and the result. It stays after the pick ends.
- **Alerts**: a compact card (grouped by type); the bell opens the full list. No pop-ups.
- **Chart**: residual, torque or payload for the selected joint, with the no-alarm band; per-joint bars; a torque breakdown.
- **Panels**: telemetry (current frame), payload estimator (with tare), About.
- **Benchmark tab**: the three movement styles on a real pick (`docs/o6pick_summary.json`), plus this session's scores and the residual
  floor against speed.

## What is real

- **Real analysis code**: `observers/residual.py` (`compute_residual`, 20 Hz zero-phase filter), `observers/pose_mask.py` (the detector's pose
  rule), the payload formula of `observers/payload.py`, on a sliding 1.2 s window (live values lag by the 0.15 s filter edge).
- **Faults** (`engine.Analyzer`): the fault channel uses the applied torque; the payload uses the measured (net) torque. A pick is compared
  with a healthy run of the same task (style, cube mass, speed bar, cube position), recorded automatically the first time; choosing a fault
  with no reference first runs a healthy pick. An alarm needs |z| > 8 for 150 ms and a residual of at least 0.10 N*m and 0.5% of the joint's
  limit. Picks without a reference (random cube positions) are not judged for faults.
- **Speed**: capped at 50% of a joint's velocity limit everywhere (the engine clamps it; live picks refuse more).

## Files

`app.py` layout and callbacks, `views.py` figures and panels (pure functions), `engine.py` sources, analysis and alerts,
`assets/style.css` the look, `assets/ntt_data_logo.png` the header logo. Smoke tests: `tests/test_dashboard_engine.py`.
Screenshots: `PAL_DASH_TAB=bench` or `PAL_DASH_DRAWER=alerts` start the page on that tab or panel; headless Chrome can then capture it.

State is global in the server process (one engine), so open one browser tab. Refresh the tab after a restart.
