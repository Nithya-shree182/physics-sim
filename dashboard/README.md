# PAL dashboard (objective O7)

Plotly Dash, one page, following [docs/DASHBOARD_PLAN.md](../docs/DASHBOARD_PLAN.md) and the final design canvas.

```
conda activate pal
python dashboard/app.py          # http://127.0.0.1:8050
```

## What it runs on today

| Source (header switch) | What it is |
|---|---|
| **Stand-in plant** (default) | Our Pinocchio plant (link masses ±3%, friction ×1.15, sensor noise) under a joint-impedance controller, stepped live. The mock load, speed and test-fault controls drive it. **Not Isaac Sim**: the header says so. |
| **Replay real 0 kg / 1 kg** | Dev's real Isaac Sim slow-reference logs from `data/real/`, looped. Controls are disabled; the payload estimate and residual are the real ones (1 kg reads about 0.96 to 1.02 kg raw). |

The live Isaac Sim feed is a third source (`engine.IsaacSource`, started by `./pal.sh`; rows over HTTP from `workspace/pal_live.py`).

## What is real and what is a stand-in

- **Real analysis code**: `observers/residual.py` (`compute_residual`, 20 Hz zero-phase filter), `observers/pose_mask.py` (the detector's pose mask), the payload formula of `observers/payload.py`. They run on a sliding 1.2 s window, so live values lag by the 0.15 s filter edge.
- **Live detector** (`engine.Analyzer._alerts`): same constants as `observers/detector.py` (z > 8, 3 s calibration window, pose gate), applied per 50 ms block, plus a payload-signature fit to tell a load from a fault, and a lag estimate for the tracking channel. Calibration is per speed (the page recalibrates when you change speed). Thresholds are tuned on synthetic data: re-tune on Isaac Sim fault logs.
- **Speed is capped at 50% of a joint's velocity limit** (`DECISIONS.md` D1): the speed bars read 0-100%, and 100% on a bar is half the limit, the range where the residual is trustworthy (`RESIDUAL_FLOOR.md`). The engine clamps any speed above that, so there is no over-speed state.
- **Row rate**: the live Isaac Sim feed runs at 250 Hz physics (`DECISIONS.md` D3) and sends 250 Hz rows; the engine reads the rate from the simulator's status and times the analysis window by it.
- **Not built yet**: the in-simulator panel, the O6 trajectory comparison and the validation push in the dashboard.

## Files

`app.py` layout and callbacks, `views.py` figures and panels (pure functions), `engine.py` sources, analysis and alerts, `assets/style.css` the Physyk AI look and feel only. Smoke test: `tests/test_dashboard_engine.py`.

State is global in the server process (one engine), so open one browser tab.
