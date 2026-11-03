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

When dev's HTTP feed exists, a third source replaces the stand-in (`engine.py`, one class with `next_block()`).

## What is real and what is a stand-in

- **Real analysis code**: `observers/residual.py` (`compute_residual`, 20 Hz zero-phase filter), `observers/confidence.py` (conditioning, gates), the payload formula of `observers/payload.py`. They run on a sliding 1.2 s window, so live values lag by the 0.15 s filter edge.
- **Live detector** (`engine.Analyzer._alerts`): same constants as `observers/detector.py` (z > 8, 3 s calibration window, pose gate), applied per 50 ms block, plus a payload-signature fit to tell a load from a fault, and a lag estimate for the tracking channel. Calibration is per speed (the page recalibrates when you change speed). Thresholds are tuned on synthetic data: re-tune on Isaac Sim fault logs.
- **Above 50% of a joint's velocity limit** the stand-in imitates dev's drive artifact from `RESIDUAL_FLOOR.md`, and alarms and the load estimate are held with an over-speed notice.
- **Not built yet** (waits on dev): the live Isaac Sim stream and control channel, real fault and tamper runs, the O6 trajectory comparison, validation push, the in-simulator panel.

## Files

`app.py` layout and callbacks, `views.py` figures and panels (pure functions), `engine.py` sources, analysis and alerts, `assets/style.css` the Physyk AI look and feel only. Smoke test: `tests/test_dashboard_engine.py`.

State is global in the server process (one engine), so open one browser tab.
