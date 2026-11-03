"""PAL dashboard (objective O7), Plotly Dash.

    conda activate pal
    python dashboard/app.py            # then open http://127.0.0.1:8050

Follows docs/DASHBOARD_PLAN.md and the final design canvas. Nothing here is Isaac Sim yet: the page
runs either on a stand-in plant (our Pinocchio model, stepped live) or on a loop of dev's real logs,
and says which in the header. See engine.py for what is real and what is a stand-in.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from dash import ALL, Dash, Input, Output, State, ctx, dcc, html, no_update

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import views as V  # noqa: E402
from engine import BAND, Engine, REAL_RMS_PCT  # noqa: E402

E = Engine()
app = Dash(__name__, suppress_callback_exceptions=True, title="PAL dashboard")

DEMO = [
    dict(t="1 · Pick up an unknown object", len=16, reach=0.0, drawer="o4", chan="pay", ctl=dict(load=0.0, fault="none"), later=(3.0, dict(load=1.2)),
         d="A 1.2 kg load appears at the gripper after 3 s. The twin is never told: the residual rises on joints 2 and 4 and the estimate converges."),
    dict(t="2 · Carry it through a near-singular pose", len=20, reach=1.0, drawer="o3", chan="conf", ctl={},
         d="The elbow stretches out. Confidence dips, the gate closes and the estimate is held instead of drifting."),
    dict(t="3 · An inconsistent command", len=14, reach=0.0, drawer="alerts", chan="res", ctl=dict(fault="delay"),
         d="Stand-in for a tampered waypoint (dev's attack runs are pending): a 50 ms command delay. The residual stays quiet; the tracking error raises the alert."),
    dict(t="4 · Friction fault at joint 3", len=14, reach=0.0, drawer="alerts", chan="res", sel=2, ctl=dict(fault="friction", fault_joint=2),
         d="Friction is injected at joint 3 and identified by its torque signature: that joint stops fitting the load while the others still do."),
    dict(t="5 · Compare the trajectory styles", len=10, reach=0.0, drawer=None, tab="bench", ctl=dict(fault="none"),
         d="Waits for dev: three movement styles between the same waypoints, with time, peak torque and energy."),
]
DEF_UI = dict(tab="arm", drawer=None, chan="res", sel=3, more=False, brk=False, view="twin", demo=dict(on=False, step=0, t0=0.0, later=False), nonce=0)
PANELS = [("o1", "O1 Telemetry"), ("o3", "O3 Confidence"), ("o4", "O4 Payload"), ("about", "About")]
CHANS = [("res", "Residual"), ("tau", "Torque"), ("conf", "Confidence"), ("pay", "Payload")]
TITLES = dict(o1="O1 Telemetry · current frame", o3="O3 Pose confidence", o4="O4 Payload estimator", alerts="Alerts", about="About this view")


def slider(id_, lo, hi, step, val):
    from dash import dcc as d
    return d.Input(id=id_, type="range", min=lo, max=hi, step=step, value=val, className="rng", debounce=False)


layout_joint_speeds = [html.Div(style=dict(display="flex", flexDirection="column"), children=[
    html.Div(style=dict(display="flex", justifyContent="space-between", fontSize=12), children=[
        html.Label(f"Joint {i + 1}", htmlFor=f"spd-{i}"), html.Span(id=f"spd-txt-{i}", className="num")]),
    slider(f"spd-{i}", 5, 100, 5, 30)]) for i in range(7)]

def serve_layout():
    """Runs on every page load: the server has one engine, so a fresh page starts it afresh (stand-in plant,
    running, no fault) and the controls on screen always match what the engine is doing."""
    E.use("live")
    E.playing = True
    return html.Div(className="page", children=[
        dcc.Store(id="ui", data=DEF_UI), dcc.Store(id="demo-ctl", data={}), dcc.Store(id="dummy"), dcc.Store(id="ctl-sync"),
        dcc.Interval(id="tick", interval=500),
        html.Header(className="hdr", children=[
            html.Div(className="brand", children=[html.B("PAL"), html.Span("Franka Panda #1", className="muted", style=dict(fontSize=13))]),
            html.Div(id="pill", className="pill"),
            dcc.RadioItems(id="source", className="radio", value="live", inline=True,
                           options=[dict(label="Stand-in plant", value="live"), dict(label="Replay real 0 kg", value="0kg"), dict(label="Replay real 1 kg", value="1kg")]),
            html.Span(id="hdr-t", className="num", style=dict(fontSize=13, minWidth=96)),
            html.Button("Pause", id="play", n_clicks=0, className="btn", style=dict(minWidth=88)),
            html.Button("Demo", id="demo-btn", n_clicks=0, className="btn", style=dict(minWidth=88)),
            html.Button(id="bell", n_clicks=0, className="btn", style=dict(minWidth=76), **{"aria-label": "Alerts"}),
        ]),
        html.Section(id="demo-banner", className="demo", style=dict(display="none"), children=[
            html.Div(style=dict(flex="1 1 420px", minWidth=0, display="flex", flexDirection="column", gap=4), children=[
                html.Div(className="row", children=[html.Span("GUIDED DEMO", className="lbl", style=dict(color="#93c5fd")), html.Span(id="demo-title", style=dict(fontSize=14, fontWeight=700)),
                                                    html.Span(id="demo-remain", className="num", style=dict(fontSize=12, color="#9ca3af"))]),
                html.Div(id="demo-text", className="muted", style=dict(color="#c7cbd1"))]),
            html.Div(id="demo-dots", style=dict(display="flex", gap=6)),
            html.Div(style=dict(display="flex", gap=8), children=[html.Button("Back", id="demo-back", n_clicks=0, className="btn btn-s"),
                                                                  html.Button("Next", id="demo-next", n_clicks=0, className="btn btn-s"),
                                                                  html.Button("Stop", id="demo-stop", n_clicks=0, className="btn btn-s")])]),
        html.Div(className="nav", children=[
            html.Nav(style=dict(display="flex", gap=4, flex="1 1 360px"), children=[
                html.Button("O2 Arm & residual", id="tab-arm", n_clicks=0, className="tab"), html.Button("O6 Benchmark", id="tab-bench", n_clicks=0, className="tab")]),
            html.Div(className="row", style=dict(gap="6px"), children=[html.Span("PANELS", className="lbl")] +
                     [html.Button(lbl, id=f"p-{k}", n_clicks=0, className="btn btn-s") for k, lbl in PANELS])]),
        html.Div(className="body", children=[
            html.Main(className="main", children=[
                html.Div(id="main-arm", style=dict(display="flex", flexDirection="column", gap=20), children=[
                    html.Section(className="card", children=[
                        html.Div(className="spread", children=[html.H2("Arm twin", className="h2"),
                            html.Div(style=dict(display="flex", gap=4), children=[html.Button("Twin", id="v-twin", n_clicks=0, className="btn btn-s"),
                                                                                  html.Button("Isaac Sim stream", id="v-stream", n_clicks=0, className="btn btn-s")])]),
                        html.Div(style=dict(display="flex", flexWrap="wrap", gap=16, alignItems="stretch"), children=[
                            html.Div(style=dict(flex="3 1 420px", minWidth=0), children=[
                                dcc.Graph(id="twin-graph", config=dict(displayModeBar=False, staticPlot=True), style=dict(height=380)),
                                html.Div(id="stream", style=dict(display="none", aspectRatio="640 / 420", background="#05070a", border="1px dashed rgba(255,255,255,0.18)", borderRadius=6,
                                                                 alignItems="center", justifyContent="center", flexDirection="column", gap=8, textAlign="center"),
                                         children=[html.Div("Headless Isaac Sim, streamed over HTTP"), html.Div("[stream URL from dev]", className="muted", style=dict(fontSize=12))]),
                                html.Div(className="row", style=dict(marginTop=8, fontSize=11, color="#9ca3af", gap="6px 18px"), children=[
                                    html.Span("Link tint: share of torque limit"),
                                    html.Span(style=dict(display="flex", alignItems="center", gap=6), children=[
                                        html.Span(style=dict(width=60, height=8, borderRadius=4, background="linear-gradient(90deg,#4b5563,#fab219,#f87171)")), "0 – 100%"]),
                                    html.Span("Dashed ellipse: how freely the hand can move (flat = near singular)")])]),
                            html.Div(id="stats", style=dict(flex="1 1 200px", minWidth=0, display="flex", flexDirection="column", gap=10))])]),
                    html.Section(className="card", style=dict(padding="14px 18px"), children=[
                        html.Div(id="ctl-note"),
                        html.Div(id="ctl-wrap", children=[
                            html.Div(className="row", style=dict(gap="12px 24px"), children=[
                                html.Div(style=dict(flex="1 1 280px", minWidth=0, display="flex", flexDirection="column", gap=2), children=[
                                    html.Div(className="spread", children=[html.Label("MOCK LOAD ON THE GRIPPER", htmlFor="load", className="lbl"), html.Span(id="load-txt", className="num", style=dict(fontSize=16))]),
                                    html.Div(style=dict(display="flex", alignItems="center", gap=8), children=[
                                        html.Button("−", id="load-dn", n_clicks=0, className="btn", style=dict(width=44, padding=0, fontSize=18), **{"aria-label": "Decrease load by 0.1 kilogram"}),
                                        slider("load", 0, 3, 0.05, 0),
                                        html.Button("+", id="load-up", n_clicks=0, className="btn", style=dict(width=44, padding=0, fontSize=18), **{"aria-label": "Increase load by 0.1 kilogram"})])]),
                                html.Div(style=dict(flex="1 1 280px", minWidth=0, display="flex", flexDirection="column", gap=2), children=[
                                    html.Div(className="spread", children=[html.Label("JOINT SPEED, ALL JOINTS", htmlFor="spd-all", className="lbl"), html.Span(id="spd-txt", className="num", style=dict(fontSize=16))]),
                                    slider("spd-all", 5, 100, 5, 30)]),
                                html.Button("More controls", id="more-btn", n_clicks=0, className="btn")]),
                            html.Div(id="more", style=dict(display="none", flexDirection="column", gap=14, paddingTop=12, borderTop="1px solid rgba(255,255,255,0.12)", marginTop=12), children=[
                                html.Div(style=dict(display="grid", gridTemplateColumns="repeat(auto-fit,minmax(140px,1fr))", gap="8px 16px"), children=layout_joint_speeds),
                                html.Div(className="row", style=dict(alignItems="flex-end", gap="12px 20px"), children=[
                                    html.Div(children=[html.Div("TEST-ONLY FAULT", className="lbl"),
                                                       dcc.RadioItems(id="fault", className="radio", value="none", options=[
                                                           dict(label="None", value="none"), dict(label="Friction ×3 (stiction / bearing wear)", value="friction"),
                                                           dict(label="Stuck joint", value="stuck_joint"), dict(label="Command delay 50 ms", value="delay")])]),
                                    html.Div(children=[html.Div("ON JOINT", className="lbl"),
                                                       dcc.RadioItems(id="fault-joint", className="radio", value=2, inline=True, options=[dict(label=str(i + 1), value=i) for i in range(7)])]),
                                    html.Button("Reset speeds to 30%", id="reset-speeds", n_clicks=0, className="btn")]),
                                html.P("In the live product these are commands sent to Isaac Sim. Above 50% of a joint's velocity limit the reported torque is mostly drive artifact, "
                                       "so residual alarms and the load estimate are held.", className="muted", style=dict(margin=0, fontSize=12))])])]),
                    html.Section(className="card", children=[
                        html.Div(className="spread", children=[html.H2(id="chart-title", className="h2"),
                            html.Div(className="row", style=dict(gap=4), children=[html.Button(l, id=f"ch-{k}", n_clicks=0, className="btn btn-s") for k, l in CHANS])]),
                        html.Span(id="chart-sub", className="muted", style=dict(fontSize=12)),
                        dcc.Graph(id="main-chart", config=dict(displayModeBar=False), style=dict(height=240)),
                        html.Div(style=dict(display="flex", flexDirection="column", gap=6), children=[
                            html.Span("RESIDUAL PER JOINT, % OF TORQUE LIMIT · CLICK A JOINT TO SELECT IT", className="lbl"), html.Div(children=[
                                html.Button(id={"type": "jbar", "i": i}, n_clicks=0, className="jbar", title=f"Select joint {i + 1}",
                                            children=html.Div(id={"type": "jb", "i": i}, style=dict(display="contents"))) for i in range(7)])]),
                        html.Div(children=html.Button(id="brk-btn", n_clicks=0, className="btn btn-s")), html.Div(id="brk")])]),
                html.Div(id="main-bench", style=dict(display="none", flexDirection="column", gap=20))]),
            html.Aside(id="drawer", className="drawer", style=dict(display="none"), children=[
                html.Div(className="drawer-head", children=[html.H2(id="drawer-title", className="h2"),
                                                            html.Button("×", id="drawer-close", n_clicks=0, className="btn", style=dict(width=44, padding=0, fontSize=18), **{"aria-label": "Close panel"})]),
                html.Div(id="drawer-body", className="drawer-body")])]),
        html.Div(id="toasts", className="toasts", **{"aria-live": "polite"}),
    ])


app.layout = serve_layout


# ── controls → engine ───────────────────────────────────────────────────────────────────
# No callback both reads and writes the same slider (that loops): sliders are read here, and written
# by the small callbacks below, which read other components.
@app.callback(
    Output("ctl-sync", "data"),
    [Input("load", "value"), Input("fault", "value"), Input("fault-joint", "value")] + [Input(f"spd-{i}", "value") for i in range(7)])
def controls(load, fault, fjoint, *spd):
    if E.source.kind == "live":
        E.set_controls(load=float(load or 0), speeds=[float(v or 30) for v in spd], fault=fault or "none",
                       fault_joint=int(fjoint if fjoint is not None else 2))
    return no_update


@app.callback(Output("load", "value", allow_duplicate=True), Input("load-dn", "n_clicks"), Input("load-up", "n_clicks"), State("load", "value"), prevent_initial_call=True)
def load_buttons(_dn, _up, load):
    step = -0.1 if ctx.triggered_id == "load-dn" else 0.1
    return float(max(0.0, min(3.0, round((float(load or 0) + step) * 20) / 20)))


@app.callback([Output(f"spd-{i}", "value") for i in range(7)], Input("spd-all", "value"), prevent_initial_call=True)
def speed_all(v):
    return [float(v or 30)] * 7


@app.callback([Output("spd-all", "value", allow_duplicate=True)] + [Output(f"spd-{i}", "value", allow_duplicate=True) for i in range(7)],
              Input("reset-speeds", "n_clicks"), prevent_initial_call=True)
def reset_speeds(_):
    return [30.0] * 8


@app.callback(Output("dummy", "data"), Input("play", "n_clicks"), prevent_initial_call=True)
def play(_):
    E.playing = not E.playing
    return E.playing


@app.callback(
    [Output("load", "value", allow_duplicate=True), Output("spd-all", "value", allow_duplicate=True), Output("fault", "value", allow_duplicate=True),
     Output("fault-joint", "value", allow_duplicate=True)],
    Input("source", "value"), prevent_initial_call=True)
def source(which):
    E.use(which)
    return (E.controls.load, 30, "none", 2) if which == "live" else (E.controls.load, no_update, "none", no_update)


@app.callback(Output("dummy", "data", allow_duplicate=True), Input("tare-on", "n_clicks"), Input("tare-off", "n_clicks"), prevent_initial_call=True)
def tare(on, off):
    if ctx.triggered_id == "tare-on" and on:
        E.an.set_tare(True)
    elif ctx.triggered_id == "tare-off" and off:
        E.an.set_tare(False)
    return no_update


@app.callback(Output("dummy", "data", allow_duplicate=True), Input({"type": "dismiss", "aid": ALL}, "n_clicks"), prevent_initial_call=True)
def dismiss(clicks):
    for t in ctx.triggered:
        if t["value"]:
            aid = json.loads(t["prop_id"].rsplit(".", 1)[0])["aid"]
            if aid in E.an.alerts:
                E.an.alerts[aid]["dismissed"] = True
    return no_update


# ── UI state, including the guided demo ─────────────────────────────────────────────────
def _demo_apply(ui, k):
    st = DEMO[k]
    ui["demo"] = dict(on=True, step=k, t0=E.t_run, later=False)
    ui.update(chan=st["chan"] if "chan" in st else ui["chan"], drawer=st["drawer"], tab=st.get("tab", "arm"), view="twin")
    if "sel" in st:
        ui["sel"] = st["sel"]
    E.set_controls(reach=st["reach"])
    ui["nonce"] += 1
    return dict(nonce=ui["nonce"], **st["ctl"])


@app.callback(
    [Output("ui", "data"), Output("demo-ctl", "data")],
    [Input("tab-arm", "n_clicks"), Input("tab-bench", "n_clicks"), Input("bell", "n_clicks"), Input("drawer-close", "n_clicks"),
     Input("more-btn", "n_clicks"), Input("brk-btn", "n_clicks"), Input("v-twin", "n_clicks"), Input("v-stream", "n_clicks"),
     Input("demo-btn", "n_clicks"), Input("demo-next", "n_clicks"), Input("demo-back", "n_clicks"), Input("demo-stop", "n_clicks"), Input("tick", "n_intervals")]
    + [Input(f"p-{k}", "n_clicks") for k, _ in PANELS] + [Input(f"ch-{k}", "n_clicks") for k, _ in CHANS] + [Input({"type": "jbar", "i": ALL}, "n_clicks")],
    State("ui", "data"), prevent_initial_call=True)
def ui_state(*args):
    ui = dict(args[-1])
    ui["demo"] = dict(ui["demo"])
    # Dash merges triggers that arrive while a call is in flight (a click plus a tick), which makes
    # ctx.triggered_id None: read the list instead, and let a real click win over the tick.
    real = [t for t in ctx.triggered if t["prop_id"] != "tick.n_intervals" and t["value"]]
    if real:
        pid = real[0]["prop_id"].rsplit(".", 1)[0]
        trig = json.loads(pid) if pid.startswith("{") else pid
    elif any(t["prop_id"] == "tick.n_intervals" for t in ctx.triggered):
        trig = "tick"
    else:
        return no_update, no_update
    ctl = no_update
    if trig == "tick":
        d = ui["demo"]
        if not d["on"]:
            return no_update, no_update
        st, el = DEMO[d["step"]], E.t_run - d["t0"]
        if "later" in st and st["later"] and not d["later"] and el >= st["later"][0]:
            d["later"] = True
            ui["nonce"] += 1
            return ui, dict(nonce=ui["nonce"], **st["later"][1])
        if el > st["len"]:
            if d["step"] < len(DEMO) - 1:
                return ui, _demo_apply(ui, d["step"] + 1)
            E.set_controls(reach=0.0)
            ui["demo"] = dict(on=False, step=0, t0=0.0, later=False)
            return ui, no_update
        return no_update, no_update
    if trig == "tab-arm":
        ui["tab"] = "arm"
    elif trig == "tab-bench":
        ui["tab"] = "bench"
    elif trig == "bell":
        ui["drawer"] = None if ui["drawer"] == "alerts" else "alerts"
    elif trig == "drawer-close":
        ui["drawer"] = None
    elif trig == "more-btn":
        ui["more"] = not ui["more"]
    elif trig == "brk-btn":
        ui["brk"] = not ui["brk"]
    elif trig in ("v-twin", "v-stream"):
        ui["view"] = "twin" if trig == "v-twin" else "stream"
    elif isinstance(trig, str) and trig.startswith("p-"):
        k = trig[2:]
        ui["drawer"] = None if ui["drawer"] == k else k
    elif isinstance(trig, str) and trig.startswith("ch-"):
        ui["chan"] = trig[3:]
    elif isinstance(trig, dict) and trig.get("type") == "jbar":
        ui["sel"] = trig["i"]
    elif trig == "demo-btn":
        if ui["demo"]["on"]:
            E.set_controls(reach=0.0)
            ui["demo"] = dict(on=False, step=0, t0=0.0, later=False)
        else:
            ctl = _start_demo(ui)
    elif trig == "demo-next":
        ctl = _advance(ui, min(len(DEMO) - 1, ui["demo"]["step"] + 1))
    elif trig == "demo-back":
        ctl = _advance(ui, max(0, ui["demo"]["step"] - 1))
    elif trig == "demo-stop":
        E.set_controls(reach=0.0)
        ui["demo"] = dict(on=False, step=0, t0=0.0, later=False)
    return ui, ctl


def _advance(ui, k):
    return _demo_apply(ui, k)


def _start_demo(ui):
    return dict(_demo_apply(ui, 0), speeds=30)


@app.callback(
    [Output("load", "value", allow_duplicate=True), Output("spd-all", "value", allow_duplicate=True),
     Output("fault", "value", allow_duplicate=True), Output("fault-joint", "value", allow_duplicate=True)],
    Input("demo-ctl", "data"), prevent_initial_call=True)
def demo_to_controls(d):
    if not d:
        return no_update, no_update, no_update, no_update
    return d.get("load", no_update), d.get("speeds", no_update), d.get("fault", no_update), d.get("fault_joint", no_update)


# ── render ──────────────────────────────────────────────────────────────────────────────
_REAL = None
_toast_sig = None


def _real_pct():
    global _REAL
    if _REAL is None:
        try:
            from residual import compute_residual, load_run
            p = Path(__file__).resolve().parents[1] / "data" / "ref_matched_0kg_1000hz.parquet"
            res = compute_residual(load_run(p), E.twin if E.which == "live" else None)
            _REAL = (100 * np.sqrt(np.nanmean(res.r ** 2, axis=0)) / V.TL).tolist()
        except Exception:
            _REAL = list(REAL_RMS_PCT)
    return _REAL


OUTS = ["hdr-t", "bell", "bell:cls", "pill", "pill:style", "play", "demo-banner:style", "demo-title", "demo-remain", "demo-text", "demo-dots",
        "tab-arm:cls", "tab-bench:cls", "main-arm:style", "main-bench:style", "twin-graph:style", "stream:style", "v-twin:cls", "v-stream:cls",
        "stats", "ctl-wrap:cls", "ctl-note", "more:style", "more-btn", "load-txt", "spd-txt", "chart-title", "chart-sub", "main-chart:fig",
        "brk-btn", "brk", "main-bench", "drawer:style", "drawer-title", "drawer-body", "toasts"] + [f"p-{k}:cls" for k, _ in PANELS] + [f"ch-{k}:cls" for k, _ in CHANS] + \
       [f"spd-txt-{i}" for i in range(7)] + ["twin-graph:fig"]
_PROP = {"cls": "className", "style": "style", "fig": "figure"}


def _out(name):
    i, _, p = name.partition(":")
    return Output(i, _PROP.get(p, "children"))


@app.callback([_out(o) for o in OUTS] + [Output({"type": "jb", "i": ALL}, "children"), Output({"type": "jbar", "i": ALL}, "className")],
              Input("tick", "n_intervals"), Input("ui", "data"))
def render(_n, ui):
    global _toast_sig
    snap = E.an.snapshot
    ctl = E.controls
    r = {}
    if not snap or "geom" not in snap:
        raise_ = [no_update] * (len(OUTS) + 2)
        raise_[OUTS.index("hdr-t")] = "warming up…"
        return raise_
    an = E.an
    cal = an.cal
    live = E.source.kind == "live"
    active = [a for a in snap["alerts"].values() if a["sev"] > 0]
    sel = ui["sel"]
    dm = ui["demo"]
    on = lambda cond, base: base + (" btn-on" if cond else "")   # noqa: E731

    r["hdr-t"] = f"t = {snap['t']:.2f} s"
    r["bell"] = f"Alerts {len(active)}"
    r["bell:cls"] = on(ui["drawer"] == "alerts", "btn")
    if live:
        r["pill"] = [html.Span(className="dot", style=dict(background="#fab219")), "STAND-IN · Pinocchio plant, not Isaac Sim"]
    else:
        r["pill"] = [html.Span(className="dot", style=dict(background="#34d399")), f"REPLAY · dev's Isaac Sim log ({E.source.name.split('_')[2]}), looped"]
    r["pill:style"] = dict(color=("#fab219" if active else "#e5e7eb"))
    r["play"] = "Pause" if E.playing else "Resume"
    st = DEMO[dm["step"]]
    r["demo-banner:style"] = dict(display="flex") if dm["on"] else dict(display="none")
    r["demo-title"], r["demo-text"] = st["t"], st["d"]
    r["demo-remain"] = f"{max(0, round(st['len'] - (E.t_run - dm['t0'])))} s left"
    r["demo-dots"] = [html.Span(style=dict(width=24, height=6, borderRadius=3, background="#60a5fa" if i < dm["step"] else ("#e5e7eb" if i == dm["step"] else "#3a3f47"))) for i in range(len(DEMO))]
    r["tab-arm:cls"] = "tab" + (" tab-on" if ui["tab"] == "arm" else "")
    r["tab-bench:cls"] = "tab" + (" tab-on" if ui["tab"] == "bench" else "")
    r["main-arm:style"] = dict(display="flex" if ui["tab"] == "arm" else "none", flexDirection="column", gap=20)
    r["main-bench:style"] = dict(display="flex" if ui["tab"] == "bench" else "none", flexDirection="column", gap=20)
    r["twin-graph:style"] = dict(height=380, display="block" if ui["view"] == "twin" else "none")
    r["stream:style"] = dict(display="flex" if ui["view"] == "stream" else "none", aspectRatio="640 / 420", background="#05070a",
                             border="1px dashed rgba(255,255,255,0.18)", borderRadius=6, alignItems="center", justifyContent="center",
                             flexDirection="column", gap=8, textAlign="center")
    r["v-twin:cls"] = on(ui["view"] == "twin", "btn btn-s")
    r["v-stream:cls"] = on(ui["view"] == "stream", "btn btn-s")
    r["stats"] = V.stat_cards(snap, cal)
    r["ctl-wrap:cls"] = "" if live else "disabled"
    r["ctl-note"] = [] if live else html.Div("Replay of a real log: the load and speed are what dev recorded. Switch to the stand-in plant to use the controls.", className="banner", style=dict(marginBottom=10))
    r["more:style"] = dict(display="flex" if ui["more"] else "none", flexDirection="column", gap=14, paddingTop=12,
                           borderTop="1px solid rgba(255,255,255,0.12)", marginTop=12)
    r["more-btn"] = "Fewer controls" if ui["more"] else "More controls"
    r["load-txt"] = f"{ctl.load:.2f} kg"
    mx = max(ctl.speeds)
    r["spd-txt"] = f"{round(np.mean(ctl.speeds) / 5) * 5}%" + (" · above band" if mx > BAND * 100 else "")
    for i in range(7):
        v = ctl.speeds[i]
        r[f"spd-txt-{i}"] = f"{v:.0f}% · {v / 100 * V.QDM[i]:.2f} rad/s" + (" · above band" if v > 50 else "")
    hist = list(an.hist)
    fig, title, sub = V.chart_figure(hist, ui["chan"], sel, cal, snap)
    r["chart-title"], r["chart-sub"], r["main-chart:fig"] = title + " · " + ("live" if live else "replay"), sub, fig
    jb_content, jb_cls = V.joint_bars(snap, sel, cal)
    r["brk-btn"] = f"{'Hide' if ui['brk'] else 'Show'} torque breakdown for joint {sel + 1}"
    r["brk"] = V.breakdown(snap, sel) if ui["brk"] else []
    snap_b = dict(snap)
    snap_b["_err"] = [h["est"] - h["load"] for h in hist[-200:]]
    r["main-bench"] = V.bench(snap_b, _real_pct(), ctl.speeds) if ui["tab"] == "bench" else no_update
    r["drawer:style"] = dict(display="flex" if ui["drawer"] else "none", flexDirection="column")
    r["drawer-title"] = TITLES.get(ui["drawer"], "")
    r["drawer-body"] = V.drawer_body(ui["drawer"], dict(snap, tare=an.tare), an, hist, snap["load"]) if ui["drawer"] else []
    toasts = [(aid, a) for aid, a in snap["alerts"].items() if a["sev"] > 0 and not a["dismissed"] and snap["t"] - a["born"] < 8][:3]
    sig = tuple((aid, a["born"]) for aid, a in toasts)
    if sig == _toast_sig:
        r["toasts"] = no_update
    else:
        _toast_sig = sig
        r["toasts"] = [html.Div(className="toast", role="status", style=dict(borderColor=V.SEV_COLOR[a["sev"]]), children=[
            html.Div(style=dict(display="flex", justifyContent="space-between", alignItems="flex-start", gap=8), children=[
                html.Div(style=dict(display="flex", alignItems="center", gap=8, minWidth=0), children=[
                    html.Span(className="dot", style=dict(background=V.SEV_COLOR[a["sev"]])), html.Span(a["title"], style=dict(fontSize=13, fontWeight=700))]),
                html.Button("×", id={"type": "dismiss", "aid": aid}, n_clicks=0, **{"aria-label": "Dismiss alert"},
                            style=dict(width=32, height=32, border=0, background="transparent", color="#9ca3af", fontSize=18, cursor="pointer"))]),
            html.Div(f"{a['where']} · {a['cause']}", style=dict(fontSize=12, color=V.SEV_COLOR[a["sev"]])),
            html.Div(a["lines"][0] if a["lines"] else "", className="line")]) for aid, a in toasts]
    for k, _ in PANELS:
        r[f"p-{k}:cls"] = on(ui["drawer"] == k, "btn btn-s")
    for k, _ in CHANS:
        r[f"ch-{k}:cls"] = on(ui["chan"] == k, "btn btn-s")
    r["twin-graph:fig"] = V.twin_figure(snap, snap["alerts"], snap["load"])
    return [r.get(o, no_update) for o in OUTS] + [jb_content, jb_cls]


if __name__ == "__main__":
    print("PAL dashboard on http://127.0.0.1:8050  (stand-in plant; not Isaac Sim)")
    app.run(debug=False, host="127.0.0.1", port=8050)
