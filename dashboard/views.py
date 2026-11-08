"""Pure functions: snapshot in, Dash components or Plotly figures out. No state here."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from dash import dcc, html

from engine import DOF, LIVE_Z, TAU_MAX, floor_pct

TL = TAU_MAX                      # torque limits of whichever arm is loaded
QDM = np.array([2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61])
SEV_COLOR = ["#9ca3af", "#fab219", "#fb923c", "#f87171"]
C_MEAS, C_PIN, C_GRAV, C_INER, C_COUP, C_FRIC, C_LOAD = "#e5e7eb", "#3987e5", "#199e70", "#d95926", "#9085e9", "#c98500", "#6ee7b7"
MONO = "JetBrains Mono, ui-monospace, monospace"
WIN_S = 12.0


def f2(v, d=2):
    return ("" if v >= 0 else "−") + f"{abs(v):.{d}f}"


# ── arm twin ────────────────────────────────────────────────────────────────────────────
def _tint(s):
    s = max(0.0, min(1.0, s))
    a, b, c = (75, 85, 99), (250, 178, 25), (248, 113, 113)
    u, v, t = (a, b, s * 2) if s < 0.5 else (b, c, (s - 0.5) * 2)
    return "rgb(%d,%d,%d)" % tuple(round(u[k] + (v[k] - u[k]) * t) for k in range(3))


def twin_figure(snap, alerts, load):
    g = snap["geom"]
    pts = [g["base"]] + list(g["joints"]) + [g["hand"]]
    fig = go.Figure()
    fig.add_shape(type="line", x0=-0.35, x1=0.95, y0=0, y1=0, line=dict(color="rgba(255,255,255,0.14)"))
    for k in range(len(pts) - 1):
        j = min(k, DOF - 1)              # the last segment (wrist to tool) is tinted by the last joint
        s = abs(snap["tau_meas"][j]) / TL[j] * 1.6
        fig.add_trace(go.Scatter(x=[pts[k][0], pts[k + 1][0]], y=[pts[k][2], pts[k + 1][2]], mode="lines",
                                 line=dict(color=_tint(s), width=11), hoverinfo="skip", showlegend=False))
    if load > 0.04:
        p = g["hand"] + np.array([0, 0, 0.0])
        fig.add_trace(go.Scatter(x=[p[0]], y=[p[2] - 0.05], mode="markers",
                                 marker=dict(size=10 + 7 * load, color="#e5e7eb", opacity=0.9), hoverinfo="skip", showlegend=False))
    est = max(snap["est"], 0.0)
    if est > 0.05:
        L = min(0.33, est * 9.81 * 0.012)
        h = g["hand"]
        fig.add_annotation(x=h[0], y=h[2] - 0.05 - L, ax=h[0], ay=h[2] - 0.05, xref="x", yref="y", axref="x", ayref="y",
                           showarrow=True, arrowhead=2, arrowwidth=3, arrowcolor="#fab219", text="")
        fig.add_annotation(x=h[0] + 0.03, y=h[2] - 0.05 - L / 2, text=f"{est * 9.81:.1f} N", showarrow=False,
                           font=dict(color="#fab219", size=12, family=MONO), xanchor="left")
    ring = {}
    for a in alerts.values():
        if a["sev"] > 0 and a["where"].startswith("Joint "):
            n = int(a["where"].split()[1]) - 1
            ring[n] = max(ring.get(n, 0), a["sev"])
    for i, p in enumerate(g["joints"]):
        sv = ring.get(i, 0)
        show = i % 2 == 0 or sv          # label every other joint, whatever the arm's count
        fig.add_trace(go.Scatter(x=[p[0]], y=[p[2]], mode="markers+text" if show else "markers",
                                 marker=dict(size=17 if sv else 11, color="#0d0f13", line=dict(color=SEV_COLOR[sv] if sv else "#e5e7eb", width=2.5)),
                                 text=[f"J{i + 1} {abs(snap['tau_meas'][i] / TL[i] * 100):.0f}%"] if show else None,
                                 textposition="top right", textfont=dict(color=SEV_COLOR[sv] if sv else "#c7cbd1", size=12, family=MONO),
                                 hoverinfo="skip", showlegend=False))
    # The frame has to hold whatever arm is loaded, so the view is sized from the points actually
    # drawn rather than fixed to one arm's reach.
    xs = [p[0] for p in pts]; zs = [p[2] for p in pts]
    pad = 0.18
    x0, x1 = min(xs) - pad, max(xs) + pad
    z0, z1 = min(min(zs) - pad, -0.05), max(zs) + pad
    fig.update_layout(_base_layout(height=380, margin=dict(l=0, r=0, t=0, b=0)),
                      xaxis=dict(range=[x0, x1], visible=False, scaleanchor="y"),
                      yaxis=dict(range=[z0, z1], visible=False))
    return fig


def _base_layout(**kw):
    return go.Layout(paper_bgcolor="#0e1218", plot_bgcolor="#0e1218", font=dict(color="#9ca3af", family="Inter, system-ui, sans-serif", size=11),
                     uirevision="keep", **kw)


# ── charts ──────────────────────────────────────────────────────────────────────────────
def _series(hist, f):
    return np.array([h["t"] for h in hist]), np.array([f(h) for h in hist])


def chart_figure(hist, chan, sel, cal, snap, height=240):
    fig = go.Figure()
    if len(hist) < 3:
        fig.update_layout(_base_layout(height=height))
        return fig, "Collecting…", ""
    hist = list(hist)
    t1 = hist[-1]["t"]
    hist = [h for h in hist if h["t"] >= t1 - WIN_S]
    sub = ""
    if chan == "res":
        t, y = _series(hist, lambda h: h["r"][sel])
        fig.add_trace(go.Scatter(x=t - t1, y=y, mode="lines", name="r = measured − model", line=dict(color=C_MEAS, width=2)))
        if cal["ready"]:
            b = LIVE_Z * cal["sd_u"][sel]
            fig.add_hrect(y0=-b, y1=b, fillcolor="#fab219", opacity=0.14, line_width=0)
            sub = f"N·m · shaded = no-alarm band ±{b:.2f}"
        else:
            sub = "N·m · alarm band appears after calibration"
        title, yr = f"Joint {sel + 1} residual", None
    elif chan == "tau":
        t, y1 = _series(hist, lambda h: h["tau_meas"][sel])
        _, y2 = _series(hist, lambda h: h["tau_model"][sel])
        fig.add_trace(go.Scatter(x=t - t1, y=y1, mode="lines", name="measured", line=dict(color=C_MEAS, width=2)))
        fig.add_trace(go.Scatter(x=t - t1, y=y2, mode="lines", name="model (Pinocchio)", line=dict(color=C_PIN, width=2)))
        title, sub, yr = f"Joint {sel + 1} torque", "N·m", None
    else:
        t, y = _series(hist, lambda h: h["est"])
        _, ld = _series(hist, lambda h: h["load"])
        fig.add_trace(go.Scatter(x=t - t1, y=y, mode="lines", name="estimate", line=dict(color=C_PIN, width=2)))
        fig.add_trace(go.Scatter(x=t - t1, y=ld, mode="lines", name="true load" if snap.get("truth") else "load",
                                 line=dict(color="#9ca3af", width=2, dash="dash")))
        title, sub, yr = "Payload estimate against the load", "kg · 1 s running mean", None
    fig.update_layout(_base_layout(height=height, margin=dict(l=44, r=8, t=6, b=30), showlegend=True,
                                   legend=dict(orientation="h", y=1.12, x=0)),
                      xaxis=dict(range=[-WIN_S, 0], gridcolor="rgba(255,255,255,0.05)", ticksuffix=" s", zeroline=False),
                      yaxis=dict(gridcolor="rgba(255,255,255,0.05)", zerolinecolor="rgba(255,255,255,0.14)", range=yr))
    return fig, title, sub


# ── small pieces ────────────────────────────────────────────────────────────────────────
def track(v, scale, color):
    w = min(50.0, abs(v) * scale * 50)
    left = 50 if v >= 0 else 50 - w
    return html.Span(className="track", children=[html.I(style=dict(left=f"{left}%", width=f"{w}%", background=color)), html.U()])


STYLE_NAME = {"trapezoidal": "Trapezoidal", "s_curve": "S-curve", "torque_aware": "Torque-aware"}
FAULT_NAME = {"none": "None", "friction": "Friction ×8", "stuck_joint": "Locked joint", "delay": "Command delay 50 ms"}


def _kv(k, v, tone=None):
    return html.Div(className="kv", children=[html.Span(k, className="k"), html.Span(v, className="v", style=dict(color=tone) if tone else None)])


def _chip(text, kind):
    return html.Span(text, className=f"chip chip-{kind}")


def summary_card(snap, status, ctl):
    """What this run is doing and what came out of it, in plain words; it stays on screen after the pick ends."""
    sm = snap["summary"]
    info, rows = sm["info"], []
    j, pk = sm["peak"]
    grp = {}
    for w, t in sm["alerts"]:
        grp.setdefault(t, []).append(w.replace("Joint ", ""))
    alerts = [f"{t} ({'joint ' if all(x.isdigit() for x in ws) else ''}{', '.join(sorted(ws))})" for t, ws in grp.items()]
    if sm["picks"]:
        st = (status or {})
        running = sm["running"] and st.get("state") == "running"
        last = st.get("last") if not running else None
        chip = _chip("RUNNING", "run") if running else (_chip("FINISHED", "ok") if (info and last) else _chip("READY", "idle"))
        if info:
            boxes = info.get("boxes") or 0
            rows += [_kv("Task", f"{STYLE_NAME.get(info.get('style'), '—')} · {info.get('kg', 0):g} kg boxes "
                                 f"· speed {round(100 * info.get('speed', 0))}%"),
                     _kv("Boxes placed", f"{info.get('boxes_placed', 0)} of {boxes}" if boxes else "—"),
                     _kv("Fault", FAULT_NAME.get(info.get("fault", "none"), "None") + (f" on joint {info['joint'] + 1}" if info.get("fault", "none") not in ("none", "delay") else ""))]
            dur = info.get("duration") or 0
            if running and dur:
                rows.append(html.Div(className="kv", children=[html.Span("Progress", className="k"),
                    html.Div(className="prog", children=[html.Div(className="prog-track", children=html.Div(className="prog-bar", style=dict(width=f"{min(100, 100 * (sm['elapsed'] or 0) / dur):.0f}%"))),
                                                         html.Span(f"{min(sm['elapsed'] or 0, dur):.1f} / {dur:.1f} s", className="num", style=dict(whiteSpace="nowrap"))])]))
            ce = sm["carry_est"]
            if ce is not None:
                rows.append(_kv("Payload", f"set {info.get('kg', 0):.2f} kg · estimated {ce:.2f} kg ({ce - info.get('kg', 0):+.2f})" + (" · a fault was active, so this is unreliable" if alerts else ""),
                                "#34d399" if (abs(ce - info.get("kg", 0)) <= max(0.05 * info.get("kg", 1), 0.06) and not alerts) else "#fab219"))
            else:
                rows.append(_kv("Payload", "estimated once a box is lifted" if running else "—"))
            rows.append(_kv("Typical peak residual", f"J{j + 1} · {pk:.1f}% of its torque limit"))
            rows.append(_kv("Alarms", "None" if not alerts else "; ".join(alerts), "#34d399" if not alerts else "#fb923c"))
            det = {"recorded": "Healthy run: saved as the reference for this cycle",
                   "compared": "Compared with the healthy reference for this cycle",
                   "none": "No healthy reference for this cycle: fault alarms are off"}[sm["ref"]]
            rows.append(_kv("Fault detection", det))
            if last:
                n, total = last.get("boxes_placed", 0), last.get("boxes", 0)
                verdict = (f"All {total} boxes placed on the belt" if last["ok"] else
                           "Stopped" if last["aborted"] else
                           f"Cycle incomplete: {n} of {total} boxes placed")
                rows.append(_kv("Result", verdict, "#34d399" if last["ok"] else "#f87171"))
        else:
            rows.append(html.Div("Press Run pick. The task, the payload estimate, the largest residual and every alarm are summarised here as the cycle runs.", className="muted"))
    else:
        chip = _chip("LIVE", "run")
        est = snap["est"]
        rows += [_kv("Mock load", f"set {ctl.load:.2f} kg · estimated {est:.2f} kg ({est - ctl.load:+.2f})" if ctl else "—"),
                 _kv("Joint speed", f"{round(np.mean(ctl.speeds) / 0.5 / 5) * 5}% of the speed bar" if ctl else "—"),
                 _kv("Test fault", FAULT_NAME.get(ctl.fault, "None") if ctl else "—"),
                 _kv("Peak residual (20 s)", f"J{j + 1} · {pk:.1f}% of its torque limit"),
                 _kv("Alarms", "None" if not alerts else " · ".join(alerts), "#34d399" if not alerts else "#fb923c")]
    return html.Div(className="panel", children=[
        html.Div(className="spread", children=[html.Span("RUN SUMMARY", className="lbl"), chip]), html.Div(className="kvs", children=rows)])


def alerts_card(snap):
    active = sorted([a for aid, a in snap["alerts"].items() if a["sev"] > 0 and aid != "payload"], key=lambda a: (-a["sev"], a["born"]))
    groups = {}
    for a in active:
        groups.setdefault((a["title"], a["sev"]), []).append(a["where"].replace("Joint ", "").replace(" carries most of it", ""))
    rows = [html.Div(className="arow", children=[html.Span(className="dot", style=dict(background=SEV_COLOR[sv])),
                                                 html.Span(t, className="atitle"), html.Span(("joint " if all(w.isdigit() for w in ws) else "") + ", ".join(ws), className="awhere")])
            for (t, sv), ws in list(groups.items())[:4]]
    return html.Div(className="panel", children=[
        html.Div(className="spread", children=[html.Span("ALERTS", className="lbl"), _chip(f"{len(active)}", "bad" if active else "ok")]),
        html.Div(className="kvs", children=rows or [html.Div(className="arow", children=[html.Span(className="dot", style=dict(background="#34d399")),
                                                                                         html.Span("No active alerts", className="atitle")])])])


def stat_cards(snap, cal, status=None, ctl=None):
    worst = int(np.argmax(np.abs(snap["r_pct"])))
    fault_active = any(a.startswith(("fric", "stuck", "delay")) for a in snap["alerts"])
    holding = snap["load"] > 0
    state, state_col = ("fault alarm active: the estimate may be off", "#fab219") if fault_active else (
        ("holding a box" if holding else "tool empty") if snap["summary"]["picks"] else ("estimating", "#9ca3af")[0], "#9ca3af")
    return [
        html.Div(className="kpis", children=[
            html.Div(className="kpi", children=[html.Span("PAYLOAD ESTIMATE", className="lbl"),
                *( [html.Span("—", className="num big"), html.Span("kg", className="unit"),
                    html.Span("run a pick to estimate the carried mass", className="sub")]
                   if (snap["summary"]["picks"] and not snap["summary"]["running"]) else
                   [html.Span(f"{snap['est']:.2f}", className="num big"), html.Span("kg", className="unit"),
                    html.Span(f"± {snap['band']:.2f} kg · {state}", className="sub", style=dict(color=state_col))])]),
            html.Div(className="kpi", children=[html.Span("LARGEST RESIDUAL", className="lbl"),
                html.Span(f"J{worst + 1} · {abs(snap['r_pct'][worst]):.1f}%", className="num big"),
                html.Span(f"of its torque limit · {f2(snap['r'][worst])} N·m", className="sub")])]),
        summary_card(snap, status, ctl), alerts_card(snap)]


def joint_bars(snap, sel, cal):
    """Contents and class names for the seven fixed joint buttons (the buttons themselves live in the
    layout, so redrawing them cannot fire their click callbacks)."""
    contents, classes = [], []
    for i in range(DOF):
        thr = LIVE_Z * cal["sd_u"][i] if cal["ready"] else 1e9
        col = "#fab219" if abs(snap["r"][i]) > thr else "#9ca3af"
        contents.append([html.Span(f"J{i + 1}"), track(snap["r_pct"][i] / 100, 12, col),
                         html.Span(f"{f2(snap['r_pct'][i], 1)}%", className="num", style=dict(textAlign="right"))])
        classes.append("jbar" + (" on" if sel == i else ""))
    return contents, classes


def breakdown(snap, sel):
    lim = TL[sel]
    c = snap["comp"]
    parts = [("Gravity of the arm", c["gravity"][sel], C_GRAV), ("Inertia (M q̈)", c["inertia"][sel], C_INER),
             ("Coupling (C q̇)", c["coupling"][sel], C_COUP), ("Friction (model)", c["friction"][sel], C_FRIC),
             ("Not explained (residual)", c["unexplained"][sel], C_LOAD)]
    rows = [html.Div(style=dict(display="grid", gridTemplateColumns="minmax(120px,190px) minmax(0,1fr) 64px", alignItems="center", gap=10, fontSize=12),
                     children=[html.Span(n), track(v / lim, 2.2, col), html.Span(f2(v), className="num", style=dict(textAlign="right"))])
            for n, v, col in parts]
    return html.Div(style=dict(display="flex", flexDirection="column", gap=8, background="#0d0f13", borderRadius=6, padding="12px 14px"),
                    children=[html.Span(f"Signed N·m; bars scaled to the joint's limit ({lim:.0f} N·m). The first four are what the twin predicts; "
                                        "the last is what it cannot explain.", className="muted", style=dict(fontSize=12))] + rows)


# ── drawer ──────────────────────────────────────────────────────────────────────────────
def drawer_body(kind, snap, an, hist, load_truth):
    if kind == "o1":
        rows = [html.Tr([html.Td(f"J{i + 1}"), html.Td(f2(snap["q"][i], 3)), html.Td(f2(snap["qd"][i], 2)), html.Td(f2(snap["tau_meas"][i], 1)),
                         html.Td(f2(snap["tau_model"][i], 1)), html.Td(f2(snap["r"][i]))]) for i in range(DOF)]
        return [html.Div(style=dict(overflowX="auto"), children=html.Table(className="tbl", children=[
                    html.Thead(html.Tr([html.Th(h) for h in ("Joint", "q (rad)", "qd", "τ meas", "τ model", "resid")])), html.Tbody(rows)])),
                html.Div(style=dict(display="grid", gridTemplateColumns="1fr 1fr", gap=10, fontSize=12), children=[
                    html.Div([html.Div("FRAME", className="lbl"), html.Div(f"{snap['n']} · display 2 Hz", className="num")]),
                    html.Div([html.Div("ANALYSIS", className="lbl"), html.Div("1.2 s window, 20 Hz filter")]),
                    html.Div([html.Div("SCHEMA", className="lbl"), html.Div("telemetry v0")]),
                    html.Div([html.Div("FEED HEALTH", className="lbl"), html.Div("not reported")])])]
    if kind == "o4":
        err = snap["est"] - load_truth
        fig, _, _ = chart_figure(hist, "pay", 0, an.cal, snap, height=200)
        bars = [html.Div(style=dict(display="grid", gridTemplateColumns="28px minmax(0,1fr) 60px", alignItems="center", gap=8, fontSize=12), children=[
                    html.Span(f"J{i + 1}"), track(snap["tkg"][i] * snap["est"] / TL[i], 9, C_LOAD),
                    html.Span(f2(snap["tkg"][i] * snap["est"]), className="num", style=dict(textAlign="right"))]) for i in range(DOF)]
        state = ("Fault alarm active: the estimate may be off" if any(a.startswith(("fric", "stuck", "delay")) for a in snap["alerts"]) else "Estimating")
        return [html.Div(style=dict(display="grid", gridTemplateColumns="1fr 1fr", gap=10), children=[
                    html.Div([html.Div("ESTIMATE", className="lbl"), html.Div(f"{snap['est']:.3f} kg", className="num", style=dict(fontSize=24)),
                              html.Div(f"± {snap['band']:.3f} kg", className="muted", style=dict(fontSize=12))]),
                    html.Div([html.Div("LOAD (known to the source)", className="lbl"), html.Div(f"{load_truth:.3f} kg", className="num", style=dict(fontSize=24)),
                              html.Div(f"error {f2(err, 3)} kg", className="muted", style=dict(fontSize=12, color="#fab219" if abs(err) > 0.06 else "#34d399"))])]),
                html.Span(state, className="muted", style=dict(fontSize=12, color="#fab219" if state != "Estimating" else "#34d399")),
                html.Div(className="row", children=[html.Button("Tare (gripper empty)", id="tare-on", n_clicks=0, className="btn btn-s"),
                                                    html.Button("Clear tare", id="tare-off", n_clicks=0, className="btn btn-s"),
                                                    html.Span(f"tare {snap['tare']:.3f} kg", className="num", style=dict(fontSize=12, color="#9ca3af"))]),
                dcc.Graph(figure=fig, config=dict(displayModeBar=False), style=dict(height=200)),
                html.Div(style=dict(display="flex", flexDirection="column", gap=6), children=[html.Span("WHERE THE LOAD SHOWS UP, N·M PER JOINT", className="lbl")] + bars)]
    if kind == "alerts":
        active = sorted([a for a in snap["alerts"].values() if a["sev"] > 0], key=lambda a: (-a["sev"], a["born"]))
        cards = [html.Div(className="alert", style=dict(borderColor=SEV_COLOR[a["sev"]]), children=[
                    html.Div(style=dict(display="flex", alignItems="center", gap=8), children=[
                        html.Span(className="dot", style=dict(background=SEV_COLOR[a["sev"]])), html.Span(a["title"], style=dict(fontSize=13, fontWeight=700))]),
                    html.Div(f"{a['where']} · {a['cause']}", style=dict(fontSize=12, color=SEV_COLOR[a["sev"]]))] +
                    [html.Div(t, className="line") for t in a["lines"]]) for a in active]
        log = [html.Li([html.Span(l["t"], style=dict(color="#e5e7eb")), " " + l["text"]], className="num", style=dict(fontSize=12, color="#9ca3af")) for l in snap["log"]]
        return (cards or [html.P("No active alerts.", className="muted", style=dict(margin=0))]) + [
            html.Span("RECENT EVENTS", className="lbl"),
            html.Ul(log, style=dict(listStyle="none", margin=0, padding=0, display="flex", flexDirection="column", gap=6)) if log
            else html.P("Nothing raised yet.", className="muted", style=dict(margin=0))]
    b = lambda h, t: html.P([html.B(h + " ", style=dict(color="#e5e7eb")), t], className="muted", style=dict(margin=0))   # noqa: E731
    return [b("Independent twin.", "Pinocchio 7-DOF Panda from the empty-arm URDF (gripper 0.93 kg included), armature 0.0061, Gaz friction. "
                                   "Residual r = τ measured − τ model. The twin is never told the payload."),
            b("Where the numbers come from.", "Stand-in mode: a Pinocchio plant (link masses ±3%, friction ×1.15, sensor noise) driven by a joint "
                                              "impedance controller, stepped live. Replay mode: recorded Isaac Sim slow-reference logs, looped. "
                                              "Analysis runs on a sliding 1.2 s window."),
            b("Checks.", "The twin matches the simulator's gravity torque to under 0.15 N·m at every probe pose."),
            b("Reference logs.", "Empty arm, matched twin, no noise, 22% of the speed limit: 0.01% of limit on joints 1–4 and 0.08% on joints 5–6. 1 kg reads 1.000 kg. "
                                 "With the plant's deliberate 5% mass mismatch the elbow (joint 4) shows a steady 0.8 N·m, and a wrist joint that barely moves can show "
                                 "up to its static friction, about 0.2 N·m, which is 1.7% of its 12 N·m limit."),
            b("Limits.", "Stand-in numbers are not Isaac Sim's. The URDF parameters make twin and plant agree; they are not a real arm's. "
                                "Faults are found by comparing a pick with a healthy run of the same pick. Live, the residual lags by 0.15 s (filter edge)."),
            b("Why the speed bar tops out at half speed.", "The bar runs from 0 to 100%, and 100% on the bar is half of each joint's maximum velocity. "
                             "In this range the torque the model predicts and the torque the arm applies agree to within about 2% of a joint's limit, "
                             "and that agreement does not depend on any simulator setting. A clean residual means any change in it has a physical cause: "
                             "a payload, extra friction or a stuck joint. Keeping the arm here keeps every reading on the dashboard trustworthy.")]


# ── benchmark tab ───────────────────────────────────────────────────────────────────────
_STYLE_COL = {"trapezoidal": "#60a5fa", "s_curve": "#34d399", "torque_aware": "#fab219"}


def _o6_data():
    try:
        return json.loads((Path(__file__).resolve().parents[1] / "docs" / "o6pick_summary.json").read_text())
    except (OSError, ValueError):
        return None


def _style_figure(runs, kg):
    fig = go.Figure()
    for st in ("trapezoidal", "s_curve", "torque_aware"):
        rr = sorted([r for r in runs if r["style"] == st and r["payload_kg"] == kg], key=lambda r: r["cycle_time_s"])
        fig.add_trace(go.Scatter(x=[100 * r["peak_torque_frac"] for r in rr], y=[r["cycle_time_s"] for r in rr], mode="lines+markers", name=STYLE_NAME[st],
                                 line=dict(color=_STYLE_COL[st], width=2), marker=dict(size=9),
                                 customdata=[[round(100 * r["speed_fraction"]), r["energy_drawn_J"]] for r in rr],
                                 hovertemplate="%{y:.1f} s · peak %{x:.0f}% · top speed %{customdata[0]}% · %{customdata[1]:.0f} J<extra>" + STYLE_NAME[st] + "</extra>"))
    fig.update_layout(height=270, margin=dict(l=48, r=12, t=28, b=44), paper_bgcolor="rgba(0,0,0,0)", plot_bgcolor="rgba(0,0,0,0)",
                      title=dict(text=("Empty gripper" if kg == 0 else f"{kg:g} kg payload"), x=0.0, font=dict(size=13, color="#e5e7eb")),
                      font=dict(color="#9ca3af", size=11), legend=dict(orientation="h", y=-0.28, x=0),
                      xaxis=dict(title="Peak torque, % of limit", gridcolor="rgba(255,255,255,0.06)", zeroline=False),
                      yaxis=dict(title="Cycle time, s", gridcolor="rgba(255,255,255,0.06)", zeroline=False, rangemode="tozero"))
    return fig


def _o6_section():
    d = _o6_data()
    if not d:
        return html.Section(className="card", children=[html.H2("Movement styles", className="h2"),
                                                        html.P("The benchmark results file (docs/o6pick_summary.json) is not available.", className="muted")])
    runs = d["runs"]
    budget_rows = []
    for b in d["shortest_under_budget"]:
        vals = {k: b[k] for k in STYLE_NAME}
        best = min([v for v in vals.values() if v is not None], default=None)
        budget_rows.append(html.Tr([html.Td(f"{b['payload_kg']:g} kg"), html.Td(f"{round(100 * b['budget'])}%")] + [
            html.Td("—" if v is None else f"{v:.2f} s", style=dict(color="#34d399", fontWeight=600) if (v is not None and v == best) else None) for v in vals.values()]))
    run_rows = [html.Tr([html.Td(STYLE_NAME[r["style"]]), html.Td(f"{r['payload_kg']:g} kg"), html.Td(f"{round(100 * r['speed_fraction'])}%"),
                         html.Td(f"{r['cycle_time_s']:.2f} s"), html.Td(f"{100 * r['peak_torque_frac']:.0f}%"), html.Td(f"{r['energy_drawn_J']:.0f} J"),
                         html.Td(f"{r['track_err_max_rad']:.2f} rad")])
                for r in sorted(runs, key=lambda r: (r["payload_kg"], r["style"], r["cycle_time_s"]))]
    return html.Section(className="card", children=[
        html.Div(children=[html.H2("Movement styles", className="h2"),
                           html.P("One pick-and-place cycle in three trajectory styles, each at three speeds, with an empty gripper and with 2 kg. "
                                  "Lower and further left is better: a shorter cycle for less peak torque. These runs go faster than the 50% speed band "
                                  "the live demo stays in, so they are used for time, torque and energy only, never for fault detection.", className="muted", style=dict(margin="4px 0 0"))]),
        html.Div(className="grid2", children=[dcc.Graph(figure=_style_figure(runs, kg), config=dict(displayModeBar=False)) for kg in (0.0, 2.0)]),
        html.Div(className="grid2", children=[
            html.Div(children=[html.Span("FASTEST CYCLE UNDER A PEAK-TORQUE BUDGET", className="lbl"),
                               html.Table(className="tbl", children=[html.Thead(html.Tr([html.Th(h) for h in ("Payload", "Budget", "Trapezoidal", "S-curve", "Torque-aware")])),
                                                                     html.Tbody(budget_rows)]),
                               html.Span("Green is the fastest style that stays under the budget; a dash means none does.", className="muted", style=dict(fontSize=12))]),
            html.Div(children=[html.Span("ALL 18 RUNS", className="lbl"),
                               html.Div(style=dict(maxHeight=250, overflowY="auto"), children=html.Table(className="tbl", children=[
                                   html.Thead(html.Tr([html.Th(h) for h in ("Style", "Payload", "Top speed", "Cycle", "Peak torque", "Energy", "Max tracking error")])), html.Tbody(run_rows)]))])])])


def bench(snap, real_pct, speeds):
    spd = np.array(speeds) / 100
    regimes = [(0.1, "physical"), (0.2, "physical"), (0.3, "physical"), (0.5, "physical, knee"), (0.75, "drive artifact"), (1.0, "drive artifact")]
    rows, lo = [], 0.0
    for s, rg in regimes:
        js = [f"J{i + 1}" for i in range(DOF) if lo + 1e-9 < spd[i] <= s + 1e-9]
        rows.append(html.Tr([html.Td(f"{round(s * 100)}%"), html.Td(f"{floor_pct(s):.2f}%"), html.Td(rg), html.Td(" ".join(js) or "—", style=dict(color="#60a5fa"))]))
        lo = s
    m = snap["metrics"]
    errs = [abs(h) for h in snap.get("_err", [])]
    tiles = [("PAYLOAD ERROR", f"{np.mean(errs):.3f} kg" if errs else "—", "mean |estimate − load|, last 10 s"),
             ("PAYLOAD DETECTION DELAY", "—" if m["delay_payload"] is None else f"{m['delay_payload']:.2f} s", "last load change to the alert"),
             ("FAULT DETECTION DELAY", "—" if m["delay_fault"] is None else f"{m['delay_fault']:.2f} s", m["fault_kind"] or "inject a fault to measure"),
             ("FALSE ALARMS", str(m["false_alarms"]), "no load, no fault, speeds ≤ 50%")]
    return [
        _o6_section(),
        html.Section(className="card", children=[
            html.H2("This session", className="h2"),
            html.P("How this session is doing against the load and fault you set. Stand-in numbers are optimistic: the plant and the twin share a URDF.", className="muted", style=dict(margin=0)),
            html.Div(style=dict(display="grid", gridTemplateColumns="repeat(auto-fit,minmax(190px,1fr))", gap=12), children=[
                html.Div(className="stat dark", children=[html.Span(a, className="lbl"), html.Span(b_, className="num", style=dict(fontSize=22, fontWeight=500)),
                                                          html.Span(c_, className="muted", style=dict(fontSize=12))]) for a, b_, c_ in tiles]),
            html.H3("Why the demo stays at or below half speed: the residual floor against speed", className="h2", style=dict(fontSize=13)),
            html.Div(style=dict(overflowX="auto"), children=html.Table(className="tbl", children=[
                html.Thead(html.Tr([html.Th(h) for h in ("Speed", "Worst-joint RMS, % of limit", "Regime", "Joints now in this band")])), html.Tbody(rows)])),
            html.H3("Empty-arm reference log: RMS residual, % of limit", className="h2", style=dict(fontSize=13)),
            html.Div(className="num", style=dict(display="grid", gridTemplateColumns=f"repeat({DOF},minmax(0,1fr))", gap=8, fontSize=12, textAlign="center"), children=[
                html.Div(style=dict(background="#0d0f13", borderRadius=6, padding="8px 4px"), children=[html.Div(f"J{i + 1}", className="lbl"), html.Div(f"{v:.2f}%")])
                for i, v in enumerate(real_pct)])])]
