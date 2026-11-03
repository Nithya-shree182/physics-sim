#!/usr/bin/env python3
"""O6 comparison chart as a dependency-free SVG: cycle time against peak torque, three styles.

    python3 tools/plot_o6.py out/o6/summary.json docs/o6_comparison.svg

One panel per payload, one line per movement style, one marker per run. Lower-left is better.
"""
import json
import sys

src, dst = sys.argv[1], sys.argv[2]
runs = json.load(open(src))["runs"]
styles = [("trapezoidal", "#1f77b4", "circle"), ("s_curve", "#d95f02", "square"), ("torque_aware", "#2a9d3f", "tri")]
payloads = sorted({r["payload_kg"] for r in runs})
W, H, PAD = 430, 330, 55
tmax = max(r["cycle_time_s"] for r in runs) * 1.05
pmax = 100 * max(r["peak_torque_frac"] for r in runs) * 1.1


def marker(kind, x, y, c):
    if kind == "circle":
        return f'<circle cx="{x:.1f}" cy="{y:.1f}" r="4.5" fill="{c}"/>'
    if kind == "square":
        return f'<rect x="{x - 4:.1f}" y="{y - 4:.1f}" width="8" height="8" fill="{c}"/>'
    return f'<polygon points="{x:.1f},{y - 5:.1f} {x - 5:.1f},{y + 4:.1f} {x + 5:.1f},{y + 4:.1f}" fill="{c}"/>'


out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W * len(payloads)}" height="{H + 30}" '
       f'font-family="sans-serif" font-size="12"><rect width="100%" height="100%" fill="white"/>',
       f'<text x="10" y="18" font-size="14" font-weight="bold">O6: cycle time against peak torque (lower-left is better)</text>']
for pi, kg in enumerate(payloads):
    ox = pi * W
    X = lambda t: ox + PAD + (W - PAD - 20) * t / tmax
    Y = lambda p: 30 + (H - PAD - 10) * (1 - p / pmax)
    out.append(f'<text x="{ox + PAD}" y="48" font-weight="bold">payload {kg:g} kg</text>')
    out.append(f'<line x1="{X(0)}" y1="{Y(0)}" x2="{X(tmax)}" y2="{Y(0)}" stroke="#444"/>'
               f'<line x1="{X(0)}" y1="{Y(0)}" x2="{X(0)}" y2="{Y(pmax)}" stroke="#444"/>')
    for t in range(0, int(tmax) + 1):
        out.append(f'<text x="{X(t):.0f}" y="{Y(0) + 16:.0f}" text-anchor="middle">{t}</text>')
    for p in range(0, int(pmax) + 1, 20):
        out.append(f'<line x1="{X(0)}" y1="{Y(p):.0f}" x2="{X(tmax)}" y2="{Y(p):.0f}" stroke="#ddd"/>'
                   f'<text x="{X(0) - 6:.0f}" y="{Y(p) + 4:.0f}" text-anchor="end">{p}</text>')
    out.append(f'<text x="{(X(0) + X(tmax)) / 2:.0f}" y="{H + 22}" text-anchor="middle">cycle time (s)</text>')
    out.append(f'<text x="{ox + 14}" y="{(Y(0) + Y(pmax)) / 2:.0f}" transform="rotate(-90 {ox + 14} {(Y(0) + Y(pmax)) / 2:.0f})" '
               f'text-anchor="middle">peak torque (% of limit)</text>')
    for si, (style, colour, kind) in enumerate(styles):
        pts = sorted(((r["cycle_time_s"], 100 * r["peak_torque_frac"]) for r in runs
                      if r["style"] == style and r["payload_kg"] == kg))
        out.append('<polyline fill="none" stroke="%s" stroke-width="1.5" points="%s"/>'
                   % (colour, " ".join(f"{X(t):.1f},{Y(p):.1f}" for t, p in pts)))
        out += [marker(kind, X(t), Y(p), colour) for t, p in pts]
        out.append(marker(kind, ox + W - 135, 62 + 16 * si, colour) +
                   f'<text x="{ox + W - 124}" y="{66 + 16 * si}">{style.replace("_", "-")}</text>')
out.append("</svg>")
open(dst, "w").write("\n".join(out))
print("wrote", dst)
