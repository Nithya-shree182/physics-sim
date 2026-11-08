#!/usr/bin/env python3
"""O6 on real pick cycles: cycle time, peak torque and energy per pick, from the written runs (out/o6pick).

    .venv-tools/bin/python tools/summarize_o6pick.py [out/o6pick] [docs/o6pick_comparison.svg]

Per run, from tau_applied and qd in the Parquet (nothing stored by the simulator): cycle time (the task's duration: home to home), peak
torque as a fraction of each joint's datasheet limit, energy drawn (the positive part of the sum of tau * qd * dt) and the back-driven part
(the negative part), and the worst tracking error. Then the shortest cycle time that keeps the peak torque within a budget, for each style
and payload: the proposal's "how should the arm move so that the motion time is as short as possible" for a given payload.
"""
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "workspace"))
import pal_config as cfg  # noqa: E402

TAU_MAX = np.asarray(cfg.TAU_MAX, float)        # the active arm's datasheet effort limits (was the Panda's, hard-coded)
DOF = cfg.DOF
BUDGETS = (0.40, 0.55, 0.70)


def main(out, svg):
    out = Path(out)
    rows = []
    for p in sorted(out.glob("o6pick_*.parquet")):
        d = pd.read_parquet(p)
        man = json.loads(p.with_suffix(".json").read_text())
        arr = lambda c: np.stack([d[f"{c}_{i}"].to_numpy() for i in range(DOF)], axis=1)
        tau, qd, q, qc = arr("tau_applied"), arr("qd"), arr("q"), arr("q_cmd")
        dt = float(d["dt"].iloc[0])
        power = tau * qd
        kg = float(d["payload_kg"].max())
        rows.append({"run_id": p.stem.replace("_1000hz", ""), "style": man["task"]["name"].replace("pick_", ""), "payload_kg": kg,
                     "cycle_time_s": man["task"]["duration_s"], "peak_torque_frac": float(np.max(np.abs(tau) / TAU_MAX)),
                     "energy_drawn_J": float(np.sum(np.clip(power, 0, None)) * dt),
                     "energy_backdriven_J": float(np.sum(np.clip(power, None, 0)) * dt),
                     "track_err_max_rad": float(np.max(np.abs(q - qc))), "speed_fraction": man.get("speed_fraction")})
    if not rows:
        print("no o6pick runs found")
        return 1
    print(f"{'style':<14}{'payload':>8}{'cycle s':>9}{'peak torque':>13}{'energy J':>10}{'back-driven J':>15}{'track err rad':>15}{'peak speed':>12}")
    for r in sorted(rows, key=lambda r: (r["payload_kg"], r["style"], r["cycle_time_s"])):
        print(f"{r['style']:<14}{r['payload_kg']:>7g}k{r['cycle_time_s']:>9.2f}{r['peak_torque_frac']:>12.0%}{r['energy_drawn_J']:>10.1f}"
              f"{r['energy_backdriven_J']:>15.1f}{r['track_err_max_rad']:>15.4f}{r['speed_fraction']:>11.0%}")
    styles = ("trapezoidal", "s_curve", "torque_aware")
    best = []
    print("\nshortest cycle time that keeps peak torque within a budget (fraction of datasheet limits):")
    print(f"  {'payload':>8}{'budget':>8}" + "".join(f"{s:>16}" for s in styles))
    for kg in sorted({r["payload_kg"] for r in rows}):
        for b in BUDGETS:
            rec, line = {"payload_kg": kg, "budget": b}, f"  {kg:>7g}k{b:>8.0%}"
            for s in styles:
                ok = [r for r in rows if r["style"] == s and r["payload_kg"] == kg and r["peak_torque_frac"] <= b]
                pick = min(ok, key=lambda r: r["cycle_time_s"]) if ok else None
                rec[s] = pick["cycle_time_s"] if pick else None
                line += f"{(format(pick['cycle_time_s'], '.2f') + ' s') if pick else 'none':>16}"
            best.append(rec)
            print(line)
    Path("docs/o6pick_summary.json").write_text(json.dumps({"runs": rows, "shortest_under_budget": best}, indent=2))
    subprocess.run([sys.executable, str(ROOT / "tools/plot_o6.py"), "docs/o6pick_summary.json", svg], check=False)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "out/o6pick", sys.argv[2] if len(sys.argv) > 2 else "docs/o6pick_comparison.svg"))
