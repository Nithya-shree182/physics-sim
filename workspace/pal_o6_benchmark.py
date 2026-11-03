#!/usr/bin/env python3
"""O6: three movement styles between identical waypoints (proposal 7.6).

Out-and-back between HOME and HOME + OFFSET with a short dwell at each end, for two payloads.

  trapezoidal   velocity- and acceleration-limited trapezoid
  s_curve       jerk-limited 7-segment profile
  torque_aware  quintic (minimum-jerk) with its leg duration as the knob

Per run: cycle time, peak torque (as a fraction of each joint's limit), energy (the proposal's
sum of torque x velocity x dt; the positive part is energy drawn, the negative part is
back-driven motion), and the worst tracking error, which says how far the arm was from the plan.

The proposal's question: for a given payload, how fast can the arm move before it exceeds its
torque budget. These are effort-mode runs at speeds ABOVE the 50% regime; they use torque, time
and energy, never the residual, so the residual-floor artifact does not apply here.

    tools/pal_pipeline.sh pal_o6_benchmark.py out/o6
"""
import json
import os

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_o6_benchmark.py running", flush=True)

import numpy as np
import pal_config as cfg
import pal_trajectories as T
from pal_plant import build_plant
from pal_runner import Runner, Scenario

HZ = float(os.environ.get("PAL_PHYSICS_HZ", cfg.PHYSICS_HZ))
OUT = "/workspace/out/o6"
OFFSET = np.array([0.6, -0.3, 0.3, 0.4, 0.3, -0.4, 0.3])      # [CHOICE] the "pick" displacement
Q0, Q1 = cfg.HOME_JOINTS, cfg.HOME_JOINTS + OFFSET

plant = build_plant(HZ)
runner = Runner(plant, OUT, mj_every=10)


def metrics(tr, log, kg):
    dt = 1.0 / HZ
    cmd, qd = log["cmd"], log["qd"]
    power = cmd * qd
    return {"payload_kg": kg, "cycle_time_s": tr.duration,
            "peak_torque_frac": float(np.max(np.abs(cmd) / cfg.TAU_MAX)),
            "peak_torque_frac_per_joint": np.max(np.abs(cmd) / cfg.TAU_MAX, axis=0).tolist(),
            "energy_drawn_J": float(np.sum(np.clip(power, 0, None)) * dt),
            "energy_backdriven_J": float(np.sum(np.clip(power, None, 0)) * dt),
            "energy_net_J": float(np.sum(power) * dt),
            "track_err_max_rad": float(np.max(np.abs(log["q"] - log["q_des"])))}


def run(kind, tr, kg, tag):
    sc = Scenario(name=f"o6_{kind}_{tag}", traj=tr, traj_type=kind, duration_s=tr.duration + 0.3,
                  payload_kg=kg, mass_scale=1.0, noise_seed=None)
    log = runner.run(sc)
    m = metrics(tr, log, kg); m.update({"style": kind, "run_id": f"{sc.name}_{int(HZ)}hz"})
    return m


# Every style is run over a grid of speeds so that cycle time can be read against peak torque, which
# is the proposal's "plot the three metrics against each other to find the best balance". The knob is
# the acceleration scale for the two limit-based styles and the leg duration for the quintic.
GRID = {"trapezoidal": [dict(ascale=a) for a in (1.0, 0.6, 0.4, 0.25)],
        "s_curve": [dict(ascale=a) for a in (1.0, 0.6, 0.4, 0.25)],
        "torque_aware": [dict(T=t) for t in (0.45, 0.6, 0.8, 1.1, 1.5)]}
rows = []
print(f"\n  {'style':<14}{'payload':>8}{'knob':>10}{'cycle s':>9}{'peak torque':>13}{'energy J':>10}{'back-driven J':>15}{'track err':>11}", flush=True)
for kg in (0.0, 2.0):
    tag = f"{kg:g}kg".replace(".", "p")
    for kind, grid in GRID.items():
        for i, kw in enumerate(grid):
            tr = T.cycle(Q0, Q1, kind, **kw)
            r = run(kind, tr, kg, f"{tag}_{i}")
            r["knob"] = kw
            rows.append(r)
            knob = " ".join(f"{k}={v:g}" for k, v in kw.items())
            print(f"  {kind:<14}{kg:>7g}k{knob:>10}{r['cycle_time_s']:>9.2f}{r['peak_torque_frac']:>12.0%}{r['energy_drawn_J']:>10.1f}"
                  f"{r['energy_backdriven_J']:>15.1f}{r['track_err_max_rad']:>11.4f}", flush=True)

print(f"\n  shortest cycle time that keeps peak torque within a budget (fraction of datasheet limits):", flush=True)
budgets = (0.40, 0.55, 0.70)
best = []
print(f"  {'payload':>8}{'budget':>8}" + "".join(f"{k:>16}" for k in GRID), flush=True)
for kg in (0.0, 2.0):
    for b in budgets:
        line, rec = f"  {kg:>7g}k{b:>8.0%}", {"payload_kg": kg, "budget": b}
        for kind in GRID:
            ok = [r for r in rows if r["style"] == kind and r["payload_kg"] == kg and r["peak_torque_frac"] <= b]
            pick = min(ok, key=lambda r: r["cycle_time_s"]) if ok else None
            rec[kind] = pick["cycle_time_s"] if pick else None
            line += f"{(format(pick['cycle_time_s'], '.2f') + ' s') if pick else 'none':>16}"
        best.append(rec); print(line, flush=True)
json.dump({"q0": Q0.tolist(), "q1": Q1.tolist(), "j_max_rad_s3": float(T.J_MAX[0]), "a_max_rad_s2": float(T.A_MAX[0]),
           "runs": rows, "shortest_under_budget": best}, open(f"{OUT}/summary.json", "w"), indent=2)
print(f"\n  written out/o6/summary.json ({len(rows)} runs)", flush=True)
simulation_app.close()
