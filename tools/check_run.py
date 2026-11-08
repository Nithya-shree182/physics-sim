#!/usr/bin/env python3
"""Sanity-check a captured telemetry run, independently of the simulator's own bookkeeping.

    .venv-tools/bin/python tools/check_run.py data/slow_reference_0kg_1000hz.parquet

Four checks, each with its tolerance stated in the output rather than buried:

 1. SCHEMA     schema/validate.py exits 0.
 2. GRAVITY    g(q) the simulator reports, against g(q) computed here from the URDF with plain
               numpy forward kinematics (no PhysX, no Pinocchio) at the LOGGED q. This catches
               a plant whose masses/COMs do not match the URDF the analysis side models.
 3. IDENTITY   the proposal's O2 test on simulator terms: tau_measured vs M qdd + C + g,
               worst joint as % of the joint's torque limit, against the 5% pass condition
               (proposal section 9). Simulator terms only, so it tests the plant, not the
               analysis side's model.

A 3b check (effort-mode runs only) bounds tau_applied - (M qdd + C + g) on healthy rows by the
plant's configured friction, so a friction-units regression cannot pass.

A --urdf option points check 2 at a different URDF; running it against a copy with one link
mass raised 15% makes check 2 FAIL (0.56 N*m on joint 4), which is the proof the check can fail.

Exit code 0 only if every check passes. Nothing here is a stored result: it recomputes from
the file each time.
"""
import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "workspace"))
import pal_robot  # noqa: E402
from pal_kinematics import Urdf  # noqa: E402
URDF = Path(pal_robot.active().urdf_file())   # whatever arm is loaded
DOF = pal_robot.active().dof
TAU_MAX = np.asarray(pal_robot.active().tau_max, dtype=float)  # active arm's torque limits; never hard-code the old Panda layout
GRAVITY_TOL_NM = 0.15        # [CHOICE] abs N*m; sag of the stiff drive explains ~0.1 on loaded joints
IDENTITY_TOL_PCT = 5.0       # proposal section 9, O2
ARM = DOF


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("parquet", type=Path)
    ap.add_argument("--urdf", type=Path, default=URDF, help="URDF for the independent gravity check")
    ap.add_argument("--samples", type=int, default=40, help="rows used for the URDF gravity check")
    a = ap.parse_args()
    ok = True

    r = subprocess.run([sys.executable, str(ROOT / "schema/validate.py"), str(a.parquet)],
                       capture_output=True, text=True)
    passed = r.returncode == 0
    ok &= passed
    print(f"1 SCHEMA    {'PASS' if passed else 'FAIL'}   (schema/validate.py exit {r.returncode})")
    if not passed:
        print(r.stdout[-600:], r.stderr[-300:])

    d = pd.read_parquet(a.parquet)
    n = len(d)
    arr = lambda p: np.stack([d[f"{p}_{i}"].to_numpy() for i in range(ARM)], axis=1)
    q, qd, tm, g_sim, c = arr("q"), arr("qd"), arr("tau_measured"), arr("g"), arr("coriolis")
    M = np.stack([[d[f"M_{r_}_{c_}"].to_numpy() for c_ in range(ARM)] for r_ in range(ARM)], axis=0)  # (DOF, DOF, n)
    dt = float(d["dt"].iloc[0])
    finger = 0.0
    if "finger_pos_0" in d.columns:
        finger = float(d["finger_pos_0"].mean())

    import json
    man = json.loads(a.parquet.with_suffix('.json').read_text())
    mass_scale = float(man.get('plant_link_mass_scale', 1.0))
    payload_col = d["payload_kg"].to_numpy(dtype=float)           # per row: a pickup changes it mid-run
    if man.get("payload_in_dynamics_terms", True) is False:
        # a physical cube is a separate body: the articulation's g and M do not contain it, its weight arrives as a contact load
        loaded_rows = payload_col > 0
        payload_col = np.zeros_like(payload_col)
    else:
        loaded_rows = np.zeros(len(payload_col), bool)
    payload = float(payload_col[-1])
    urdfs = {}
    def urdf_for(kg):
        if kg not in urdfs:
            urdfs[kg] = Urdf(a.urdf, mass_scale=mass_scale, payload_kg=kg)
        return urdfs[kg]
    rows = np.linspace(int(0.1 * n), n - 1, a.samples).astype(int)
    g_urdf = np.array([urdf_for(float(payload_col[k])).gravity(q[k], finger) for k in rows])
    err = np.abs(g_urdf - g_sim[rows])
    worst = err.max(axis=0)
    passed = bool((worst < GRAVITY_TOL_NM).all())
    ok &= passed
    print(f"2 GRAVITY   {'PASS' if passed else 'FAIL'}   URDF-numpy (arm mass scale {mass_scale:g}, payload {'0 to ' if payload_col.min() != payload_col.max() else ''}{payload:g} kg) vs simulator g(q), worst abs error per joint "
          f"[N*m]: {np.round(worst, 3).tolist()}  (tolerance {GRAVITY_TOL_NM})")

    traj_type = str(d["traj_type"].iloc[0])
    fast = traj_type in ("trapezoidal", "s_curve", "torque_aware")        # the O6 styles run above the 50% regime; "pick" is kept under ~40%
    lo = int(0.1 * n)
    noisy = "sensor_noise" in man
    if noisy:                                    # zero-phase 20 ms boxcar, as the analysis side filters
        w = max(1, round(0.020 / dt))      # a 20 ms boxcar at any physics rate (a fixed 20 rows smeared 80 ms at 250 Hz)
        qd_s = np.stack([np.convolve(qd[:, j], np.ones(w) / w, mode="same") for j in range(ARM)], axis=1)
    else:
        qd_s = qd
    qdd = np.vstack([np.zeros((1, ARM)), np.diff(qd_s, axis=0) / dt])      # backward difference
    net = np.array([M[:, :, k] @ qdd[k] + c[k] + g_sim[k] for k in range(n)])
    hi = n - (w if noisy else 0)
    healthy_rows = ~d["fault_active"].to_numpy(dtype=bool) & ~loaded_rows      # rows carrying a physical load are an external force
    mask = np.zeros(n, bool); mask[lo:hi] = True; mask &= healthy_rows
    res = (tm - net)[mask]
    rms_pct = 100 * np.sqrt((res ** 2).mean(axis=0)) / TAU_MAX
    sig_rms = np.sqrt((tm[mask] ** 2).mean(axis=0))
    rel_pct = 100 * np.sqrt((res ** 2).mean(axis=0)) / np.maximum(sig_rms, 1e-9)    # % of the joint's own torque
    passed = bool((rms_pct < IDENTITY_TOL_PCT).all())
    skip_identity = fast or traj_type == "push"       # push: a known external force acts on the arm
    ok &= passed if not skip_identity else True
    print(f"3 IDENTITY  {('PASS' if passed else 'FAIL') if not skip_identity else 'n/a  '}   tau_measured vs M qdd + C + g, healthy rows, RMS % of torque limit per "
          f"joint: {np.round(rms_pct, 2).tolist()}  (limit {IDENTITY_TOL_PCT}%)")
    print(f"             same residual as % of each joint's OWN torque (stricter reading of s9): "
          f"{np.round(rel_pct, 2).tolist()}  {'(all within 5%)' if (rel_pct < 5).all() else '(some joints above 5% of their own torque)'}")

    if fast:
        print(f"3b FRICTION n/a     {traj_type} runs are above the 50% speed regime (residual artifact); "
              f"only torque, time and energy are used")
    elif traj_type == "push":
        print("3b FRICTION n/a     push runs carry a known external force; checked by tools/check_push.py")
    elif str(d["drive_mode"].iloc[0]) == "torque":
        # Effort mode: the applied torque minus the dynamics must equal the plant's FRICTION (and
        # any injected fault), never tens of N*m. This is the guard against the 2026-11-03 bug
        # (friction passed in the wrong units gave ~38 N*m on joint 2).
        ta = arr("tau_applied")
        healthy = ~d["fault_active"].to_numpy(dtype=bool)
        sel = mask
        resa = (ta - net)[sel]
        fv = np.array(man["friction_values"])                       # (7, 3) static, dynamic, viscous
        qd_rms = np.sqrt((qd_s[sel] ** 2).mean(axis=0))
        bound = 2.0 * (fv[:, 0] + fv[:, 2] * qd_rms) + 0.15          # N*m, per joint
        rms = np.sqrt((resa ** 2).mean(axis=0))
        passed = bool((rms <= bound).all())
        ok &= passed
        print(f"3b FRICTION {'PASS' if passed else 'FAIL'}   healthy rows: RMS(tau_applied - net) per joint "
              f"[N*m]: {np.round(rms, 3).tolist()}  vs allowed {np.round(bound, 2).tolist()}")

    print("\nOVERALL", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
