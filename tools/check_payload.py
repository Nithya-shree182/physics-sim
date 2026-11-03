#!/usr/bin/env python3
"""O4 data check: does the plant deliver the physically correct torque for a known payload?

    .venv-tools/bin/python tools/check_payload.py out/o4

For each held pose, the 0 kg run is the tare. The change in mean applied torque is projected onto
the gravity-torque-per-kg pattern computed here from the URDF with numpy (pal_kinematics, no PhysX,
no Pinocchio):  m_est = (dtau . dg) / (dg . dg).  The estimate is compared with the known mass.

This scores the DATA (is the payload physically right?), not the analysis side's estimator, which
should be scored the same way on the same files. Pass condition, fixed in advance: every held run
within 3% of the known mass or 0.02 kg, whichever is larger.
"""
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "workspace"))
from pal_kinematics import Urdf  # noqa: E402

ARM = 7
URDF = ROOT / "assets/franka_panda/panda_arm_hand.urdf"


def main(out):
    out = Path(out)
    runs = json.loads((out / "index.json").read_text())["runs"]
    load = lambda rid: pd.read_parquet(out / f"{rid}.parquet")
    tau = lambda d, lo: np.array([d[f"tau_applied_{i}"].to_numpy()[lo:].mean() for i in range(ARM)])
    q = lambda d, lo: np.array([d[f"q_{i}"].to_numpy()[lo:].mean() for i in range(ARM)])
    print(f"  {'pose':<18}{'known kg':>9}{'estimated kg':>14}{'error kg':>10}{'error %':>9}")
    ok, n = True, 0
    for pose in sorted({r["pose"] for r in runs if r["traj_type"] == "hold"}):
        pr = [r for r in runs if r["pose"] == pose]
        tare = next(r for r in pr if r["payload_kg"] == 0.0)
        d0 = load(tare["run_id"]); lo = int(len(d0) * 0.5)
        t0, q0 = tau(d0, lo), q(d0, lo)
        finger = float(d0["finger_pos_0"].mean())
        scale = json.loads((out / f"{tare['run_id']}.json").read_text()).get("plant_link_mass_scale", 1.0)
        dg = (Urdf(URDF, mass_scale=scale, payload_kg=1.0).gravity(q0, finger)
              - Urdf(URDF, mass_scale=scale).gravity(q0, finger))
        for r in pr:
            if r["payload_kg"] == 0.0:
                continue
            d = load(r["run_id"])
            est = float((tau(d, lo) - t0) @ dg / (dg @ dg))
            err = est - r["payload_kg"]
            good = abs(err) <= max(0.03 * r["payload_kg"], 0.02)
            ok &= good; n += 1
            print(f"  {pose:<18}{r['payload_kg']:>9g}{est:>14.4f}{err:>+10.4f}{100 * err / r['payload_kg']:>8.2f}%"
                  f"{'' if good else '   <-- outside tolerance'}")
    print(f"\n  {n} held runs scored")
    print("PAYLOAD DATA", "PASS" if ok and n else "FAIL")
    return 0 if ok and n else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "out/o4"))
