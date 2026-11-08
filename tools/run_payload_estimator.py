#!/usr/bin/env python3
"""Score the analysis side's payload estimator (observers/payload.py) on our known-mass runs (out/o4).

    .venv-pal/bin/python tools/run_payload_estimator.py [out/o4]

For each pose, the 0 kg run is the tare (it carries the plant's 5% mass mismatch and the empty gripper, which the model does not
know about). The estimate for each loaded run is compared with the known mass. Pass condition, the same one used on the data by
tools/check_payload.py and by the analysis side's own tests: within 3% of the known mass or 0.02 kg, whichever is larger.
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "observers"), str(ROOT / "model")]
from payload import estimate_payload              # noqa: E402
from pinocchio_model import load_robot            # noqa: E402
from residual import compute_residual, load_run   # noqa: E402


def main(out):
    out = Path(out)
    runs = json.loads((out / "index.json").read_text())["runs"]
    model = load_robot()
    est = lambda rid, tare=0.0: estimate_payload(compute_residual(load_run(out / f"{rid}.parquet"), model), model, tare_kg=tare).final[0]
    print(f"  {'run':<30}{'known kg':>9}{'raw est kg':>12}{'tared est kg':>14}{'error kg':>10}{'error %':>9}")
    ok, n = True, 0
    for pose in sorted({r["pose"] for r in runs}):
        pr = [r for r in runs if r["pose"] == pose]
        tare_run = next(r for r in pr if r["payload_kg"] == 0.0)
        m0 = est(tare_run["run_id"])
        for r in pr:
            if r["payload_kg"] == 0.0:
                continue
            raw = est(r["run_id"]); m = est(r["run_id"], tare=m0); err = m - r["payload_kg"]
            good = abs(err) <= max(0.03 * r["payload_kg"], 0.02)
            ok &= good; n += 1
            print(f"  {r['run_id'].replace('_1000hz', ''):<30}{r['payload_kg']:>9g}{raw:>12.3f}{m:>14.3f}{err:>+10.3f}{100 * err / r['payload_kg']:>8.1f}%"
                  f"{'' if good else '   <-- outside tolerance'}")
    print(f"\n  {n} loaded runs scored (held at three poses, and moving)")
    print("PAYLOAD ESTIMATOR", "PASS" if ok and n else "FAIL")
    return 0 if ok and n else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "out/o4"))
