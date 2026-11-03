"""Pose confidence: how far each pose can be trusted, from the Jacobian.

The proposal's score c(q) = sigma_min(J) / sigma_max(J) never exceeds ~0.13 on the raw 6x7
Jacobian (dev's measurement), because its linear rows (m per rad) and angular rows (rad per
rad) have different units. Two fixes, both here:

1. Scale the angular rows by a characteristic length L before the SVD,
       J_s = diag(1, 1, 1, L, L, L) J
   so every row is in metres. L = 0.472 m, dev's measured lever arm at the tool centre at home.
   c(q) is then computed on J_s. This is the gate for the anomaly detector (O5).

2. A direction-specific score for the payload estimate (O4). The mass comes from the vertical
   force only, and a singularity can collapse a direction that is not vertical (dev's finding:
   1 kg at the near-singular extended pose was still estimated to 0.06%). With
   W = -(J_s^T)^+ r, the noise gain of the vertical force is g_z = |e_z^T (J_s^T)^+|, and
       c_v(q) = 1 / (g_z * sigma_max(J_s))
   lies between c(q) (vertical is the weakest direction) and 1 (vertical is the strongest).

Gates are physical, not percentiles. A pose is trusted when a torque error is not amplified
too much into a force error at the hand: force_noise_gain = 1/sigma_min(J_s) for the
detector, vertical_noise_gain = g_z for the payload estimate, both in N of force per N*m of
torque error. A workspace percentile was tried first and rejected: random poses inside the
joint limits include many near-singular ones, so the 5th percentile (c = 0.0069) let the
elbow-singular probe pose (c = 0.034) through.

Even scaled, c tops out near 0.22 (99th percentile of the sweep), so "near 1 means healthy"
does not happen on a 7-DOF arm. For display, c_norm and c_vertical_norm divide by that
99th-percentile reference and clip to 1.

    python observers/confidence.py --sweep 5000
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "model"))
from pinocchio_model import PandaModel, load_panda  # noqa: E402
from poses import PROBE_POSES, Q_MAX, Q_MIN  # noqa: E402

LENGTH_SCALE_M = 0.472          # dev's measured lever arm at the TCP, home pose

# [CHOICE] Gates: at most 10 N of force error per N*m of torque error, so a 0.1 N*m model or
# sensor error costs at most 1 N (about 100 g of apparent payload). Home is 4.8; the elbow-
# singular probe pose is 26.7.
FORCE_GAIN_MAX = 10.0           # detector gate, on 1/sigma_min(J_s)
VERTICAL_GAIN_MAX = 10.0        # payload gate, on g_z

# References for the 0-1 display scores: 99th percentile of `--sweep 5000 --seed 0`
# (2026-11-03). Rerun the sweep and update if LENGTH_SCALE_M or the model changes.
C_REF = 0.2239
C_V_REF = 0.4070


def scaled_jacobian(J: np.ndarray, length: float = LENGTH_SCALE_M) -> np.ndarray:
    return np.diag([1.0, 1.0, 1.0, length, length, length]) @ J


def conditioning(J: np.ndarray, length: float = LENGTH_SCALE_M) -> dict:
    """Scores for one 6x7 Jacobian (world-aligned, linear rows first)."""
    Js = scaled_jacobian(J, length)
    U, s, _ = np.linalg.svd(Js, full_matrices=False)
    s_min, s_max = float(s[-1]), float(s[0])
    # |e_z^T (Js^T)^+| = |Sigma^-1 U^T e_z|
    g_z = float(np.linalg.norm(U[2, :] / s))
    c, c_v = s_min / s_max, 1.0 / (g_z * s_max)
    return {
        "sigma_min": s_min,
        "sigma_max": s_max,
        "c": c,
        "c_vertical": c_v,
        "c_norm": min(1.0, c / C_REF),
        "c_vertical_norm": min(1.0, c_v / C_V_REF),
        "manipulability": float(np.prod(s)),
        "force_noise_gain": 1.0 / s_min,       # worst-case N of force per N*m of torque error
        "vertical_noise_gain": g_z,            # N of vertical force per N*m of torque error
    }


def raw_conditioning(J: np.ndarray) -> dict:
    """Unscaled scores, as in the proposal, for comparison with dev's table."""
    s = np.linalg.svd(J, compute_uv=False)
    return {"sigma_min": float(s[-1]), "sigma_max": float(s[0]), "c": float(s[-1] / s[0]),
            "manipulability": float(np.prod(s))}


def pose_confidence(model: PandaModel, q: np.ndarray) -> dict:
    return conditioning(model.jacobian(q))


def gates(scores: dict, force_gain_max: float = FORCE_GAIN_MAX,
          vertical_gain_max: float = VERTICAL_GAIN_MAX) -> dict:
    """Which downstream estimates may be trusted at this pose."""
    return {"detector_ok": scores["force_noise_gain"] <= force_gain_max,
            "payload_ok": scores["vertical_noise_gain"] <= vertical_gain_max}


def sweep(model: PandaModel, n: int = 5000, seed: int = 0) -> dict:
    """Score distribution over n poses sampled uniformly inside the joint limits."""
    rng = np.random.default_rng(seed)
    qs = rng.uniform(Q_MIN, Q_MAX, size=(n, 7))          # joint 4 range is all negative, as it must be
    keys = ("c", "c_vertical", "force_noise_gain", "vertical_noise_gain")
    vals = {k: np.empty(n) for k in keys}
    det_ok = np.empty(n, dtype=bool)
    pay_ok = np.empty(n, dtype=bool)
    for i, q in enumerate(qs):
        s = pose_confidence(model, q)
        for k in keys:
            vals[k][i] = s[k]
        g = gates(s)
        det_ok[i], pay_ok[i] = g["detector_ok"], g["payload_ok"]
    pct = [1, 5, 10, 25, 50, 75, 90, 99]
    out = {"n": n, "seed": seed, "length_scale_m": LENGTH_SCALE_M,
           "force_gain_max": FORCE_GAIN_MAX, "vertical_gain_max": VERTICAL_GAIN_MAX,
           "detector_gated_fraction": round(float(1 - det_ok.mean()), 4),
           "payload_gated_fraction": round(float(1 - pay_ok.mean()), 4)}
    for k in keys:
        out[f"{k}_percentiles"] = dict(zip(map(str, pct), np.percentile(vals[k], pct).round(4).tolist()))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sweep", type=int, default=0, help="number of random poses to sweep (0: skip)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=REPO_ROOT / "data" / "analysis" / "confidence_sweep.json")
    args = ap.parse_args()

    model = load_panda()
    print(f"Named poses, L = {LENGTH_SCALE_M} m. Gates: force gain <= {FORCE_GAIN_MAX}, "
          f"vertical gain <= {VERTICAL_GAIN_MAX} (N per N*m)")
    print(f"{'pose':24s} {'raw c':>7s} {'c':>7s} {'c_norm':>7s} {'cv_norm':>7s} {'F gain':>7s} {'Fz gain':>7s}"
          "  detector  payload")
    for name, q in PROBE_POSES.items():
        J = model.jacobian(q)
        s, raw = conditioning(J), raw_conditioning(J)
        g = gates(s)
        print(f"{name:24s} {raw['c']:7.4f} {s['c']:7.4f} {s['c_norm']:7.3f} {s['c_vertical_norm']:7.3f}"
              f" {s['force_noise_gain']:7.2f} {s['vertical_noise_gain']:7.2f}"
              f"  {'ok' if g['detector_ok'] else 'GATED':8s}  {'ok' if g['payload_ok'] else 'GATED'}")

    if args.sweep:
        res = sweep(model, args.sweep, args.seed)
        print(f"\nSweep of {res['n']} poses (seed {res['seed']}):")
        for k in ("c", "c_vertical", "force_noise_gain", "vertical_noise_gain"):
            print(f"  {k + ' percentiles':32s}: {res[k + '_percentiles']}")
        print(f"  gated by the detector gate : {100 * res['detector_gated_fraction']:.1f}% of poses")
        print(f"  gated by the payload gate  : {100 * res['payload_gated_fraction']:.1f}% of poses")
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(res, indent=2), encoding="utf-8")
        print(f"  written {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
