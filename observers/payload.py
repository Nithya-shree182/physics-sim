"""Payload estimation (O4): the mass of an unknown held object, from the residual alone.

The residual's external wrench W_ext (observers/residual.py) is the force the payload exerts
on the hand. For a payload of mass m whose centre is at point p,

    F = m (g_vec - a_p),     g_vec = (0, 0, -9.81),  a_p = acceleration of p

so each valid row gives an estimate  m_k = F . d / (d . d)  with d = g_vec - a_p. At rest this
is simply -F_z / 9.81; in motion the acceleration term keeps it unbiased.

Rows are only used when the pose passes the payload gate (observers/confidence.py,
vertical_noise_gain); otherwise the estimate is held ("paused, not wrong", proposal section 8).
The running estimate is an exponentially weighted mean with time constant TAU_S, and the band
is two standard errors of that mean.

WAITING ON DEV: where the payload actually sits. PAYLOAD_OFFSET_M (0.1034 m along panda_hand z,
the Franka TCP) only matters while the arm accelerates; at rest the mass does not depend on it.

    python observers/payload.py data/synthetic/synth_hold_1kg_none_s0.parquet
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent / "model")]
from confidence import VERTICAL_GAIN_MAX, conditioning  # noqa: E402
from pinocchio_model import PandaModel, load_panda  # noqa: E402
from residual import DEFAULT_CUTOFF_HZ, Residual, compute_residual, load_run  # noqa: E402

GRAVITY_VEC = np.array([0.0, 0.0, -9.81])
PAYLOAD_OFFSET_M = (0.0, 0.0, 0.1034)   # in panda_hand frame; the Franka TCP. Ask dev (NOTES_FOR_DEV 6)
TAU_S = 1.0                             # [CHOICE] time constant of the running mean, s
MIN_D_NORM = 2.0                        # [CHOICE] skip rows in near free fall (|g - a| < 2 m/s^2)


@dataclass
class PayloadEstimate:
    t: np.ndarray
    m_raw: np.ndarray        # per-row estimate, NaN where unusable
    m_est: np.ndarray        # running estimate, held while gated
    m_band: np.ndarray       # two standard errors of the running mean
    gated: np.ndarray        # True where the pose failed the payload gate

    @property
    def final(self) -> tuple[float, float]:
        return float(self.m_est[-1]), float(self.m_band[-1])

    def to_frame(self) -> pd.DataFrame:
        return pd.DataFrame({"t": self.t, "m_raw": self.m_raw, "m_est": self.m_est,
                             "m_band": self.m_band, "gated": self.gated})


def estimate_payload(res: Residual, model: PandaModel, offset_local=PAYLOAD_OFFSET_M,
                     tau_s: float = TAU_S, tare_kg: float = 0.0,
                     vertical_gain_max: float = VERTICAL_GAIN_MAX) -> PayloadEstimate:
    """Run the estimator over a computed residual. tare_kg is subtracted from every row
    (e.g. the phantom mass measured with an empty gripper)."""
    if res.wrench is None:
        raise ValueError("residual was computed without the wrench")
    n = len(res.t)
    dt = float(np.median(np.diff(res.t)))
    alpha = min(1.0, dt / tau_s)

    m_raw = np.full(n, np.nan)
    gated = np.zeros(n, dtype=bool)
    for k in np.flatnonzero(res.valid):
        if conditioning(res.J[k])["vertical_noise_gain"] > vertical_gain_max:
            gated[k] = True
            continue
        d = GRAVITY_VEC - model.point_acceleration(res.q[k], res.qd[k], res.qdd[k], offset_local)
        if np.linalg.norm(d) < MIN_D_NORM:
            continue
        m_raw[k] = float(res.wrench[k, :3] @ d / (d @ d)) - tare_kg

    m_est = np.full(n, np.nan)
    m_band = np.full(n, np.nan)
    mean = var = None
    n_used = 0
    for k in range(n):
        x = m_raw[k]
        if np.isfinite(x):
            n_used += 1
            if mean is None:
                mean, var = x, 0.0
            else:
                # plain running average until the window is full, then exponential: the first
                # sample does not get a seed weight it has not earned
                w = max(alpha, 1.0 / n_used)
                diff = x - mean
                mean += w * diff
                var = (1 - w) * (var + w * diff * diff)
        if mean is not None:
            n_eff = min(n_used, 2.0 / alpha - 1.0)     # effective samples in the window
            m_est[k] = mean
            m_band[k] = 2.0 * np.sqrt(var / max(n_eff, 1.0))
    return PayloadEstimate(res.t, m_raw, m_est, m_band, gated)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("parquet", type=Path)
    ap.add_argument("--cutoff", type=float, default=DEFAULT_CUTOFF_HZ)
    ap.add_argument("--tare", type=float, default=0.0, help="kg subtracted from every estimate")
    ap.add_argument("--out", type=Path, default=None, help="write the estimate trace as parquet")
    args = ap.parse_args()

    run = load_run(args.parquet)
    model = load_panda()
    est = estimate_payload(compute_residual(run, model, cutoff_hz=args.cutoff), model, tare_kg=args.tare)
    m, band = est.final
    truth = float(run.df["payload_kg"].iloc[0]) if "payload_kg" in run.df else float("nan")
    print(f"{run.name}: estimate {m:.4f} ± {band:.4f} kg, label {truth:g} kg, "
          f"error {m - truth:+.4f} kg, gated {100 * est.gated.mean():.1f}% of rows")
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        est.to_frame().to_parquet(args.out, index=False)
        print(f"  written {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
