"""Gravity check for the independent Pinocchio model.

Two parts.

1. Self-tests that need no simulator. They check that the URDF was parsed and the model was
   built correctly:
     - moving mass equals the sum of every link <mass> in the URDF except the fixed base
       (parsed separately from the XML)
     - g(q) equals the numerical gradient of potential energy, dU/dq
     - M(q) is symmetric and positive definite
     - rnea(q, 0, 0) equals g(q)
     - model.armature reaches both M(q) and rnea
     - friction is zero at rest, and forward dynamics exactly inverts tau_model
     - torque limits from the URDF are [87 x4, 12 x3] N*m

2. Comparison with the simulator's g(q), optional. At rest (qd = qdd = 0) the torque a joint
   must supply is g(q), so Isaac Sim's get_generalized_gravity_forces() should match this
   model. Pass dev's numbers with --expect, using 1-based joint numbers:

       python model/gravity_check.py --pose home --expect 2=-1.6548,4=20.0771,6=0.8557

   Error is reported in N*m and as a percentage of that joint's torque limit. Per
   WORK_SPLIT.md the tolerance is normalised by torque limit, not by the joint's own value,
   because joints 1, 3, 5 and 7 carry almost no gravity load.

   The check also tries the opposite sign, so a flipped sign convention is reported as such
   rather than as a modelling error.

Exit code is 0 only if every self-test passes (and any --expect comparison is inside --tol).
"""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from pinocchio_model import DEFAULT_URDF, PandaModel, load_panda  # noqa: E402
from poses import PROBE_POSES  # noqa: E402

EXPECTED_EFFORT = np.array([87, 87, 87, 87, 12, 12, 12], dtype=float)


def urdf_moving_mass(urdf: Path, base_link: str = "panda_link0") -> float:
    """Sum of every link mass in the URDF except the base, which is fixed to the world and so
    is not part of the articulated model."""
    root = ET.parse(urdf).getroot()
    return sum(
        float(link.find("inertial/mass").get("value"))
        for link in root.iter("link")
        if link.get("name") != base_link and link.find("inertial/mass") is not None
    )


def numerical_gravity(p: PandaModel, q: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """dU/dq by central differences. Independent of Pinocchio's own gravity routine."""
    grad = np.zeros(p.nq)
    for i in range(p.nq):
        dq = np.zeros(p.nq)
        dq[i] = eps
        grad[i] = (p.potential_energy(q + dq) - p.potential_energy(q - dq)) / (2 * eps)
    return grad


def run_self_tests(p: PandaModel, urdf: Path) -> bool:
    ok = True

    def report(name: str, passed: bool, detail: str) -> None:
        nonlocal ok
        ok &= passed
        print(f"  [{'PASS' if passed else 'FAIL'}] {name}: {detail}")

    print("Self-tests (no simulator involved)")

    m_urdf = urdf_moving_mass(urdf)
    report("moving mass", abs(p.total_mass - m_urdf) < 1e-6,
           f"model {p.total_mass:.6f} kg, URDF sum without base {m_urdf:.6f} kg")

    report("torque limits", np.allclose(p.effort_limit, EXPECTED_EFFORT),
           f"{p.effort_limit.tolist()}")

    worst_grad, worst_rnea, worst_sym, min_eig = 0.0, 0.0, 0.0, np.inf
    rng = np.random.default_rng(0)
    qs = list(PROBE_POSES.values()) + [rng.uniform(-1.5, 1.5, 7) for _ in range(20)]
    for q in qs:
        g = p.g(q)
        worst_grad = max(worst_grad, np.max(np.abs(g - numerical_gravity(p, q))))
        worst_rnea = max(worst_rnea, np.max(np.abs(p.tau_model(q, np.zeros(7), np.zeros(7)) - g)))
        M = p.M(q)
        worst_sym = max(worst_sym, np.max(np.abs(M - M.T)))
        min_eig = min(min_eig, np.linalg.eigvalsh(M).min())

    # Armature must reach both M(q) and rnea, or the residual gets an acceleration-proportional
    # error that looks like a payload. Compare against the same URDF loaded with zero armature.
    bare = load_panda(urdf, armature=0.0)
    q, qdd = PROBE_POSES["home"], rng.uniform(-2.0, 2.0, 7)
    dM = np.diag(p.M(q) - bare.M(q))
    dtau = p.tau_model(q, np.zeros(7), qdd) - bare.tau_model(q, np.zeros(7), qdd)
    report("armature in M(q)", np.allclose(dM, p.armature, atol=1e-12),
           f"diagonal increase {dM.round(6).tolist()}")
    report("armature in rnea", np.allclose(dtau, p.armature * qdd, atol=1e-12),
           "torque increase equals armature * qdd" if np.allclose(dtau, p.armature * qdd, atol=1e-12)
           else f"torque increase {dtau.round(6).tolist()}, expected {(p.armature * qdd).round(6).tolist()}")

    report("friction zero at rest", np.allclose(p.friction(np.zeros(7)), 0.0, atol=1e-12),
           f"tau_f(0) = {p.friction(np.zeros(7)).round(12).tolist()}")

    # The synthetic generator integrates forward dynamics and the residual uses inverse
    # dynamics; they must be exact inverses, armature and friction included.
    worst_inv = 0.0
    for _ in range(20):
        q, qd, tau = rng.uniform(-1.5, 1.5, 7), rng.uniform(-2.0, 2.0, 7), rng.uniform(-10, 10, 7)
        worst_inv = max(worst_inv, np.max(np.abs(p.tau_model(q, qd, p.forward_dynamics(q, qd, tau)) - tau)))
    report("forward dynamics inverts tau_model", worst_inv < 1e-9, f"worst difference {worst_inv:.2e} N*m")

    report("g(q) = dU/dq", worst_grad < 1e-4, f"worst difference {worst_grad:.2e} N*m over {len(qs)} poses")
    report("rnea(q,0,0) = g(q)", worst_rnea < 1e-9, f"worst difference {worst_rnea:.2e} N*m")
    report("M symmetric", worst_sym < 1e-12, f"worst asymmetry {worst_sym:.2e}")
    report("M positive definite", min_eig > 0, f"smallest eigenvalue {min_eig:.4f} kg*m^2")
    return ok


def parse_expect(text: str) -> dict[int, float]:
    out = {}
    for item in text.split(","):
        k, v = item.split("=")
        k = int(k)
        if not 1 <= k <= 7:
            raise ValueError(f"joint number must be 1-7, got {k}")
        out[k - 1] = float(v)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    ap.add_argument("--pose", default="home", choices=sorted(PROBE_POSES), help="named pose for the comparison")
    ap.add_argument("--expect", type=str, default=None,
                    help="simulator g(q) values as 'joint=value,...' (1-based joints), e.g. 2=-1.6548,4=20.0771")
    ap.add_argument("--tol", type=float, default=0.05,
                    help="pass if error / torque limit is below this (default 0.05 = 5 percent)")
    ap.add_argument("--armature", type=float, default=0.0061)
    args = ap.parse_args()

    p = load_panda(args.urdf, armature=args.armature)
    print(f"Model: {p.nq} DOF, joints {p.joint_names[0]}..{p.joint_names[-1]}, "
          f"total mass {p.total_mass:.4f} kg, armature {args.armature} kg*m^2\n")

    ok = run_self_tests(p, args.urdf)

    print("\ng(q) at the named poses (N*m, torque needed to hold the arm still)")
    header = "pose".ljust(24) + "".join(f"  j{i + 1:<6}" for i in range(7))
    print(header)
    for name, q in PROBE_POSES.items():
        print(name.ljust(24) + "".join(f"{v:9.4f}" for v in p.g(q)))

    if args.expect:
        q = PROBE_POSES[args.pose]
        g = p.g(q)
        exp = parse_expect(args.expect)
        print(f"\nComparison with simulator values at pose '{args.pose}'")
        print("joint   simulator    model     error   err/limit   | model with sign flipped: error")
        within = True
        flip_better = 0
        for j, sim in sorted(exp.items()):
            err = g[j] - sim
            err_flip = -g[j] - sim
            pct = abs(err) / p.effort_limit[j]
            within &= pct < args.tol
            flip_better += abs(err_flip) < abs(err)
            print(f"  {j + 1}    {sim:9.4f} {g[j]:9.4f} {err:9.4f}   {100 * pct:6.2f}%   |   {err_flip:9.4f}")
        if within:
            print(f"\nMATCH: every compared joint is within {100 * args.tol:.0f}% of its torque limit.")
        elif flip_better == len(exp):
            print("\nNO MATCH, but the opposite sign fits better on every joint: likely a sign "
                  "convention difference between the simulator's g(q) and holding torque.")
        else:
            print("\nNO MATCH. Check, in this order: is the simulator pose the one used here "
                  f"('{args.pose}')? are the fingers/hand mass and armature the same? is this the "
                  "same URDF as the asset?")
        ok &= within

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
