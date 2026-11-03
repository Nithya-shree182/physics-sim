"""Synthetic telemetry generator: schema-valid logs from a Pinocchio "plant", for building and
testing the analysis side before real Isaac Sim logs exist.

Two copies of the same arm:
  - the PLANT stands in for the simulator. It can carry a payload, perturbed link masses,
    friction scaled by 1.15 (dev's PLANT_FRICTION_SCALE), injected faults and sensor noise.
  - the TWIN is the clean model from pinocchio_model.load_panda(). The controller uses it
    for feedforward, exactly as a real controller only knows the nominal model.

The plant is driven in torque (effort) mode at 1 kHz by a joint impedance controller
(libfranka joint_impedance example gains) with twin feedforward, and integrated with
semi-implicit Euler. Each row is written in the format of schema/telemetry_v0.yaml, plus a
JSON manifest, and the result is checked with schema/validate.py before the run is accepted.

Timing convention, recorded in the manifest: tau at row k is applied during step k and
moves qd[k] to qd[k+1]. A forward difference of qd therefore recovers that step's qdd.

LIMITATION: the plant is built from the same URDF as the twin, so results on these logs are
optimistic. They show the code works; reported numbers must come from Isaac Sim logs.

Examples:
    python model/synth_log.py --traj excitation --duration 20
    python model/synth_log.py --traj slow_reference --payload 1.0
    python model/synth_log.py --traj excitation --fault friction --fault-joint 3 --fault-param 3
    python model/synth_log.py --batch                 # standard set into data/synthetic/
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import platform
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pinocchio as pin

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
sys.path[:0] = [str(HERE), str(REPO_ROOT / "schema")]
from pinocchio_model import (  # noqa: E402
    ARMATURE_DEFAULT, DEFAULT_URDF, FINGER_OPEN, FRICTION_PHI1, FRICTION_PHI2, FRICTION_PHI3,
    GRAVITY, PandaModel, load_panda,
)
from poses import HOME_JOINTS, Q_MAX, Q_MIN  # noqa: E402
from validate import load_schema, required_fields, validate_frame, validate_manifest  # noqa: E402

PHYSICS_HZ = 1000.0
QD_MAX = np.array([2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61])       # rad/s, datasheet

# libfranka examples/joint_impedance_control gains, the per-joint set dev's DRIVE_MODES uses.
KP = np.array([600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0])
KD = np.array([50.0, 50.0, 50.0, 20.0, 20.0, 20.0, 10.0])

PLANT_FRICTION_SCALE = 1.15     # dev's pal_config.PLANT_FRICTION_SCALE
MASS_MISMATCH = 0.03            # [CHOICE] each link mass and inertia scaled by U(1-x, 1+x)
PAYLOAD_TCP_OFFSET = 0.1034     # m along panda_hand z: the Franka hand's TCP (fingertip centre)
PAYLOAD_RADIUS = 0.025          # m, payload modelled as a solid sphere (fits the 50 mm cube)

# [CHOICE] sensor noise, standard deviations. Not from a datasheet; recorded per run.
NOISE_Q = 2e-5                  # rad
NOISE_QD = 2e-3                 # rad/s
NOISE_TAU = 0.05                # N*m

TRAJ_TYPES = ("hold", "slow_reference", "excitation")
FAULT_TYPES = ("none", "friction", "stuck_joint", "delay")
FAULT_DEFAULT_PARAM = {"friction": 3.0, "stuck_joint": float("nan"), "delay": 0.05}


# ── trajectories ─────────────────────────────────────────────────────────────────────────
def make_trajectory(kind: str, t: np.ndarray, seed: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """q, qd, qdd of shape (len(t), 7), analytic, centred on HOME_JOINTS."""
    n = len(t)
    if kind == "hold":
        return np.tile(HOME_JOINTS, (n, 1)), np.zeros((n, 7)), np.zeros((n, 7))

    if kind == "slow_reference":
        amp = np.array([0.3, 0.2, 0.3, 0.3, 0.3, 0.3, 0.3])
        w = 2 * np.pi * 0.1
        phase = np.linspace(0, np.pi, 7)
        s = np.sin(w * t[:, None] + phase)
        c = np.cos(w * t[:, None] + phase)
        return HOME_JOINTS + amp * (s - np.sin(phase)), amp * w * c, -amp * w**2 * s

    if kind == "excitation":
        # Multi-sine in the style of Gaz et al. (5 harmonics of 0.15 Hz), random coefficients,
        # scaled so peak speed stays under 60 percent of the datasheet limit and the pose
        # stays inside the joint limits.
        rng = np.random.default_rng(seed)
        w = 2 * np.pi * 0.15 * np.arange(1, 6)
        a = rng.uniform(-1, 1, (5, 7)) / np.arange(1, 6)[:, None]
        ph = rng.uniform(0, 2 * np.pi, (5, 7))
        arg = w[None, :, None] * t[:, None, None] + ph[None]
        q_rel = np.sum(a[None] * (np.sin(arg) - np.sin(ph)[None]), axis=1)
        qd = np.sum(a[None] * w[None, :, None] * np.cos(arg), axis=1)
        qdd = np.sum(-a[None] * (w**2)[None, :, None] * np.sin(arg), axis=1)
        scale = np.minimum(0.6 * QD_MAX / np.abs(qd).max(axis=0), 0.6 / np.abs(q_rel).max(axis=0))
        q = HOME_JOINTS + scale * q_rel
        if np.any(q < Q_MIN) or np.any(q > Q_MAX):
            raise RuntimeError("excitation trajectory leaves the joint limits; try another seed")
        return q, scale * qd, scale * qdd

    raise ValueError(f"unknown trajectory {kind!r}, choose from {TRAJ_TYPES}")


# ── plant ────────────────────────────────────────────────────────────────────────────────
def build_plant(payload_kg: float, mass_mismatch: float, friction_scale: float,
                rng: np.random.Generator) -> tuple[PandaModel, np.ndarray]:
    plant = load_panda()
    m = plant.model
    scales = 1.0 + rng.uniform(-mass_mismatch, mass_mismatch, 7) if mass_mismatch > 0 else np.ones(7)
    for jid in range(1, m.njoints):
        I = m.inertias[jid]
        m.inertias[jid] = pin.Inertia(I.mass * scales[jid - 1], I.lever, I.inertia * scales[jid - 1])
    if payload_kg > 0:
        frame = m.frames[m.getFrameId("panda_hand")]
        at = frame.placement.act(np.array([0.0, 0.0, PAYLOAD_TCP_OFFSET]))
        sphere = pin.Inertia.FromSphere(payload_kg, PAYLOAD_RADIUS)
        m.inertias[frame.parentJoint] = m.inertias[frame.parentJoint] + pin.SE3(np.eye(3), at).act(sphere)
    plant.data = m.createData()
    plant.friction_scale = np.full(7, friction_scale)
    return plant, scales


# ── one run ──────────────────────────────────────────────────────────────────────────────
def generate(traj: str = "excitation", duration: float = 20.0, payload_kg: float = 0.0,
             fault: str = "none", fault_joint: int = -1, fault_param: float | None = None,
             fault_onset: float | None = None, mismatch: bool = True, noise: bool = True,
             seed: int = 0) -> tuple[pd.DataFrame, dict]:
    """Simulate one run. fault_joint is 0-based (0-6), -1 when fault is none."""
    if fault not in FAULT_TYPES:
        raise ValueError(f"fault must be one of {FAULT_TYPES}")
    if (fault == "none") != (fault_joint == -1):
        raise ValueError("fault_joint must be -1 exactly when fault is none")
    if fault_param is None:
        fault_param = FAULT_DEFAULT_PARAM.get(fault, float("nan"))
    if fault_onset is None:
        fault_onset = duration / 2

    rng = np.random.default_rng(seed)
    dt_s = 1.0 / PHYSICS_HZ
    n = int(round(duration * PHYSICS_HZ))
    t = np.arange(n) * dt_s

    twin = load_panda()
    plant, mass_scales = build_plant(payload_kg, MASS_MISMATCH if mismatch else 0.0,
                                     PLANT_FRICTION_SCALE if mismatch else 1.0, rng)
    base_friction = plant.friction_scale.copy()
    q_cmd, qd_cmd, qdd_cmd = make_trajectory(traj, t, seed)
    delay_steps = int(round(fault_param * PHYSICS_HZ)) if fault == "delay" else 0
    sig = (NOISE_Q, NOISE_QD, NOISE_TAU) if noise else (0.0, 0.0, 0.0)

    cols = {k: np.zeros((n, 7)) for k in ("q", "qd", "tau_applied", "tau_measured", "g", "coriolis")}
    M_log, J_log = np.zeros((n, 7, 7)), np.zeros((n, 6, 7))
    pos_log, quat_log = np.zeros((n, 3)), np.zeros((n, 4))
    fault_active = np.zeros(n, dtype=bool)

    q, qd = q_cmd[0].copy(), qd_cmd[0].copy()
    q_stuck = None
    for k in range(n):
        active = fault != "none" and t[k] >= fault_onset
        fault_active[k] = active

        q_meas = q + rng.normal(0, sig[0], 7)
        qd_meas = qd + rng.normal(0, sig[1], 7)

        # The controller sees the command late when the delay fault is active; the log keeps
        # the command as scheduled, which is what the tracking-error signal compares against.
        kc = max(k - delay_steps, 0) if (fault == "delay" and active) else k
        tau = (twin.tau_model(q_cmd[kc], qd_cmd[kc], qdd_cmd[kc])
               + KP * (q_cmd[kc] - q_meas) + KD * (qd_cmd[kc] - qd_meas))
        tau = np.clip(tau, -plant.effort_limit, plant.effort_limit)

        plant.friction_scale = base_friction.copy()
        if fault == "friction" and active:
            plant.friction_scale[fault_joint] *= fault_param

        cols["q"][k], cols["qd"][k] = q_meas, qd_meas
        cols["tau_applied"][k] = tau
        cols["tau_measured"][k] = tau + rng.normal(0, sig[2], 7)
        cols["g"][k] = plant.g(q)
        cols["coriolis"][k] = plant.coriolis(q, qd)
        M_log[k], J_log[k] = plant.M(q), plant.jacobian(q)
        pos_log[k], quat_log[k] = plant.ee_pose(q)

        if fault == "stuck_joint" and active:
            # The seized joint cannot move: its acceleration is zero and its brake absorbs
            # whatever torque is applied there. The other joints follow the constrained
            # dynamics M_FF qdd_F = (tau - C qd - g - tau_f)_F, which keeps the coupling exact.
            if q_stuck is None:
                q_stuck = q[fault_joint]
                qd[fault_joint] = 0.0
            free = np.arange(7) != fault_joint
            b = tau - plant.tau_model(q, qd, np.zeros(7))
            qdd = np.zeros(7)
            qdd[free] = np.linalg.solve(plant.M(q)[np.ix_(free, free)], b[free])
        else:
            qdd = plant.forward_dynamics(q, qd, tau)
        qd = qd + dt_s * qdd
        q = q + dt_s * qd
        if q_stuck is not None:
            q[fault_joint], qd[fault_joint] = q_stuck, 0.0
        if not np.all(np.isfinite(q)):
            raise RuntimeError(f"simulation diverged at step {k}")

    run_id = f"synth_{traj}_{payload_kg:g}kg_{fault}{'' if fault == 'none' else fault_joint + 1}_s{seed}"
    data = {"t": t, "step": np.arange(n, dtype=np.int64), "dt": np.full(n, dt_s)}
    for name, arr in cols.items():
        data |= {f"{name}_{i}": arr[:, i] for i in range(7)}
    data |= {f"M_{r}_{c}": M_log[:, r, c] for r in range(7) for c in range(7)}
    data |= {f"J_{r}_{c}": J_log[:, r, c] for r in range(6) for c in range(7)}
    data |= {f"ee_pos_{a}": pos_log[:, i] for i, a in enumerate("xyz")}
    data |= {f"ee_quat_{a}": quat_log[:, i] for i, a in enumerate("wxyz")}
    for name, arr in (("q_cmd", q_cmd), ("qd_cmd", qd_cmd), ("qdd_cmd", qdd_cmd)):
        data |= {f"{name}_{i}": arr[:, i] for i in range(7)}
    data |= {"finger_pos_0": np.full(n, FINGER_OPEN), "finger_pos_1": np.full(n, FINGER_OPEN),
             "finger_force_0": np.zeros(n), "finger_force_1": np.zeros(n)}
    data |= {
        "run_id": np.full(n, run_id, dtype=object), "traj_type": np.full(n, traj, dtype=object),
        "drive_mode": np.full(n, "torque", dtype=object), "payload_kg": np.full(n, float(payload_kg)),
        "payload_attached": np.full(n, payload_kg > 0), "fault_type": np.full(n, fault, dtype=object),
        "fault_joint": np.full(n, fault_joint, dtype=np.int8),
        "fault_param": np.full(n, fault_param, dtype=float), "fault_active": fault_active,
        "attack_type": np.full(n, "none", dtype=object), "attack_active": np.zeros(n, dtype=bool),
        "push_force_x": np.full(n, np.nan), "push_force_y": np.full(n, np.nan),
        "push_force_z": np.full(n, np.nan),
    }
    df = pd.DataFrame(data)[list(required_fields(load_schema()))]

    manifest = {
        "schema_version": 0,
        "run_id": run_id,
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds"),
        "git_commit": _git_commit(),
        "machine": f"synthetic ({platform.node()})",
        "isaac_sim_version": "n/a (synthetic, Pinocchio plant)",
        "physics_hz": PHYSICS_HZ,
        "solver": {"type": "pinocchio aba + semi-implicit Euler", "pinocchio": pin.__version__},
        "drive_mode": "torque",
        "asset": {"urdf": DEFAULT_URDF.name, "plant": "Pinocchio model of the same URDF"},
        "ee_frame": "panda_hand",
        "ee_body_index": 7,
        "armature": [ARMATURE_DEFAULT] * 7,
        "friction_source": "gaz_sigmoid",
        "plant_friction_scale": float(base_friction[0]),
        "friction_values": {"phi1": (FRICTION_PHI1 * base_friction).tolist(),
                            "phi2": FRICTION_PHI2.tolist(), "phi3": FRICTION_PHI3.tolist()},
        "torque_convention": "link_side (synthetic: tau_measured = tau_applied + noise)",
        "gravity": list(GRAVITY),
        "seed": seed,
        "payload_kg": float(payload_kg),
        "finger_q_ref": FINGER_OPEN,
        # extras, not required by the schema
        "synthetic": True,
        "timing": "tau at row k moves qd[k] to qd[k+1]",
        "mass_scale_per_link": mass_scales.tolist(),
        "payload_tcp_offset_m": PAYLOAD_TCP_OFFSET,
        "controller": {"type": "joint impedance + twin feedforward", "kp": KP.tolist(), "kd": KD.tolist()},
        "noise_std": {"q": sig[0], "qd": sig[1], "tau": sig[2]},
        "fault": {"type": fault, "joint": fault_joint, "param": fault_param,
                  "onset_s": fault_onset if fault != "none" else None},
    }
    return df, manifest


def _git_commit() -> str:
    try:
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=REPO_ROOT,
                             capture_output=True, text=True, timeout=10)
        return out.stdout.strip() or "unknown"
    except (OSError, subprocess.SubprocessError):
        return "unknown"


def write_run(df: pd.DataFrame, manifest: dict, out_dir: Path) -> bool:
    """Validate, then write <run_id>.parquet and <run_id>.json. Returns True if valid."""
    schema = load_schema()
    errors, warnings = validate_frame(df, schema)
    errors += validate_manifest(manifest, schema, df)
    rid = manifest["run_id"]
    if errors:
        print(f"  {rid}: INVALID, not written")
        for e in errors:
            print(f"    ERROR {e}")
        return False
    out_dir.mkdir(parents=True, exist_ok=True)
    df.to_parquet(out_dir / f"{rid}.parquet", index=False)
    (out_dir / f"{rid}.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    track = np.abs(df[[f"q_cmd_{i}" for i in range(7)]].to_numpy() - df[[f"q_{i}" for i in range(7)]].to_numpy())
    print(f"  {rid}: {len(df)} rows, valid, max tracking error {track.max():.4f} rad"
          + "".join(f"\n    WARNING {w}" for w in warnings))
    return True


BATCH = [
    dict(traj="hold"),
    dict(traj="slow_reference"),
    dict(traj="excitation"),
    dict(traj="hold", payload_kg=1.0),
    dict(traj="slow_reference", payload_kg=1.0),
    dict(traj="excitation", payload_kg=1.0),
    dict(traj="excitation", fault="friction", fault_joint=2),
    dict(traj="excitation", fault="stuck_joint", fault_joint=3),
    # delay affects every joint; the schema needs a joint index for any fault, so 0 is used
    dict(traj="excitation", fault="delay", fault_joint=0),
]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--traj", choices=TRAJ_TYPES, default="excitation")
    ap.add_argument("--duration", type=float, default=20.0, help="seconds (default 20)")
    ap.add_argument("--payload", type=float, default=0.0, help="kg held at the TCP (default 0)")
    ap.add_argument("--fault", choices=FAULT_TYPES, default="none")
    ap.add_argument("--fault-joint", type=int, default=None, help="1-7, required with --fault")
    ap.add_argument("--fault-param", type=float, default=None,
                    help="friction: multiplier (default 3); delay: seconds (default 0.05)")
    ap.add_argument("--fault-onset", type=float, default=None, help="seconds (default: half way)")
    ap.add_argument("--no-mismatch", action="store_true", help="plant identical to the twin")
    ap.add_argument("--no-noise", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=REPO_ROOT / "data" / "synthetic")
    ap.add_argument("--batch", action="store_true", help="write the standard set of runs")
    args = ap.parse_args()

    common = dict(duration=args.duration, mismatch=not args.no_mismatch, noise=not args.no_noise,
                  seed=args.seed)
    if args.batch:
        runs = [common | r for r in BATCH]
        for r in runs:
            r.setdefault("fault_joint", -1)
    else:
        if (args.fault == "none") != (args.fault_joint is None):
            ap.error("--fault-joint (1-7) is required with --fault, and only then")
        runs = [common | dict(traj=args.traj, payload_kg=args.payload, fault=args.fault,
                              fault_joint=-1 if args.fault_joint is None else args.fault_joint - 1,
                              fault_param=args.fault_param, fault_onset=args.fault_onset)]

    print(f"Writing to {args.out}")
    ok = all([write_run(*generate(**r), args.out) for r in runs])
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
