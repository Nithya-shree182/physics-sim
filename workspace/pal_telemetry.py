"""Telemetry writer: one row per physics step, with the arm's whole physical state in it.

Schema v1 (`schema/telemetry_v1.yaml`), written as a compressed .npz with a JSON manifest
beside it and converted to Parquet on the host by tools/npz_to_parquet.py. The Isaac Sim
container has neither pandas nor pyarrow, which is the only reason for the two-step.

WHAT IS IN A ROW, and why each group is there:

  state          q, qd, qdd, and the commanded q/qd/qdd the trajectory asked for. qdd is a
                 backward difference of qd, computed here rather than left to the analysis
                 side, so the row carries the acceleration that actually belongs to its own
                 torque (see TIMING, below).
  torque         tau_applied (what the controller commanded) and tau_measured (the net joint
                 torque PhysX reports). They are not interchangeable: tau_measured obeys
                 Newton's law whatever produced it, so friction, backlash and a locked joint
                 cannot appear in it, while tau_applied shows all three (docs/REVIEW.md F2).
  decomposition  g, C qd, M qdd and the model's friction term, each as its own column. This
                 is the simulator's OWN breakdown of the torque it applied, so a reader can
                 see where a joint's torque went without re-deriving anything: gravity,
                 velocity coupling, inertia, friction, and whatever is left over.
  M, J           the full mass matrix and the tool Jacobian. Decimated (see below).
  energy         E_kin = 0.5 qd' M qd exactly; E_pot as a RELATIVE value, integrated from
                 dV/dt = g(q)' qd, because the absolute potential needs link centres of mass
                 that the articulation does not hand back cheaply. E_tot_rel is their sum and
                 is the conserved quantity on a frictionless free swing — which is what makes
                 it a check on the integrator rather than a decoration.
  power          per-joint tau*qd, its sum, the gravity power g'qd, and the cumulative
                 mechanical work. A payload that the model does not know about shows up in
                 the work budget before it shows up anywhere else.
  conditioning   singular values of the tool Jacobian's linear block, its condition number,
                 the Yoshikawa manipulability sqrt(det(J J')), and the condition number of M.
                 These say how trustworthy the pose is: a residual near a singularity is not
                 the same measurement as a residual in the middle of the workspace.
  headroom       |tau| as a fraction of each joint's effort limit, and the worst of them.
  tool           world pose, linear and angular velocity of the tool frame.
  labels         run id, trajectory style, payload, grip state, fault, in every row.

M/J DECIMATION. A full frame costs about 1.29 ms to read, and M and J are most of it, while
both change very little in one millisecond. `mj_every` reads them every Nth step and holds the
last value in between; `mj_fresh` marks the rows where they were actually read. The scalars
derived from M and J (energy, conditioning) are recomputed only on fresh rows and held too,
so decimation costs nothing extra. mj_every=1 disables it.

TIMING CONVENTION, recorded in the manifest. The row is assembled AFTER world.step(), so every
quantity in it is the post-step value and tau_measured[k] is the effort during the step that
produced q[k]. qdd[k] is therefore (qd[k] - qd[k-1])/dt, the acceleration that torque caused.
[MEASURED] evaluating the model against pre-step, post-step and midpoint state gives residuals
of 27.99 / 27.98 / 27.98 percent, i.e. the choice is not currently measurable.
"""
from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

import pal_config as cfg

SCHEMA_VERSION = 1
DOF = cfg.DOF
TAU_MAX = np.asarray(cfg.TAU_MAX, dtype=float)


def _names(dof: int = DOF) -> list[str]:
    """Column names in schema order."""
    c = ["t", "step", "dt"]
    for p in ("q", "qd", "qdd", "q_cmd", "qd_cmd", "qdd_cmd",
              "tau_applied", "tau_measured",
              "g", "coriolis", "tau_inertial", "tau_friction", "tau_unexplained",
              "P_joint", "tau_frac"):
        c += [f"{p}_{i}" for i in range(dof)]
    c += [f"M_{r}_{q}" for r in range(dof) for q in range(dof)]
    c += [f"J_{r}_{q}" for r in range(6) for q in range(dof)]
    c += ["E_kin", "E_pot_rel", "E_tot_rel", "P_mech", "P_grav", "W_mech",
          "tau_frac_max", "sigma_min", "sigma_max", "cond_J", "manip", "cond_M"]
    c += [f"ee_pos_{a}" for a in "xyz"] + [f"ee_quat_{a}" for a in "wxyz"]
    c += [f"ee_vel_{a}" for a in "xyz"] + [f"ee_omega_{a}" for a in "xyz"]
    c += ["run_id", "traj_type", "drive_mode", "payload_kg", "payload_attached",
          "grip_active", "box_index", "phase",
          "fault_type", "fault_joint", "fault_param", "fault_active"]
    c += [f"push_force_{a}" for a in "xyz"]
    return c


COLUMNS = _names()


class TelemetryWriter:
    """Accumulate rows in memory, write once at close().

    `ee_body_index` indexes the articulation's Jacobian along its body axis. It is resolved
    from the body name rather than hard-coded, because the index is a property of the USD
    asset's body ordering and a wrong one is silently wrong: it reports a Jacobian for the
    neighbouring link, which differs by one link offset and so biases every wrench estimate.
    """

    def __init__(self, articulation, run_id, out_dir="/workspace/data", *,
                 ee_body_index=None, ee_body_name=None, ee_prim=None, mj_every=1,
                 labels=None, manifest=None, noise=None, dof=DOF):
        self.art = articulation
        self.run_id = run_id
        self.dir = Path(out_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.dof = int(dof)
        self.ee_body_name = ee_body_name or cfg.TOOL_FRAME
        self.ee_body_index = (int(ee_body_index) if ee_body_index is not None
                              else self._resolve_body_index(self.ee_body_name))
        self.ee_prim = ee_prim
        self.mj_every = max(1, int(mj_every))
        self.labels = dict(labels or {})
        self.manifest_extra = dict(manifest or {})
        self.rows = []
        self.noise = noise              # optional pal_sensors.SensorNoise, applied to logged values

        n = self.dof
        self._M = np.eye(n)
        self._J = np.zeros((6, n))
        self._have_mj = False
        self._scalars = {k: float("nan") for k in
                         ("sigma_min", "sigma_max", "cond_J", "manip", "cond_M")}
        self._qd_prev = np.zeros(n)
        self._E_pot = 0.0               # relative: integrated from g' qd, starts at zero
        self._W = 0.0                   # cumulative mechanical work
        self._ndof_total = None

    # ── body-index resolution ───────────────────────────────────────────────────────────
    def _resolve_body_index(self, name):
        """The Jacobian's body-axis index for `name`.

        The articulation's Jacobian excludes the fixed base, so the index into
        get_jacobians()[0] is the body list's index minus one. Falls back to the last body
        with a warning rather than guessing a number, because a silent off-by-one here looks
        like a small modelling error rather than a wrong frame.
        """
        try:
            bodies = [str(b) for b in self.art.body_names]
        except Exception:
            bodies = []
        if name in bodies:
            return max(0, bodies.index(name) - 1)
        for cand in (cfg.TOOL_FRAME, "tool0", "flange"):
            if cand in bodies:
                print(f"[telemetry] body {name!r} not in the articulation; using {cand!r}", flush=True)
                return max(0, bodies.index(cand) - 1)
        print(f"[telemetry] body {name!r} not found in {bodies}; using the last body", flush=True)
        return max(0, len(bodies) - 2)

    # ── capture ─────────────────────────────────────────────────────────────────────────
    def capture(self, step, t, dt, *, q_cmd=None, qd_cmd=None, qdd_cmd=None,
                tau_applied=None, push_force=None, fault_active=False, grip_active=False,
                box_index=-1, phase=""):
        """Assemble and store one row. Call once per physics step, after world.step()."""
        a, n = self.art, self.dof
        if self._ndof_total is None:
            self._ndof_total = int(np.asarray(a.get_joint_positions()).reshape(-1).shape[0])
        nt = self._ndof_total

        q = np.asarray(a.get_joint_positions()).reshape(-1)[:n].astype(float)
        qd = np.asarray(a.get_joint_velocities()).reshape(-1)[:n].astype(float)
        tm = np.asarray(a.get_measured_joint_efforts()).reshape(-1)[:n].astype(float)
        if tau_applied is not None:
            ta = np.asarray(tau_applied, dtype=float).reshape(-1)[:n]
        else:
            ta = np.asarray(a.get_applied_joint_efforts()).reshape(-1)[:n].astype(float)
        g = np.asarray(a.get_generalized_gravity_forces()).reshape(-1)[:n].astype(float)
        cor = np.asarray(a.get_coriolis_and_centrifugal_forces()).reshape(-1)[:n].astype(float)

        # qdd from the velocity the previous step ended at: the acceleration this row's torque
        # produced. Step 0 has no predecessor, so it is zero rather than NaN — a NaN there
        # propagates through every downstream sum.
        qdd = (qd - self._qd_prev) / dt if step > 0 else np.zeros(n)
        self._qd_prev = qd.copy()

        fresh = (step % self.mj_every == 0) or not self._have_mj
        if fresh:
            self._M = np.asarray(a.get_mass_matrices()).reshape(nt, nt)[:n, :n].astype(float)
            J = np.asarray(a.get_jacobians())
            self._J = J.reshape(J.shape[0], -1, J.shape[-2], J.shape[-1])[0, self.ee_body_index][:, :n].astype(float)
            self._have_mj = True
            self._scalars = self._conditioning(self._M, self._J)
        M, J = self._M, self._J

        tau_inertial = M @ qdd
        tau_friction = cfg.friction_sigmoid(qd)
        # What the simulator's own decomposition does not account for. On a healthy arm driven
        # in effort mode this is the discretisation floor; it is NOT the attestation residual,
        # which is computed on the host against an independent Pinocchio model.
        tau_unexplained = ta - (tau_inertial + cor + g + tau_friction)

        E_kin = 0.5 * float(qd @ M @ qd)
        P_grav = float(g @ qd)                 # = dV/dt, since g = dV/dq
        E_pot = self._E_pot                    # the value AT this row: row 0 is the zero point
        self._E_pot += P_grav * dt             # advance for the next row
        P_joint = tm * qd
        P_mech = float(P_joint.sum())
        self._W += P_mech * dt
        tau_frac = np.abs(tm) / TAU_MAX

        ee_v = J @ qd
        ee_p, ee_q = self._ee_pose()

        row = {"t": float(t), "step": int(step), "dt": float(dt)}
        for name, vec in (("q", q), ("qd", qd), ("qdd", qdd),
                          ("tau_applied", ta), ("tau_measured", tm),
                          ("g", g), ("coriolis", cor), ("tau_inertial", tau_inertial),
                          ("tau_friction", tau_friction), ("tau_unexplained", tau_unexplained),
                          ("P_joint", P_joint), ("tau_frac", tau_frac)):
            for i in range(n):
                row[f"{name}_{i}"] = float(vec[i])
        for name, vec in (("q_cmd", q_cmd), ("qd_cmd", qd_cmd), ("qdd_cmd", qdd_cmd)):
            for i in range(n):
                row[f"{name}_{i}"] = float(vec[i]) if vec is not None else np.nan
        for r in range(n):
            for c in range(n):
                row[f"M_{r}_{c}"] = float(M[r, c])
        for r in range(6):
            for c in range(n):
                row[f"J_{r}_{c}"] = float(J[r, c])

        row["E_kin"] = E_kin
        row["E_pot_rel"] = float(E_pot)
        row["E_tot_rel"] = float(E_kin + E_pot)
        row["P_mech"] = P_mech
        row["P_grav"] = P_grav
        row["W_mech"] = float(self._W)
        row["tau_frac_max"] = float(tau_frac.max())
        row.update(self._scalars)

        for k, v in zip("xyz", ee_p):
            row[f"ee_pos_{k}"] = float(v)
        for k, v in zip("wxyz", ee_q):
            row[f"ee_quat_{k}"] = float(v)
        for k, v in zip("xyz", ee_v[:3]):
            row[f"ee_vel_{k}"] = float(v)
        for k, v in zip("xyz", ee_v[3:]):
            row[f"ee_omega_{k}"] = float(v)

        row.update(self.labels)
        row["run_id"] = self.run_id
        row["fault_active"] = bool(fault_active)
        row["grip_active"] = bool(grip_active)
        row["box_index"] = int(box_index)
        row["phase"] = str(phase)
        for k, v in zip("xyz", (push_force if push_force is not None else (np.nan,) * 3)):
            row[f"push_force_{k}"] = float(v)
        if self.mj_every > 1:
            row["mj_fresh"] = bool(fresh)
        if self.noise is not None:
            self.noise.apply(row)
        self.rows.append(row)

    # ── derived scalars ─────────────────────────────────────────────────────────────────
    @staticmethod
    def _conditioning(M, J):
        """Pose-quality scalars from the mass matrix and the tool Jacobian.

        Only the Jacobian's LINEAR block is used for the singular values and manipulability.
        The full 6xn Jacobian mixes m/s and rad/s rows, so its singular values have no single
        unit and its condition number is not a physical quantity — a trap worth naming, since
        taking the SVD of the whole thing is the obvious wrong move.
        """
        out = {}
        try:
            sv = np.linalg.svd(J[:3], compute_uv=False)
            out["sigma_min"] = float(sv.min())
            out["sigma_max"] = float(sv.max())
            out["cond_J"] = float(sv.max() / sv.min()) if sv.min() > 1e-12 else float("inf")
            out["manip"] = float(np.sqrt(max(np.linalg.det(J[:3] @ J[:3].T), 0.0)))
        except Exception:
            out.update({"sigma_min": np.nan, "sigma_max": np.nan,
                        "cond_J": np.nan, "manip": np.nan})
        try:
            out["cond_M"] = float(np.linalg.cond(M))
        except Exception:
            out["cond_M"] = np.nan
        return out

    def _ee_pose(self):
        """World pose of the tool prim. NaN if there is no stage (headless without USD)."""
        if not self.ee_prim:
            return (np.nan,) * 3, (1.0, 0.0, 0.0, 0.0)
        try:
            from pxr import UsdGeom
            import omni.usd
            prim = omni.usd.get_context().get_stage().GetPrimAtPath(self.ee_prim)
            m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
            tr = m.ExtractTranslation()
            qt = m.ExtractRotationQuat()
            im = qt.GetImaginary()
            return (tr[0], tr[1], tr[2]), (qt.GetReal(), im[0], im[1], im[2])
        except Exception:
            return (np.nan,) * 3, (1.0, 0.0, 0.0, 0.0)

    # ── summary and write ───────────────────────────────────────────────────────────────
    def summary(self) -> dict:
        """Per-run facts worth having without reloading the run: the energy budget, the worst
        headroom, the closest approach to a singularity. Goes into the manifest."""
        if not self.rows:
            return {}

        def col(name):
            return np.array([r.get(name, np.nan) for r in self.rows], dtype=float)

        tf = col("tau_frac_max")
        sig = col("sigma_min")
        ek, ep = col("E_kin"), col("E_pot_rel")
        return {
            "rows": len(self.rows),
            "duration_s": round(float(col("t")[-1]), 4),
            "energy": {
                "E_kin_max_J": round(float(np.nanmax(ek)), 4),
                "E_pot_swing_J": round(float(np.nanmax(ep) - np.nanmin(ep)), 4),
                "work_total_J": round(float(col("W_mech")[-1]), 4),
                "peak_power_W": round(float(np.nanmax(np.abs(col("P_mech")))), 3),
            },
            "headroom": {
                "tau_frac_max": round(float(np.nanmax(tf)), 4),
                "tau_frac_mean": round(float(np.nanmean(tf)), 4),
                "worst_joint": int(np.nanargmax([np.nanmax(np.abs(col(f"tau_measured_{i}"))) / TAU_MAX[i]
                                                 for i in range(self.dof)])),
            },
            "conditioning": {
                "sigma_min_worst": round(float(np.nanmin(sig)), 5),
                "manip_min": round(float(np.nanmin(col("manip"))), 6),
                "cond_M_max": round(float(np.nanmax(col("cond_M"))), 2),
            },
            "speed_fraction": round(float(np.nanmax([
                np.nanmax(np.abs(col(f"qd_{i}"))) / cfg.QD_MAX[i] for i in range(self.dof)])), 4),
        }

    def close(self):
        """Write <run_id>.npz and <run_id>.json into out_dir."""
        if not self.rows:
            print("[telemetry] nothing captured; not writing", flush=True)
            return None
        cols = [c for c in COLUMNS if c in self.rows[0]]
        if self.mj_every > 1:
            cols.append("mj_fresh")
        extra = [c for c in self.rows[0] if c not in cols]      # anything a caller added via labels
        cols += sorted(extra)
        data = {}
        for c in cols:
            vals = [r.get(c) for r in self.rows]
            if isinstance(vals[0], str):
                data[c] = np.array(vals, dtype=object)
            elif isinstance(vals[0], (bool, np.bool_)):
                data[c] = np.array(vals, dtype=bool)
            else:
                data[c] = np.asarray(vals, dtype=np.float64)
        data["__columns__"] = np.array(cols, dtype=object)
        out = self.dir / f"{self.run_id}.npz"
        np.savez_compressed(out, **data)

        man = {"schema_version": SCHEMA_VERSION, "run_id": self.run_id,
               "robot": cfg.ROBOT_NAME, "dof": self.dof,
               "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
               "ee_body_index": self.ee_body_index, "ee_body_name": self.ee_body_name,
               "ee_frame": self.ee_prim, "mj_every": self.mj_every,
               "tau_max": TAU_MAX.tolist(), "qd_max": np.asarray(cfg.QD_MAX).tolist(),
               "torque_convention": cfg.TORQUE_CONVENTION,
               "timing_convention":
                   "row assembled after world.step(); all quantities are post-step values; "
                   "tau_measured[k] is the effort during the step that produced q[k]; "
                   "qdd[k] = (qd[k] - qd[k-1])/dt. Measured: pre-step / post-step / midpoint "
                   "evaluation give 27.99 / 27.98 / 27.98 percent residual, i.e. "
                   "indistinguishable.",
               "energy_convention":
                   "E_kin = 0.5 qd' M qd with armature included in M. E_pot_rel is RELATIVE to "
                   "the first row, integrated from dV/dt = g(q)' qd; only differences are "
                   "meaningful. E_tot_rel = E_kin + E_pot_rel is conserved on a frictionless "
                   "free swing and is a check on the integrator.",
               "summary": self.summary()}
        man.update(self.manifest_extra)
        # Always on record, even when nothing was injected: "no sensor_noise key" and "ideal
        # sensing" are different claims, and a reader should not have to guess which one a run
        # meant. An absent key used to mean the first; now it cannot happen.
        man["sensor_noise"] = (self.noise.manifest() if self.noise is not None else
                               {"ideal": True, "sigma": {"q": 0.0, "qd": 0.0, "tau": 0.0},
                                "applied_to": [], "note": "no sensor model attached to this run"})
        for k in ("drive_mode", "payload_kg", "traj_type"):
            if k in self.labels:
                man[k] = self.labels[k]
        with open(self.dir / f"{self.run_id}.json", "w") as f:
            json.dump(man, f, indent=2, default=str)

        s = man["summary"]
        print(f"[telemetry] {len(self.rows)} rows x {len(cols)} cols -> {out}", flush=True)
        print(f"[telemetry] peak torque {s['headroom']['tau_frac_max']:.1%} of limit "
              f"(joint {s['headroom']['worst_joint']}), work {s['energy']['work_total_J']:.1f} J, "
              f"sigma_min {s['conditioning']['sigma_min_worst']:.4f}", flush=True)
        return out
