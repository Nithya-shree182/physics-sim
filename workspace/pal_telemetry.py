"""Telemetry writer for schema/telemetry_v0.yaml (the analysis side's schema, unchanged).

One row per physics step, written to Parquet, with a JSON manifest beside it.

Two things worth knowing before using it:

M/J DECIMATION. A full frame costs ~1.29 ms to read, which is why 1 kHz runs at 0.76x
real time. M (49 columns) and J (42) are 57% of the frame and dominate that cost, while
both change very little in a millisecond. `mj_every` reads them every Nth step and holds
the last value in between; `mj_fresh` marks the rows where they were actually read, which
is the encoding the analysis side proposed. mj_every=1 disables it.

TIMING CONVENTION. Written into the manifest as `timing_convention`. Measured, not
assumed: evaluating the model against pre-step, post-step and midpoint state gives
residuals of 27.99 / 27.98 / 27.98 percent, i.e. indistinguishable. The row is assembled
AFTER world.step(), so every quantity in it — state, efforts and dynamics terms alike —
is the post-step value, and tau_measured[k] is the effort during the step that produced
q[k]. That matches the analysis side's synthetic convention closely enough that the
choice is not currently measurable.
"""
from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

ARM = 7
SCHEMA_VERSION = 0


def _names():
    """Column names in schema order."""
    c = ["t", "step", "dt"]
    c += [f"q_{i}" for i in range(ARM)] + [f"qd_{i}" for i in range(ARM)]
    c += [f"tau_applied_{i}" for i in range(ARM)] + [f"tau_measured_{i}" for i in range(ARM)]
    c += [f"g_{i}" for i in range(ARM)] + [f"coriolis_{i}" for i in range(ARM)]
    c += [f"M_{r}_{q}" for r in range(ARM) for q in range(ARM)]
    c += [f"J_{r}_{q}" for r in range(6) for q in range(ARM)]
    c += [f"ee_pos_{a}" for a in "xyz"] + [f"ee_quat_{a}" for a in "wxyz"]
    c += [f"q_cmd_{i}" for i in range(ARM)] + [f"qd_cmd_{i}" for i in range(ARM)]
    c += [f"qdd_cmd_{i}" for i in range(ARM)]
    c += [f"finger_pos_{i}" for i in range(2)] + [f"finger_force_{i}" for i in range(2)]
    c += ["run_id", "traj_type", "drive_mode", "payload_kg", "payload_attached",
          "fault_type", "fault_joint", "fault_param", "fault_active",
          "attack_type", "attack_active"]
    c += [f"push_force_{a}" for a in "xyz"]
    return c


COLUMNS = _names()


class TelemetryWriter:
    def __init__(self, articulation, run_id, out_dir="/workspace/data", *,
                 ee_body_index=7, ee_prim=None, mj_every=1, labels=None, manifest=None, noise=None):
        self.art = articulation
        self.run_id = run_id
        self.dir = Path(out_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.ee_body_index = ee_body_index
        self.ee_prim = ee_prim
        self.mj_every = max(1, int(mj_every))
        self.labels = dict(labels or {})
        self.manifest_extra = dict(manifest or {})
        self.rows = []
        self._M = np.zeros((ARM, ARM))
        self._J = np.zeros((6, ARM))
        self._have_mj = False
        self.dropped = 0
        self.noise = noise              # optional pal_sensors.SensorNoise, applied to logged values

    def capture(self, step, t, dt, *, q_cmd=None, qd_cmd=None, qdd_cmd=None,
                push_force=None, fault_active=False, attack_active=False):
        a = self.art
        q_all = np.asarray(a.get_joint_positions()).flatten()
        qd_all = np.asarray(a.get_joint_velocities()).flatten()
        tm_all = np.asarray(a.get_measured_joint_efforts()).flatten()
        ta_all = np.asarray(a.get_applied_joint_efforts()).flatten()
        g = np.asarray(a.get_generalized_gravity_forces()).flatten()[:ARM]
        cor = np.asarray(a.get_coriolis_and_centrifugal_forces()).flatten()[:ARM]

        fresh = (step % self.mj_every == 0) or not self._have_mj
        if fresh:
            self._M = np.asarray(a.get_mass_matrices()).reshape(9, 9)[:ARM, :ARM]
            J = np.asarray(a.get_jacobians())
            self._J = J.reshape(J.shape[0], -1, J.shape[-2], J.shape[-1])[0, self.ee_body_index][:, :ARM]
            self._have_mj = True

        ee_p, ee_q = self._ee_pose()

        row = {"t": float(t), "step": int(step), "dt": float(dt)}
        for i in range(ARM):
            row[f"q_{i}"] = float(q_all[i]);            row[f"qd_{i}"] = float(qd_all[i])
            row[f"tau_applied_{i}"] = float(ta_all[i]); row[f"tau_measured_{i}"] = float(tm_all[i])
            row[f"g_{i}"] = float(g[i]);                row[f"coriolis_{i}"] = float(cor[i])
            row[f"q_cmd_{i}"] = float(q_cmd[i]) if q_cmd is not None else np.nan
            row[f"qd_cmd_{i}"] = float(qd_cmd[i]) if qd_cmd is not None else np.nan
            row[f"qdd_cmd_{i}"] = float(qdd_cmd[i]) if qdd_cmd is not None else np.nan
        for r in range(ARM):
            for c in range(ARM):
                row[f"M_{r}_{c}"] = float(self._M[r, c])
        for r in range(6):
            for c in range(ARM):
                row[f"J_{r}_{c}"] = float(self._J[r, c])
        for k, v in zip("xyz", ee_p):
            row[f"ee_pos_{k}"] = float(v)
        for k, v in zip("wxyz", ee_q):
            row[f"ee_quat_{k}"] = float(v)
        for i in range(2):
            row[f"finger_pos_{i}"] = float(q_all[ARM + i]) if len(q_all) > ARM + i else np.nan
            row[f"finger_force_{i}"] = float(tm_all[ARM + i]) if len(tm_all) > ARM + i else np.nan
        row.update(self.labels)
        row["run_id"] = self.run_id
        row["fault_active"] = bool(fault_active)
        row["attack_active"] = bool(attack_active)
        for k, v in zip("xyz", (push_force if push_force is not None else (np.nan,) * 3)):
            row[f"push_force_{k}"] = float(v)
        if self.mj_every > 1:
            row["mj_fresh"] = bool(fresh)
        if self.noise is not None:
            self.noise.apply(row)
        self.rows.append(row)

    def _ee_pose(self):
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

    def close(self):
        """Write the run.

        The Isaac Sim container has neither pandas nor pyarrow, so the frame is written
        as a compressed .npz with numpy alone and converted to Parquet on the host by
        tools/npz_to_parquet.py. Parquet is still what the analysis side receives; this
        is only about where the conversion happens.
        """
        cols = [c for c in COLUMNS if c in self.rows[0]]
        if self.mj_every > 1:
            cols.append("mj_fresh")
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
               "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
               "ee_body_index": self.ee_body_index, "ee_frame": self.ee_prim,
               "rows": len(self.rows), "mj_every": self.mj_every,
               "timing_convention":
                   "row assembled after world.step(); all quantities are post-step values; "
                   "tau_measured[k] is the effort during the step that produced q[k]. "
                   "Measured: pre-step / post-step / midpoint evaluation give "
                   "27.99 / 27.98 / 27.98 percent residual, i.e. indistinguishable."}
        man.update(self.manifest_extra)
        if self.noise is not None:
            man["sensor_noise"] = self.noise.manifest()
        for k in ("drive_mode", "payload_kg", "traj_type"):
            if k in self.labels:
                man[k] = self.labels[k]
        with open(self.dir / f"{self.run_id}.json", "w") as f:
            json.dump(man, f, indent=2, default=str)

        print(f"[telemetry] {len(self.rows)} rows x {len(cols)} cols -> {out}", flush=True)
        print(f"[telemetry] manifest -> {self.dir / (self.run_id + '.json')}", flush=True)
        return out
