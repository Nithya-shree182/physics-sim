"""Data engine behind the dashboard: a source of telemetry blocks, and the analysis on top.

Two sources produce the same kind of block (50 ms of 1 kHz physics, decimated to 200 Hz):

  LiveSource    a stand-in plant: our Pinocchio plant (link masses +-3%, friction x1.15, sensor
                noise, optional payload / fault) driven by a joint-impedance controller, stepped in
                real time while the user moves the load, speed and fault controls.
                It is NOT Isaac Sim. Replace it with dev's HTTP feed when that exists.
  ReplaySource  one of our reference runs (data/ref_matched_*, tracked in git), looped.

Analyzer runs the repo's own analysis code on a sliding window of blocks: observers/residual.py
(compute_residual, 20 Hz zero-phase filter), observers/confidence.py (conditioning, gates) and the
payload formula of observers/payload.py. The decisions on top (payload vs fault, which joint) are
a live version of observers/detector.py: same constants, same standardised z-score rule, but with a
calibration window taken from the live session. Offline it is exact; live it lags by the 0.15 s of
filter edge that residual.py drops.
"""
from __future__ import annotations

import json
import os
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[1]
for sub in ("model", "observers", "schema"):
    sys.path.insert(0, str(REPO / sub))

from confidence import FORCE_GAIN_MAX, VERTICAL_GAIN_MAX, conditioning  # noqa: E402
from detector import (CALIBRATION_S, FAULT_LABELS, FLOOR_RESIDUAL_NM, FLOOR_TRACKING_RAD,  # noqa: E402
                      Z_THRESHOLD)
from payload import GRAVITY_VEC, MIN_D_NORM, PAYLOAD_OFFSET_M, TAU_S  # noqa: E402
from pinocchio_model import load_panda  # noqa: E402
from poses import HOME_JOINTS  # noqa: E402
from residual import DEFAULT_CUTOFF_HZ, TIMING_FORWARD, Run, compute_residual  # noqa: E402
from synth_log import (KD, KP, MASS_MISMATCH, NOISE_Q, NOISE_QD, NOISE_TAU, PLANT_FRICTION_SCALE,  # noqa: E402
                       QD_MAX, build_plant)

BLOCK_STEPS = 50                 # physics steps per block (50 ms at 1 kHz)
DECIM = 5                        # analysis rows are every 5th step: 200 Hz
DT_ROW = DECIM * 1e-3
WINDOW_ROWS = 240                # 1.2 s of 200 Hz rows
EDGE_S = 0.15                    # rows dropped by compute_residual at each end (3 filter time constants)
BAND = 0.5                       # trusted fraction of the velocity limit (dev, RESIDUAL_FLOOR.md)
CONSEC_TICKS = 3                 # [CHOICE] blocks in a row (150 ms), as detector.CONSECUTIVE but per block
LIVE_Z = Z_THRESHOLD
PAYLOAD_ON_KG, PAYLOAD_OFF_KG = 0.20, 0.12
AMP = np.array([0.4, 0.25, 0.4, 0.3, 0.5, 0.3, 0.5])         # rad, stand-in trajectory amplitudes
# Dev's sweep (RESIDUAL_FLOOR.md): worst-joint RMS residual, % of limit, 1000 Hz position drive.
FLOOR_TABLE = [(0.0, 0.0), (0.1, 0.43), (0.2, 0.86), (0.3, 1.27), (0.5, 2.05), (0.75, 8.79), (1.0, 30.21)]
# Empty-arm residual of data/ref_matched_0kg, % of each joint's limit, tau_measured against the model without friction (the
# simulator's measured torque is net torque; see docs/REVIEW.md). The old values, from the wrong-plant log, were 0.09-1.75%.
REAL_RMS_PCT = [0.0071, 0.0103, 0.0113, 0.006, 0.0763, 0.0815, 0.0007]


def floor_pct(s: float) -> float:
    xs, ys = zip(*FLOOR_TABLE)
    return float(np.interp(s, xs, ys))


# ── sources ─────────────────────────────────────────────────────────────────────────────
@dataclass
class Controls:
    load: float = 0.0
    speeds: list = field(default_factory=lambda: [30.0] * 7)     # % of velocity limit
    fault: str = "none"            # none | friction | stuck_joint | delay
    fault_joint: int = 2
    reach: float = 0.0             # 0 home, 1 elbow stretched towards the singular pose


@dataclass
class Block:
    t: float
    q: np.ndarray        # (10, 7) logged, decimated
    qd: np.ndarray
    tau: np.ndarray      # measured torque, mean over each decimation step
    q_cmd: np.ndarray
    truth: dict          # what the source knows and the analysis must not use


class LiveSource:
    kind = "live"
    timing = "forward"

    def __init__(self, seed: int = 0):
        self.twin = load_panda()
        self.rng = np.random.default_rng(seed)
        self.seed = seed
        self.c = Controls()
        self._load_built = None
        self._friction_base = None
        self.plant = None
        self._build_plant(0.0)
        self.q = HOME_JOINTS.copy()
        self.qd = np.zeros(7)
        self.phase = np.zeros(7)
        self.q0 = HOME_JOINTS.copy()
        self.t = 0.0
        self.fifo = deque(maxlen=60)
        self.q_stuck = None
        self.artifact = False

    def _build_plant(self, load: float):
        rng = np.random.default_rng(self.seed)
        self.plant, _ = build_plant(load, MASS_MISMATCH, PLANT_FRICTION_SCALE, rng)
        self._friction_base = self.plant.friction_scale.copy()
        self._load_built = load

    def _command(self, dt: float):
        sp = np.maximum(np.array(self.c.speeds) / 100.0, 0.02)
        w = sp * QD_MAX / AMP
        self.phase += w * dt
        tgt = HOME_JOINTS.copy()
        tgt[3] = HOME_JOINTS[3] + (-0.8 - HOME_JOINTS[3]) * self.c.reach     # stretch the elbow
        step = np.clip(tgt - self.q0, -0.4 * dt, 0.4 * dt)                    # <= 0.4 rad/s drift
        self.q0 = self.q0 + step
        amp = AMP.copy()
        amp[3] = AMP[3] * (1 - 0.5 * self.c.reach)
        qc = self.q0 + amp * np.sin(self.phase)
        qdc = amp * w * np.cos(self.phase) + step / dt
        qddc = -amp * w * w * np.sin(self.phase)
        return qc, qdc, qddc

    def next_block(self) -> Block:
        c = self.c
        if abs(c.load - self._load_built) > 1e-6:
            self._build_plant(c.load)
        dt = 1e-3
        fault, fj = c.fault, c.fault_joint
        rows = {k: [] for k in ("q", "qd", "tau", "qc")}
        for k in range(BLOCK_STEPS):
            qc, qdc, qddc = self._command(dt)
            self.fifo.append((qc, qdc, qddc))
            lag = 50 if fault == "delay" else 0
            cq, cqd, cqdd = self.fifo[max(len(self.fifo) - 1 - lag, 0)]
            qm = self.q + self.rng.normal(0, NOISE_Q, 7)
            qdm = self.qd + self.rng.normal(0, NOISE_QD, 7)
            tau = (self.twin.tau_model(cq, cqd, cqdd) + KP * (cq - qm) + KD * (cqd - qdm))
            tau = np.clip(tau, -self.plant.effort_limit, self.plant.effort_limit)
            self.plant.friction_scale = self._friction_base.copy()
            if fault == "friction":
                self.plant.friction_scale[fj] *= 3.0
            tau_meas = tau + self.rng.normal(0, NOISE_TAU, 7)
            if max(c.speeds) > BAND * 100:
                # Above ~50% of the velocity limit dev's position drive reports torque that is mostly
                # artifact (RESIDUAL_FLOOR.md). The stand-in imitates it from dev's table, per joint.
                s = np.array(c.speeds) / 100.0
                amp_pct = np.array([floor_pct(v) for v in s]) * (s > BAND)
                tau_meas = tau_meas + amp_pct / 100 * self.plant.effort_limit * 1.4 * np.sin(self.phase)
            if fault == "stuck_joint":
                if self.q_stuck is None:
                    self.q_stuck = self.q[fj]
                    self.qd[fj] = 0.0
                free = np.arange(7) != fj
                b = tau - self.plant.tau_model(self.q, self.qd, np.zeros(7))
                qdd = np.zeros(7)
                qdd[free] = np.linalg.solve(self.plant.M(self.q)[np.ix_(free, free)], b[free])
            else:
                self.q_stuck = None
                qdd = self.plant.forward_dynamics(self.q, self.qd, tau)
            if k % DECIM == 0:
                rows["q"].append(qm), rows["qd"].append(qdm), rows["qc"].append(qc)
                rows["tau"].append([])
            rows["tau"][-1].append(tau_meas)
            self.qd = self.qd + dt * qdd
            self.q = self.q + dt * self.qd
            if self.q_stuck is not None:
                self.q[fj], self.qd[fj] = self.q_stuck, 0.0
            self.t += dt
        tau_rows = np.array([np.mean(t, axis=0) for t in rows["tau"]])
        return Block(self.t, np.array(rows["q"]), np.array(rows["qd"]), tau_rows, np.array(rows["qc"]),
                     {"load": c.load, "fault": fault, "fault_joint": fj})


class ReplaySource:
    kind = "replay"
    timing = "backward"

    def __init__(self, path: Path):
        self.twin = None
        df = pd.read_parquet(path)
        self.payload_kg = float(df["payload_kg"].iloc[0])
        self.name = path.stem
        sel = lambda p: df[[f"{p}_{i}" for i in range(7)]].to_numpy()   # noqa: E731
        q, qd, tau, qc = sel("q"), sel("qd"), sel("tau_measured"), sel("q_cmd")
        n = (len(df) // BLOCK_STEPS) * BLOCK_STEPS
        self.q, self.qd, self.tau, self.qc = q[:n], qd[:n], tau[:n], qc[:n]
        self.n, self.k, self.t = n, 0, 0.0
        self.c = Controls(load=self.payload_kg)
        self.wrapped = False

    def next_block(self) -> Block:
        sl = slice(self.k, self.k + BLOCK_STEPS)
        idx = np.arange(self.k, self.k + BLOCK_STEPS)
        take = idx[::DECIM]
        tau = self.tau[sl].reshape(-1, DECIM, 7).mean(axis=1)
        self.k += BLOCK_STEPS
        self.t += BLOCK_STEPS * 1e-3
        self.wrapped = self.k >= self.n
        if self.wrapped:
            self.k = 0
        return Block(self.t, self.q[take], self.qd[take], tau, self.qc[take],
                     {"load": self.payload_kg, "fault": "none", "fault_joint": -1})


class IsaacSource:
    """The live Isaac Sim (workspace/pal_live.py, started by pal.sh): rows come over HTTP, the dashboard commands picks back.
    Rows are 200 Hz; a block is 10 of them (50 ms). A new `run_id` means a new pick: the window and the detector restart."""
    kind = "isaac"
    timing = "backward"
    URL = os.environ.get("PAL_LIVE_URL", "http://127.0.0.1:8765")

    def __init__(self):
        self.twin = None
        self.cursor = 0
        self._synced = False
        self.pending = deque()
        self.status = {}
        self.connected = False
        self.run_id = None
        self.new_run = False
        self.c = Controls()
        self.c.load = 1.0
        self.last_error = ""
        self._last_poll = 0.0

    def _get(self):
        import urllib.request
        with urllib.request.urlopen(f"{self.URL}/state?since={self.cursor}", timeout=1.5) as r:
            return json.loads(r.read())

    def poll(self):
        try:
            d = self._get()
        except Exception as exc:
            self.connected, self.last_error = False, repr(exc)
            return
        self.connected = True
        self.status = d["status"]
        if not self._synced or d["seq"] < self.cursor:   # first contact, or the simulator restarted: start at "now", not at its history
            self._synced = True
            self.cursor = d["seq"]
            self.pending.clear()
            return
        self.cursor = d["last"]
        self.pending.extend(d["rows"])
        while len(self.pending) > 1500:                  # more than 7 s behind: skip ahead
            self.pending.popleft()

    def command(self, **msg) -> dict:
        import urllib.request
        req = urllib.request.Request(f"{self.URL}/cmd", json.dumps(msg).encode(), {"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=3) as r:
                return json.loads(r.read())
        except Exception as exc:
            return {"ok": False, "error": repr(exc)}

    def next_block(self):
        if len(self.pending) < DECIM * 2 and time.time() - self._last_poll > 0.04:
            self._last_poll = time.time()
            self.poll()
        n = BLOCK_STEPS // DECIM
        if len(self.pending) < n:
            return None
        rows = [self.pending.popleft() for _ in range(n)]
        if rows[0][-1] != self.run_id:
            self.run_id, self.new_run = rows[0][-1], True
        rows = [r for r in rows if r[-1] == self.run_id] or rows
        a = np.array([r[:-2] for r in rows], dtype=float)
        held = float(rows[-1][-2])
        st = self.status
        return Block(t=float(a[-1, 0]), q=a[:, 1:8], qd=a[:, 8:15], tau=a[:, 15:22], q_cmd=a[:, 22:29],
                     truth={"load": held, "fault": st.get("fault", "none"), "fault_joint": st.get("joint", -1)})


# ── analysis ────────────────────────────────────────────────────────────────────────────
def payload_torque_per_kg(twin, q, qd=None, qdd=None) -> np.ndarray:
    """Torque on each joint from 1 kg at the payload point: J_p^T (a_p + (0, 0, g)). At rest this is
    the weight; with qd and qdd it includes the payload's own acceleration (a moving load pulls
    differently from a static one)."""
    J = twin.jacobian(q)
    a = np.zeros(3) if qd is None else twin.point_acceleration(q, qd, qdd, PAYLOAD_OFFSET_M)
    R = twin.data.oMf[twin.model.getFrameId("panda_hand")].rotation
    p = R @ np.array(PAYLOAD_OFFSET_M)
    skew = np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])
    return (J[:3] - skew @ J[3:]).T @ (a + np.array([0, 0, 9.81]))


class Analyzer:
    """Sliding-window residual, confidence, payload and alert logic."""

    def __init__(self, twin, timing: str, detector: bool = True):
        self.twin, self.timing, self.use_detector = twin, timing, detector
        self.limit = twin.effort_limit
        self.reset_window()
        self.tare = 0.0
        self.hist = deque(maxlen=400)                # one entry per block (20 Hz): last 20 s
        self.alerts: dict[str, dict] = {}
        self.log: list[dict] = []
        self.snapshot: dict = {}
        self.cal = None
        self._start_calibration()
        self.metrics = dict(delay_payload=None, delay_fault=None, fault_kind=None, false_alarms=0)
        self._t_load = self._t_fault = None
        self._healthy_since = 0.0
        self.last_row_t = -1.0
        self.m_mean = None
        self.m_var = 0.0
        self.m_n = 0
        self.m_est_prev = 0.0
        self.streak: dict[str, int] = {}
        self.fault_hold = False

    def reset_window(self):
        self.buf = {k: deque(maxlen=WINDOW_ROWS) for k in ("t", "q", "qd", "tau", "qc")}

    def _start_calibration(self):
        self.cal = {"u": [], "e": [], "lag": [], "ready": False, "mu_lag": 0.0, "sd_lag": 0.01, "mu_u": np.zeros(7), "sd_u": np.ones(7),
                    "mu_e": np.zeros(7), "sd_e": np.ones(7), "t0": None}

    def recalibrate(self):
        self._start_calibration()
        self.streak.clear()

    def set_tare(self, on: bool):
        self.tare = self.m_est_prev if on else 0.0

    # --- one block in, one snapshot out ---
    def feed(self, blk: Block, controls: Controls, running_s: float) -> dict:
        t_rows = blk.t - BLOCK_STEPS * 1e-3 + DT_ROW * np.arange(1, len(blk.q) + 1) - DT_ROW
        for i in range(len(blk.q)):
            self.buf["t"].append(t_rows[i])
            for k, arr in (("q", blk.q), ("qd", blk.qd), ("tau", blk.tau), ("qc", blk.q_cmd)):
                self.buf[k].append(arr[i])
        if len(self.buf["t"]) < int(2.5 * EDGE_S / DT_ROW):
            return self.snapshot
        n = len(self.buf["t"])
        df = pd.DataFrame({"t": list(self.buf["t"]), "dt": DT_ROW})
        for key, name in (("q", "q"), ("qd", "qd"), ("tau", "tau_measured")):
            a = np.array(self.buf[key])
            for j in range(7):
                df[f"{name}_{j}"] = a[:, j]
        man = {"timing": TIMING_FORWARD} if self.timing == "forward" else {"timing_convention": "after world.step() produced q[k]"}
        man["drive_mode"] = df["drive_mode"] = "torque" if self.timing == "forward" else "position_stiff"
        res = compute_residual(Run(df, man, "live"), self.twin, cutoff_hz=DEFAULT_CUTOFF_HZ)
        ok = np.flatnonzero(res.valid)
        if len(ok) < 6:
            return self.snapshot
        sl = ok[-10:]                                   # last 50 ms of valid rows
        k = ok[-1]
        r_now = res.r[sl].mean(axis=0)
        q, qd = res.q[k], res.qd[k]
        cs = conditioning(res.J[k])
        gated_det, gated_pay = cs["force_noise_gain"] > FORCE_GAIN_MAX, cs["vertical_noise_gain"] > VERTICAL_GAIN_MAX
        qc = np.array(self.buf["qc"])[k]
        track = qc - np.array(self.buf["q"])[k]
        tkg = payload_torque_per_kg(self.twin, q, qd, res.qdd[k])
        over = [j for j, v in enumerate(controls.speeds) if v > BAND * 100] if self.use_detector else []

        # payload: running mean of the per-row estimate over the rows not yet used
        new = [i for i in ok if res.t[i] > self.last_row_t]
        for i in new:
            if conditioning(res.J[i])["vertical_noise_gain"] > VERTICAL_GAIN_MAX:
                continue
            d = GRAVITY_VEC - self.twin.point_acceleration(res.q[i], res.qd[i], res.qdd[i], PAYLOAD_OFFSET_M)
            if np.linalg.norm(d) < MIN_D_NORM or over or self.fault_hold:
                continue
            m_raw = float(res.wrench[i, :3] @ d / (d @ d))
            self.m_n += 1
            if self.m_mean is None:
                self.m_mean, self.m_var = m_raw, 0.0
            else:
                alpha = min(1.0, DT_ROW / TAU_S)
                w = max(alpha, 1.0 / self.m_n)
                diff = m_raw - self.m_mean
                self.m_mean += w * diff
                self.m_var = (1 - w) * (self.m_var + w * diff * diff)
        if new:
            self.last_row_t = res.t[new[-1]]
        m_est = self.m_mean if self.m_mean is not None else 0.0
        n_eff = max(min(self.m_n, 2.0 / min(1.0, DT_ROW / TAU_S) - 1.0), 1.0)
        band = 2.0 * np.sqrt(self.m_var / n_eff)
        self.m_est_prev = m_est
        mass = m_est - self.tare

        events = self._alerts(blk, controls, r_now, track, qd, tkg, mass, gated_det, gated_pay, over, running_s, cs) \
            if self.use_detector else {}

        nowpct = r_now / self.limit * 100
        comp = {"gravity": self.twin.g(q), "inertia": self.twin.M(q) @ res.qdd[k],
                "coupling": self.twin.coriolis(q, qd), "friction": self.twin.friction(qd), "unexplained": r_now}
        entry = {"t": blk.t, "r": r_now.copy(), "tau_meas": res.tau[k].copy(), "tau_model": res.tau_model[k].copy(),
                 "cn": cs["c_norm"], "est": mass, "load": blk.truth["load"], "track": track.copy(), "gated": gated_det}
        self.hist.append(entry)
        pts = self._geometry(q)
        self.snapshot = {
            "t": blk.t, "q": q, "qd": qd, "r": r_now, "r_pct": nowpct, "tau_meas": res.tau[k], "tau_model": res.tau_model[k],
            "track": track, "conf": cs, "gated_det": bool(gated_det), "gated_pay": bool(gated_pay),
            "est": mass, "band": band, "raw_est": m_est, "load": blk.truth["load"], "tare": self.tare,
            "wrench": res.wrench[k], "comp": comp, "tkg": tkg, "geom": pts, "events": events,
            "alerts": self.alerts, "log": self.log[:8], "over": over, "metrics": self.metrics,
            "calibrating": self.use_detector and not self.cal["ready"],
            "cal_progress": min(1.0, (len(self.cal["u"]) * 0.05) / CALIBRATION_S),
            "truth": blk.truth, "n": len(self.hist),
        }
        return self.snapshot

    # --- twin geometry for the picture ---
    def _geometry(self, q) -> dict:
        tw = self.twin
        tw.ee_pose(q)
        origins = [np.zeros(3)] + [tw.data.oMi[j].translation.copy() for j in range(1, 8)]
        hand = tw.ee_position(q)
        J = tw.jacobian(q)
        S = J[[0, 2]] @ J[[0, 2]].T
        w, v = np.linalg.eigh(S)
        return {"joints": origins[1:], "base": origins[0], "hand": hand,
                "ell": {"a": float(np.sqrt(max(w[1], 0))), "b": float(np.sqrt(max(w[0], 0))),
                        "ang": float(np.arctan2(v[1, 1], v[0, 1]))}}

    # --- decisions ---
    def _alerts(self, blk, ctl, r, track, qd, tkg, mass, gated_det, gated_pay, over, running_s, cs):
        out: dict[str, dict] = {}
        J = lambda i: f"Joint {i + 1}"   # noqa: E731
        t = blk.t
        # payload signature fit: which mass explains the residual pattern, ignoring joints it cannot explain
        sd_u = self.cal["sd_u"] if self.cal["ready"] else np.ones(7)
        wgt = 1.0 / np.maximum(sd_u, FLOOR_RESIDUAL_NM) ** 2
        ex = np.zeros(7, bool)
        fit = 0.0
        for _ in range(4):
            den = float((wgt * tkg * tkg)[~ex].sum())
            fit = float((wgt * r * tkg)[~ex].sum() / den) if den > 0 else 0.0
            nx = np.abs(r - fit * tkg) > 6 * np.maximum(sd_u, FLOOR_RESIDUAL_NM) + 0.25
            if (nx == ex).all():
                break
            ex = nx
        u = r - fit * tkg
        e = track
        healthy = ctl.fault == "none" and ctl.load < 0.1 and max(ctl.speeds) <= BAND * 100
        if not healthy:
            self._healthy_since = t

        if not self.cal["ready"]:
            if not over and not gated_det and ctl.fault == "none":
                self.cal["u"].append(u.copy())
                self.cal["e"].append(e.copy())
                self.cal["lag"].append(self._lag())
                if len(self.cal["u"]) * 0.05 >= CALIBRATION_S:
                    U, E = np.array(self.cal["u"]), np.array(self.cal["e"])
                    self.cal.update(ready=True, mu_u=U.mean(0), sd_u=np.maximum(U.std(0), FLOOR_RESIDUAL_NM),
                                    mu_e=E.mean(0), sd_e=np.maximum(E.std(0), FLOOR_TRACKING_RAD),
                                    mu_lag=float(np.mean(self.cal["lag"])), sd_lag=max(float(np.std(self.cal["lag"])), 0.005))
            self._set_alerts({}, t, healthy)
            return {}
        z_u = (u - self.cal["mu_u"]) / self.cal["sd_u"]
        z_e = (e - self.cal["mu_e"]) / self.cal["sd_e"]
        lag = self._lag()
        z_lag = (lag - self.cal["mu_lag"]) / self.cal["sd_lag"]

        if over:
            for i in over:
                out[f"speed{i}"] = dict(sev=2, where=J(i), title="Speed above the trusted band",
                    cause="Position-drive floor, not a fault",
                    lines=[f"{J(i)} is at {ctl.speeds[i]:.0f}% of its velocity limit.",
                           f"Reported-torque floor about {floor_pct(ctl.speeds[i] / 100):.1f}% of limit. "
                           "Residual alarms and the load estimate are held until it is back in band."])
            self._set_alerts(out, t, healthy)
            return out
        if gated_det:
            out["gate"] = dict(sev=0, where="Whole arm", title="Detector gated at this pose", cause="Near a singularity",
                               lines=[f"Force noise gain {cs['force_noise_gain']:.1f} N per N·m is above {FORCE_GAIN_MAX:.0f}; "
                                      "residual alarms are held."])
            self._set_alerts(out, t, healthy)
            return out

        hit_u = np.abs(z_u) > LIVE_Z
        for i in range(7):
            self.streak[f"u{i}"] = self.streak.get(f"u{i}", 0) + 1 if hit_u[i] else 0
            self.streak[f"e{i}"] = self.streak.get(f"e{i}", 0) + 1 if abs(z_e[i]) > LIVE_Z else 0
        self.streak["lag"] = self.streak.get("lag", 0) + 1 if z_lag > LIVE_Z and lag - self.cal["mu_lag"] > 0.02 else 0
        rj = [i for i in range(7) if self.streak[f"u{i}"] >= CONSEC_TICKS]
        seized = [i for i in rj if abs(qd[i]) < 0.02 and self.streak[f"e{i}"] >= CONSEC_TICKS]
        faultOn = False
        if seized:                                   # a jam disturbs the other joints too: name the worst one
            i = max(seized, key=lambda j: abs(z_u[j]))
            faultOn = True
            out[f"stuck{i}"] = dict(sev=3, where=J(i), title="Stuck joint or blocked motion", cause="Collision, jam or seized joint",
                lines=[f"Unexplained torque {u[i]:+.2f} N·m ({abs(u[i]) / self.limit[i] * 100:.0f}% of limit), z = {z_u[i]:+.0f}.",
                       f"Tracking error {e[i]:+.3f} rad while the command keeps moving."])
        else:
            for i in rj:
                faultOn = True
                out[f"fric{i}"] = dict(sev=2, where=J(i), title=FAULT_LABELS["friction"], cause="Friction beyond the model",
                    lines=[f"Unexplained torque {u[i]:+.2f} N·m while moving ({abs(u[i]) / self.limit[i] * 100:.1f}% of limit), z = {z_u[i]:+.0f}.",
                           f"The other joints fit a {fit:.2f} kg load; this one does not, so it is not gravity."])
        if self.streak["lag"] >= CONSEC_TICKS and not rj:
            faultOn = True
            out["delay"] = dict(sev=3, where="All joints", title="Command delay / inconsistent command",
                cause="Tracking channel only; the residual cannot see it",
                lines=[f"Tracking error leads the motion by {lag * 1000:.0f} ms (baseline {self.cal['mu_lag'] * 1000:.0f} ms).",
                       "Torque and motion stay consistent, so the residual is quiet: only the tracking channel sees it."])
        self.fault_hold = faultOn
        for i in range(7):
            if abs(self.tau_now(i)) > 0.9 * self.limit[i]:
                out[f"limit{i}"] = dict(sev=3, where=J(i), title="Near torque limit", cause="Gravity plus payload",
                    lines=[f"Measured {self.tau_now(i):+.1f} N·m of {self.limit[i]:.0f} N·m."])
        thr = PAYLOAD_OFF_KG if "payload" in self.alerts else PAYLOAD_ON_KG
        active_fault = any(a.startswith(("fric", "stuck", "delay")) for a in self.alerts)
        if not faultOn and not active_fault and not hit_u.any() and not gated_pay and mass > thr:
            k = int(np.argmax(np.abs(tkg)))
            out["payload"] = dict(sev=1, where=f"{J(k)} carries most of it", title="Unmodelled load on the gripper",
                cause="Payload, seen as extra gravity torque",
                lines=[f"Estimated {mass:.2f} kg (mock load {blk.truth['load']:.2f} kg).",
                       f"{J(k)} residual {r[k]:+.2f} N·m ({abs(r[k]) / self.limit[k] * 100:.1f}% of limit)."])
        self._set_alerts(out, t, healthy)
        return out

    def _lag(self) -> float:
        """Command lag in seconds: least-squares slope of the tracking error on joint velocity over the
        window, with each mean removed so a steady payload sag does not count. A pure delay d gives
        e = qd * d. This is the tracking channel's reading of a 'delayed or inconsistent command'."""
        e = np.array(self.buf["qc"]) - np.array(self.buf["q"])
        v = np.array(self.buf["qd"])
        e, v = e - e.mean(0), v - v.mean(0)
        den = float((v * v).sum())
        return float((e * v).sum() / den) if den > 1e-9 else 0.0

    def tau_now(self, i):
        return float(self.buf["tau"][-1][i]) if len(self.buf["tau"]) else 0.0

    def _set_alerts(self, out, t, healthy):
        for aid, a in out.items():
            if aid not in self.alerts:
                self.alerts[aid] = dict(a, born=t, last=t, dismissed=False)
                self.log.insert(0, {"t": f"{t:.1f} s", "text": f"raised: {a['title']} · {a['where']}"})
                if aid == "payload" and self._t_load is not None and self.metrics["delay_payload"] is None:
                    self.metrics["delay_payload"] = t - self._t_load
                if aid.startswith(("fric", "stuck", "delay")) and self._t_fault is not None and self.metrics["delay_fault"] is None:
                    self.metrics["delay_fault"], self.metrics["fault_kind"] = t - self._t_fault, a["title"]
                if a["sev"] > 0 and healthy and t - self._healthy_since > 3:
                    self.metrics["false_alarms"] += 1
            else:
                self.alerts[aid].update(a, last=t)
        for aid in list(self.alerts):
            if aid not in out and t - self.alerts[aid]["last"] > 1.5:
                self.log.insert(0, {"t": f"{t:.1f} s", "text": f"cleared: {self.alerts[aid]['title']}"})
                del self.alerts[aid]

    def mark_load_change(self, t):
        self._t_load, self.metrics["delay_payload"] = t, None

    def mark_fault_change(self, t, on: bool):
        self._t_fault, self.metrics["delay_fault"] = (t if on else None), None


# ── the running engine ──────────────────────────────────────────────────────────────────
class _NoBlock(Exception):
    pass


class Engine:
    """Steps the source in real time on a background thread; the UI reads `.snapshot`."""

    def __init__(self):
        self.lock = threading.Lock()
        self.playing = True
        self.cost_ms = 0.0
        self.use("live")
        self._stop = False
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def use(self, which: str):
        with getattr(self, "lock", threading.Lock()):
            if which == "live":
                self.source = LiveSource()
                self.twin = self.source.twin
                self.an = Analyzer(self.twin, "forward", detector=True)
            elif which == "isaac":
                self.source = IsaacSource()
                self.twin = load_panda()
                self.an = Analyzer(self.twin, "backward", detector=True)
            else:
                path = REPO / "data" / f"ref_matched_{which}_1000hz.parquet"      # our tracked reference runs
                self.twin = load_panda()
                self.source = ReplaySource(path)
                self.an = Analyzer(self.twin, "backward", detector=False)
            self.which = which
            self.t_run = 0.0

    @property
    def controls(self) -> Controls:
        return self.source.c

    def set_controls(self, **kw):
        c = self.controls
        t = self.an.snapshot.get("t", 0.0)
        if "load" in kw and abs(kw["load"] - c.load) > 0.04:
            self.an.mark_load_change(t)
        if "fault" in kw and kw["fault"] != c.fault:
            self.an.mark_fault_change(t, kw["fault"] != "none")
        if "fault_joint" in kw and kw["fault_joint"] != c.fault_joint and c.fault != "none":
            self.an.mark_fault_change(t, True)
        changed_speeds = "speeds" in kw and list(kw["speeds"]) != list(c.speeds)
        for k, v in kw.items():
            setattr(c, k, v)
        if changed_speeds and self.source.kind == "live":
            self.an.recalibrate()          # the detector's baseline is per speed (A1 in NOTES_FOR_DEV round 4)

    def _loop(self):
        nxt = time.perf_counter()
        while not self._stop:
            nxt += BLOCK_STEPS * 1e-3
            if self.playing:
                t0 = time.perf_counter()
                with self.lock:
                    try:
                        blk = self.source.next_block()
                        if blk is None:                                  # the simulator has not sent a full block yet
                            raise _NoBlock
                        if getattr(self.source, "new_run", False):       # a new pick: start the window and the detector afresh
                            self.source.new_run = False
                            self.an.reset_window()
                            self.an.recalibrate()
                        self.t_run += BLOCK_STEPS * 1e-3
                        if self.source.kind == "replay" and getattr(self.source, "wrapped", False):
                            self.an.reset_window()
                        self.an.feed(blk, self.controls, self.t_run)
                    except _NoBlock:
                        pass
                    except Exception as exc:                      # keep the loop alive; the UI shows the error
                        self.an.snapshot = dict(self.an.snapshot, error=repr(exc))
                self.cost_ms = (time.perf_counter() - t0) * 1e3
            delay = nxt - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                nxt = time.perf_counter()
