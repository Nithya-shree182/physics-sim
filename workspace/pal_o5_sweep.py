#!/usr/bin/env python3
"""O5 sweep: inject faults into the plant and record how the arm copes.

One process, one World build. Each scenario resets the arm to the start of the trajectory,
runs DURATION_S simulated seconds, and switches the fault on at ONSET_S, so every run has a
healthy stretch followed by a faulty one (fault_active flips in the telemetry). Speed is
fixed at 20% of velocity limit, inside the <=50% regime where the residual floor is physical
(docs/RESIDUAL_FLOOR.md).

Faults, not attacks (decided 2026-11-03): friction x2/x4/x8 on a joint, a locked joint, and
command delay. The plant changes; the analysis side's model does not.

Writes one telemetry run per scenario to out/o5/ plus out/o5/summary.json, and prints a table
of how much the fault disturbed tracking and joint torque. Convert the runs with
tools/npz_to_parquet.py on the host. The summary uses simulator terms only; it is a
quick look at the plant's response, not the detector.

    tools/run_sim.sh pal_o5_sweep.py
"""
import json
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
print("[PAL] pal_o5_sweep.py running", flush=True)

from pxr import PhysxSchema
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation
import pal_config as cfg
from pal_telemetry import TelemetryWriter
from pal_faults import FaultSpec, FaultInjector

ARM = 7
HZ = float(os.environ.get("PAL_PHYSICS_HZ", cfg.PHYSICS_HZ))
DT = 1.0 / HZ
TRAJ = "slow_reference"          # 20% of velocity limit: inside the <=50% regime
PAYLOAD = 0.0
MJ_EVERY = int(os.environ.get("PAL_MJ_EVERY", 10))
DURATION_S = float(os.environ.get("PAL_DURATION_S", 6.0))
ONSET_S = float(os.environ.get("PAL_ONSET_S", 3.0))
OUT_DIR = "/workspace/out/o5"
TCP_Z = 0.1034                      # m along panda_hand z; matches the analysis side

world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=1/30.0)
pc = world.get_physics_context()
pc.set_solver_type(cfg.SOLVER_TYPE); pc.enable_ccd(cfg.ENABLE_CCD)
pc.enable_stablization(cfg.ENABLE_STABILIZATION); pc.set_gravity(cfg.GRAVITY)
sp = PhysxSchema.PhysxSceneAPI.Apply(pc.get_current_physics_scene_prim())
sp.CreateMinPositionIterationCountAttr(cfg.POSITION_ITERATIONS)
sp.CreateMinVelocityIterationCountAttr(cfg.VELOCITY_ITERATIONS)

add_reference_to_stage(usd_path=get_assets_root_path() + cfg.ROBOT_USD_SUBPATH,
                       prim_path=cfg.ROBOT_PRIM)
art = Articulation(cfg.ROBOT_PRIM)
world.reset(); art.initialize()
idx = np.arange(ARM)

# ── URDF inertials, plus the payload folded into panda_hand at the TCP ───────────────
root = ET.parse(cfg.URDF_PATH).getroot()
U = {}
for l in root.findall("link"):
    it = l.find("inertial")
    if it is None:
        continue
    m, o, I = it.find("mass"), it.find("origin"), it.find("inertia")
    U[l.get("name")] = {"mass": float(m.get("value")),
        "com": np.array([float(x) for x in o.get("xyz").split()])
               if o is not None and o.get("xyz") else np.zeros(3),
        "diag": np.array([float(I.get("ixx")), float(I.get("iyy")), float(I.get("izz"))])}

bodies = list(art.body_names); nb = len(bodies)
cur = np.asarray(art.get_body_masses()).reshape(nb)
masses, coms, diags = [], [], []
for i, b in enumerate(bodies):
    if b not in U:
        masses.append(float(cur[i])); coms.append(np.zeros(3)); diags.append(np.full(3, 1e-3))
        continue
    m0, c0, d0 = U[b]["mass"], U[b]["com"], U[b]["diag"]
    if b == "panda_hand" and PAYLOAD > 0:
        # Composite of hand + point mass at the TCP: combined mass, combined COM, and the
        # payload's parallel-axis contribution about that combined COM.
        cp = np.array([0.0, 0.0, TCP_Z])
        m = m0 + PAYLOAD
        c = (m0 * c0 + PAYLOAD * cp) / m
        dh, dp = c0 - c, cp - c
        d = d0 + m0 * (np.dot(dh, dh) - dh**2) + PAYLOAD * (np.dot(dp, dp) - dp**2)
        masses.append(m); coms.append(c); diags.append(d)
    else:
        masses.append(m0); coms.append(c0); diags.append(d0)

masses = np.array(masses, np.float32)
coms = np.array(coms, np.float32)
diags = np.array(diags, np.float32)
quats = np.tile(np.array([1., 0, 0, 0], np.float32), (nb, 1))

art.set_gains(kps=np.full((1, ARM), 400.0 * cfg.DEG_PER_RAD, np.float32),
              kds=np.full((1, ARM), 80.0 * cfg.DEG_PER_RAD, np.float32), joint_indices=idx)
art.set_body_masses(masses.reshape(1, nb))
art.set_body_coms(coms.reshape(1, nb, 3), quats.reshape(1, nb, 4))
try:
    art.set_body_inertias(diags.reshape(1, nb, 3))
except Exception:
    f = np.zeros((nb, 9), np.float32); f[:, 0], f[:, 4], f[:, 8] = diags.T
    art.set_body_inertias(f.reshape(1, nb, 9))
art.set_armatures(cfg.ARMATURE.reshape(1, ARM).astype(np.float32), joint_indices=idx)
fric = cfg.FRICTION_COULOMB_STATIC * cfg.PLANT_FRICTION_SCALE
art.set_friction_coefficients(fric.reshape(1, ARM).astype(np.float32), joint_indices=idx)


F0 = np.array([.55, .45, .60, .50, .75, .70, .80])
K = sum(1. / (h + 1) for h in range(3))
A = 0.20 * cfg.QD_MAX / (2 * np.pi * F0 * K)


def traj(t):
    q = cfg.HOME_JOINTS.copy(); qd = np.zeros(ARM); qdd = np.zeros(ARM)
    for h in range(3):
        w = 2 * np.pi * F0 * (h + 1); a = A / (h + 1) ** 2
        q += a * np.sin(w * t); qd += a * w * np.cos(w * t); qdd -= a * w * w * np.sin(w * t)
    return q, qd, qdd


SCENARIOS = [FaultSpec("none")]
for mult in (2.0, 4.0, 8.0):
    for j in (1, 3, 5):
        SCENARIOS.append(FaultSpec("friction", j, mult, ONSET_S))
SCENARIOS.append(FaultSpec("stuck_joint", 3, 200.0, ONSET_S))
for ms in (10.0, 25.0, 50.0):
    SCENARIOS.append(FaultSpec("delay", -1, ms, ONSET_S))

inj = FaultInjector(art, fric, HZ)
q0 = traj(0.0)[0]
steps = int(DURATION_S * HZ)
onset_k = int(ONSET_S * HZ)
os.makedirs(OUT_DIR, exist_ok=True)
summary = []


def run(spec):
    inj.set_spec(spec)
    art.set_joint_positions(np.concatenate([q0, [.04, .04]]))
    art.set_joint_velocities(np.zeros((1, 9)))
    art.set_joint_position_targets(np.concatenate([q0, [.04, .04]]).reshape(1, -1))
    for _ in range(int(0.4 * HZ)):
        world.step(render=False)
    run_id = f"o5_{spec.tag}_{int(HZ)}hz"
    writer = TelemetryWriter(
        art, run_id, out_dir=OUT_DIR, ee_body_index=cfg.EE_JACOBIAN_ROW,
        ee_prim=f"{cfg.ROBOT_PRIM}/panda_hand", mj_every=MJ_EVERY,
        labels={"traj_type": TRAJ, "drive_mode": "position_stiff", "payload_kg": PAYLOAD,
                "payload_attached": False, "fault_type": spec.type,
                "fault_joint": spec.joint, "fault_param": spec.param, "attack_type": "none"},
        manifest={"physics_hz": HZ, "armature": cfg.ARMATURE.tolist(),
                  "friction_source": cfg.FRICTION_SOURCE,
                  "plant_friction_scale": cfg.PLANT_FRICTION_SCALE,
                  "friction_values": fric.tolist(), "torque_convention": cfg.TORQUE_CONVENTION,
                  "fault": {"type": spec.type, "joint": spec.joint, "param": spec.param,
                            "onset_s": spec.onset_s},
                  "speed_fraction_of_limit": 0.20, "isaac_sim_version": "6.0.1"})
    for k in range(steps):
        qt, qdt, qddt = traj(k / HZ)
        active = inj.update(k / HZ)
        tgt = inj.delay_target(np.concatenate([qt, [.04, .04]]))
        art.set_joint_position_targets(tgt.reshape(1, -1))
        world.step(render=False)
        writer.capture(k, (k + 1) / HZ, 1.0 / HZ, q_cmd=qt, qd_cmd=qdt, qdd_cmd=qddt,
                       fault_active=active)
    rows = writer.rows
    arr = lambda p: np.array([[r[f"{p}_{i}"] for i in range(ARM)] for r in rows])
    q, qc, tm = arr("q"), arr("q_cmd"), arr("tau_measured")
    pre, post = slice(int(1.0 * HZ), onset_k), slice(onset_k + int(0.5 * HZ), steps)
    err = np.abs(q - qc)
    out = {"run_id": run_id, "type": spec.type, "joint": spec.joint, "param": spec.param,
           "track_err_pre_rad": err[pre].max(axis=0).tolist(),
           "track_err_post_rad": err[post].max(axis=0).tolist(),
           "tau_rms_pre": np.sqrt((tm[pre] ** 2).mean(axis=0)).tolist(),
           "tau_rms_post": np.sqrt((tm[post] ** 2).mean(axis=0)).tolist(),
           "q_travel_post_rad": np.ptp(q[post], axis=0).tolist()}
    writer.close()
    inj.clear()
    return out


print(f"\n  {len(SCENARIOS)} scenarios | {DURATION_S:g}s each, fault at {ONSET_S:g}s | "
      f"{HZ:.0f} Hz | mj_every {MJ_EVERY}", flush=True)
print(f"  {'scenario':<26}{'track err pre':>15}{'track err post':>16}{'tau rms change':>16}"
      f"{'  joint':>7}", flush=True)
for spec in SCENARIOS:
    r = run(spec); summary.append(r)
    j = spec.joint if spec.joint >= 0 else int(np.argmax(np.array(r["track_err_post_rad"])))
    pre, post = r["track_err_pre_rad"][j], r["track_err_post_rad"][j]
    dt_ = r["tau_rms_post"][j] - r["tau_rms_pre"][j]
    print(f"  {spec.tag:<26}{pre:>14.4f} {post:>15.4f} {dt_:>+15.3f}  j{j}", flush=True)

with open(f"{OUT_DIR}/summary.json", "w") as f:
    json.dump({"hz": HZ, "duration_s": DURATION_S, "onset_s": ONSET_S, "runs": summary}, f, indent=2)
print(f"\n  written: out/o5/summary.json and {len(SCENARIOS)} runs", flush=True)
simulation_app.close()
