#!/usr/bin/env python3
"""One integrated experiment on the real arm, watchable in the viewport.

Measures rather than assumes. Phases:

  1  every reading the articulation exposes, at rest
  2  each joint moved on its own, so its torque response is not confounded by the others
  3  a velocity sweep at sane speeds (10-50% of the datasheet limit)
  4  torque required to hold and to move 0.5 kg and 1.0 kg
  5  an unknown 2 kg object, weighed from torque alone
  6  pose confidence using the analysis side's scaled formulation, not the raw ratio

Speed: the default working speed is 30% of each joint's datasheet velocity limit. Fast
enough that M qdd and C qd matter, slow enough to stay below the ~50% knee where the
residual floor climbs steeply.

Physics rate is taken from PAL_PHYSICS_HZ so the same script can be run at several rates
in separate processes — never two rates in one process, which previously produced a
reversed and wrong trend.
"""
import json
import os
import xml.etree.ElementTree as ET

import numpy as np
from isaacsim import SimulationApp

HEADLESS = os.environ.get("ISAAC_SIM_HEADLESS", "1") != "0"
simulation_app = SimulationApp({"headless": HEADLESS, "width": 1280, "height": 720,
                                "renderer": "RaytracedLighting", "anti_aliasing": 3})
print("[PAL] pal_experiment.py running", flush=True)

from pxr import PhysxSchema, UsdLux, UsdGeom, Gf, Usd, UsdShade
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation
import pal_config as cfg

ARM = 7
HZ = float(os.environ.get("PAL_PHYSICS_HZ", 500.0))
DT = 1.0 / HZ
WORK_FRAC = float(os.environ.get("PAL_FRAC", 0.30))
G = 9.81
L_SCALE = 0.472                 # characteristic length for the Jacobian row scaling
FORCE_GAIN_MAX = 10.0           # analysis side's physical gate, N per N*m
VERTICAL_GAIN_MAX = 10.0
TCP_Z = 0.1034
results = {"physics_hz": HZ, "work_frac": WORK_FRAC}


def banner(t):
    print("\n" + "=" * 92, flush=True); print(f"  {t}", flush=True); print("=" * 92, flush=True)


# ── scene ─────────────────────────────────────────────────────────────────────────────
world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=1/30.0)
stage = world.stage
pc = world.get_physics_context()
pc.set_solver_type(cfg.SOLVER_TYPE); pc.enable_ccd(cfg.ENABLE_CCD)
pc.enable_stablization(cfg.ENABLE_STABILIZATION); pc.set_gravity(cfg.GRAVITY)
sp = PhysxSchema.PhysxSceneAPI.Apply(pc.get_current_physics_scene_prim())
sp.CreateMinPositionIterationCountAttr(cfg.POSITION_ITERATIONS)
sp.CreateMinVelocityIterationCountAttr(cfg.VELOCITY_ITERATIONS)

L = cfg.LIGHTS
d = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome"); d.CreateIntensityAttr(L["dome"]["intensity"])
ov = UsdLux.DiskLight.Define(stage, "/World/Lights/Over")
UsdGeom.Xformable(ov).AddTranslateOp().Set(Gf.Vec3d(*L["overhead"]["position"]))
ov.CreateRadiusAttr(L["overhead"]["radius"]); ov.CreateIntensityAttr(L["overhead"]["intensity"])
ky = UsdLux.DistantLight.Define(stage, "/World/Lights/Key")
UsdGeom.Xformable(ky).AddRotateXYZOp().Set(Gf.Vec3f(*L["key"]["rotation"]))
ky.CreateIntensityAttr(L["key"]["intensity"])
world.scene.add_default_ground_plane(z_position=cfg.GROUND_PLANE_Z)
for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/defaultGroundPlane")):
    if prim.IsA(UsdShade.Shader):
        for n in ("diffuse_tint", "diffuse_color_constant"):
            inp = UsdShade.Shader(prim).GetInput(n)
            if inp and inp.GetAttr().IsValid():
                inp.Set(Gf.Vec3f(*cfg.GROUND_TINT))
world.scene.add(FixedCuboid(prim_path="/World/WorkTable", name="work_table",
                            position=np.array(cfg.TABLE_POSITION), scale=np.array(cfg.TABLE_SCALE),
                            color=np.array(cfg.TABLE_COLOR)))

add_reference_to_stage(usd_path=get_assets_root_path()+cfg.ROBOT_USD_SUBPATH, prim_path=cfg.ROBOT_PRIM)
art = Articulation(cfg.ROBOT_PRIM); world.reset(); art.initialize()
idx = np.arange(ARM)

# ── author the plant from the URDF (after reset; a reset discards tensor-API writes) ──
root = ET.parse(cfg.URDF_PATH).getroot(); U = {}
for l in root.findall("link"):
    it = l.find("inertial")
    if it is None: continue
    m, o, I = it.find("mass"), it.find("origin"), it.find("inertia")
    U[l.get("name")] = {"mass": float(m.get("value")),
        "com": np.array([float(x) for x in o.get("xyz").split()]) if o is not None and o.get("xyz") else np.zeros(3),
        "diag": np.array([float(I.get("ixx")), float(I.get("iyy")), float(I.get("izz"))])}
BODIES = list(art.body_names); NB = len(BODIES)
_cur = np.asarray(art.get_body_masses()).reshape(NB)


def author(payload=0.0):
    """Write URDF inertials, folding any payload into panda_hand as a point mass at the TCP."""
    ms, cs, ds = [], [], []
    for i, b in enumerate(BODIES):
        if b not in U:
            ms.append(float(_cur[i])); cs.append(np.zeros(3)); ds.append(np.full(3, 1e-3)); continue
        m0, c0, d0 = U[b]["mass"], U[b]["com"], U[b]["diag"]
        if b == "panda_hand" and payload > 0:
            cp = np.array([0.0, 0.0, TCP_Z]); m = m0 + payload
            c = (m0*c0 + payload*cp)/m
            dh, dp = c0-c, cp-c
            ds.append(d0 + m0*(np.dot(dh, dh)-dh**2) + payload*(np.dot(dp, dp)-dp**2))
            ms.append(m); cs.append(c)
        else:
            ms.append(m0); cs.append(c0); ds.append(d0)
    ms = np.array(ms, np.float32); cs = np.array(cs, np.float32); ds = np.array(ds, np.float32)
    qs = np.tile(np.array([1., 0, 0, 0], np.float32), (NB, 1))
    art.set_body_masses(ms.reshape(1, NB)); art.set_body_coms(cs.reshape(1, NB, 3), qs.reshape(1, NB, 4))
    try: art.set_body_inertias(ds.reshape(1, NB, 3))
    except Exception:
        f = np.zeros((NB, 9), np.float32); f[:, 0], f[:, 4], f[:, 8] = ds.T
        art.set_body_inertias(f.reshape(1, NB, 9))
    art.set_armatures(cfg.ARMATURE.reshape(1, ARM).astype(np.float32), joint_indices=idx)
    art.set_friction_coefficients(np.zeros((1, ARM), np.float32), joint_indices=idx)


art.set_gains(kps=np.full((1, ARM), 400.0*cfg.DEG_PER_RAD, np.float32),
              kds=np.full((1, ARM), 80.0*cfg.DEG_PER_RAD, np.float32), joint_indices=idx)
author(0.0)
HOME9 = np.concatenate([cfg.HOME_JOINTS, [.04, .04]])
RENDER_EVERY = max(1, int(HZ/30))


def step(n, render=True):
    for i in range(n):
        world.step(render=(render and not HEADLESS and i % RENDER_EVERY == 0))


def settle(q7=None, n=None):
    q = HOME9 if q7 is None else np.concatenate([q7, [.04, .04]])
    art.set_joint_positions(q)
    art.set_joint_position_targets(q.reshape(1, -1))
    step(int(1.2*HZ) if n is None else n)


def read():
    return dict(
        q=np.asarray(art.get_joint_positions()).flatten()[:ARM].copy(),
        qd=np.asarray(art.get_joint_velocities()).flatten()[:ARM].copy(),
        g=np.asarray(art.get_generalized_gravity_forces()).flatten()[:ARM].copy(),
        cor=np.asarray(art.get_coriolis_and_centrifugal_forces()).flatten()[:ARM].copy(),
        M=np.asarray(art.get_mass_matrices()).reshape(9, 9)[:ARM, :ARM].copy(),
        tau=np.asarray(art.get_measured_joint_efforts()).flatten()[:ARM].copy(),
        J=np.asarray(art.get_jacobians()).reshape(1, -1, 6, 9)[0, cfg.EE_JACOBIAN_ROW][:, :ARM].copy())


def conditioning(J, length=L_SCALE):
    """Analysis side's formulation (observers/confidence.py), reproduced here."""
    Js = np.diag([1., 1., 1., length, length, length]) @ J
    Uu, s, _ = np.linalg.svd(Js, full_matrices=False)
    s_min, s_max = float(s[-1]), float(s[0])
    g_z = float(np.linalg.norm(Uu[2, :]/s))
    return {"c": s_min/s_max, "c_vertical": 1.0/(g_z*s_max),
            "force_gain": 1.0/s_min, "vertical_gain": g_z,
            "sigma_min": s_min, "sigma_max": s_max,
            "raw_c": float(np.linalg.svd(J, compute_uv=False)[-1] /
                           np.linalg.svd(J, compute_uv=False)[0])}


# ══════════════════════════════════════════════════════════════════════════════════════
banner(f"PHASE 1 — every reading at rest, home pose   ({HZ:.0f} Hz)")
settle()
r = read()
print(f"  {'joint':<8}{'q (rad)':>10}{'qd':>9}{'g(q)':>10}{'tau_meas':>11}"
      f"{'residual':>11}{'M_ii':>10}{'tau_lim':>9}")
for i in range(ARM):
    print(f"  j{i+1:<7}{r['q'][i]:>10.4f}{r['qd'][i]:>9.4f}{r['g'][i]:>10.4f}"
          f"{r['tau'][i]:>11.4f}{r['tau'][i]-r['g'][i]:>11.5f}{r['M'][i,i]:>10.4f}"
          f"{cfg.TAU_MAX[i]:>9.0f}")
print(f"\n  total arm mass {np.asarray(art.get_body_masses()).sum():.4f} kg "
      f"| M condition number {np.linalg.cond(r['M']):.0f}")
print(f"  at rest the model reduces to g(q); worst residual "
      f"{np.max(np.abs(r['tau']-r['g'])):.5f} N*m "
      f"({100*np.max(np.abs(r['tau']-r['g'])/cfg.TAU_MAX):.3f}% of limit)")
results["phase1"] = {k: v.tolist() for k, v in r.items() if k != "M" and k != "J"}

# ══════════════════════════════════════════════════════════════════════════════════════
banner("PHASE 2 — each joint moved alone (+0.30 rad, 2 s), torque response isolated")
print("  NOTE: the last column is a PEAK residual, not the RMS used in phase 3 — a slow")
print("  single-joint move still shows a large transient at motion onset. Not comparable.\n")
print(f"  {'joint':<8}{'dq':>8}{'tau before':>12}{'tau after':>11}{'d tau':>10}"
      f"{'peak |tau|':>12}{'peak qd':>9}{'PEAK resid %':>15}")
phase2 = []
for j in range(ARM):
    settle()
    base = read()
    tgt = cfg.HOME_JOINTS.copy()
    dq = 0.30 if (cfg.HOME_JOINTS[j] + 0.30) < (cfg.Q_MAX[j] - 0.15) else -0.30
    tgt[j] += dq
    T = int(2.0*HZ)
    qp = np.zeros(ARM); pk_tau = 0.0; pk_qd = 0.0; worst = 0.0
    for k in range(T):
        a = 0.5*(1 - np.cos(np.pi*min(1.0, k/(0.7*T))))      # smooth ramp
        q = cfg.HOME_JOINTS.copy(); q[j] += dq*a
        art.set_joint_position_targets(np.concatenate([q, [.04, .04]]).reshape(1, -1))
        world.step(render=(not HEADLESS and k % RENDER_EVERY == 0))
        s_ = read()
        qdd = (s_["qd"]-qp)/DT; qp = s_["qd"].copy()
        pk_tau = max(pk_tau, abs(s_["tau"][j])); pk_qd = max(pk_qd, abs(s_["qd"][j]))
        if k > 0.2*T:
            res = s_["tau"] - (s_["M"]@qdd + s_["cor"] + s_["g"])
            worst = max(worst, float(np.max(np.abs(res)/cfg.TAU_MAX)*100))
    end = read()
    print(f"  j{j+1:<7}{dq:>8.2f}{base['tau'][j]:>12.3f}{end['tau'][j]:>11.3f}"
          f"{end['tau'][j]-base['tau'][j]:>10.3f}{pk_tau:>12.3f}{pk_qd:>9.3f}{worst:>14.2f}%")
    phase2.append({"joint": j+1, "dq": dq, "d_tau": float(end['tau'][j]-base['tau'][j]),
                   "peak_tau": pk_tau, "peak_qd": pk_qd, "worst_resid_pct": worst})
results["phase2"] = phase2

# ══════════════════════════════════════════════════════════════════════════════════════
banner("PHASE 3 — velocity sweep at sane speeds (all joints, multi-sine)")
F0 = np.array([.55, .45, .60, .50, .75, .70, .80]); KH = sum(1./(h+1) for h in range(3))


def sweep_run(frac, seconds=2.5):
    A = frac*cfg.QD_MAX/(2*np.pi*F0*KH)
    def tr(t):
        q = cfg.HOME_JOINTS.copy()
        for h in range(3): q += (A/(h+1)**2)*np.sin(2*np.pi*F0*(h+1)*t)
        return q
    settle(tr(0.0), n=int(0.5*HZ))
    T = int(seconds*HZ); warm = int(0.4*HZ)
    qp = np.zeros(ARM); R, QD = [], []
    for k in range(T):
        art.set_joint_position_targets(np.concatenate([tr(k*DT), [.04, .04]]).reshape(1, -1))
        world.step(render=(not HEADLESS and k % RENDER_EVERY == 0))
        s_ = read()
        qdd = (s_["qd"]-qp)/DT; qp = s_["qd"].copy()
        if k > warm:
            R.append(s_["tau"] - (s_["M"]@qdd + s_["cor"] + s_["g"])); QD.append(s_["qd"].copy())
    R = np.array(R); QD = np.array(QD)
    rms = np.sqrt(np.mean(R**2, axis=0))
    return rms, 100*rms/cfg.TAU_MAX, float(np.max(np.abs(QD)))


print(f"  {'speed':>7}{'qd peak':>10}{'worst joint %':>15}{'mean RMS N*m':>15}"
      f"{'payload 3-sigma (kg)':>22}")
phase3 = []
for frac in (0.10, 0.20, 0.30, 0.40, 0.50):
    rms, pct, qdpk = sweep_run(frac)
    kg3 = 3*rms[3]/4.6306
    print(f"  {frac:>6.0%}{qdpk:>10.2f}{np.max(pct):>14.2f}%{np.mean(rms):>15.4f}{kg3:>22.3f}")
    phase3.append({"frac": frac, "qd_peak": qdpk, "worst_pct": float(np.max(pct)),
                   "mean_rms": float(np.mean(rms)), "payload_3sigma_kg": float(kg3)})
results["phase3"] = phase3

# ══════════════════════════════════════════════════════════════════════════════════════
banner("PHASE 4 — torque required for a 0.5 kg and a 1.0 kg object")
print("  Static hold at home, then the same multi-sine at the working speed.\n")
print(f"  {'payload':>9}{'g j2':>10}{'g j4':>10}{'g j6':>10}{'tau j4':>10}"
      f"{'d tau j4':>11}{'N*m per kg':>13}")
phase4 = []
base_g = None
for kg in (0.0, 0.5, 1.0):
    author(kg); settle()
    s_ = read()
    if base_g is None: base_g = s_["g"].copy()
    d4 = s_["g"][3]-base_g[3]
    print(f"  {kg:>8.1f}kg{s_['g'][1]:>10.3f}{s_['g'][3]:>10.3f}{s_['g'][5]:>10.3f}"
          f"{s_['tau'][3]:>10.3f}{d4:>11.3f}{(d4/kg if kg else 0):>13.3f}")
    phase4.append({"kg": kg, "g": s_["g"].tolist(), "tau": s_["tau"].tolist(),
                   "d_tau_j4": float(d4)})
print(f"\n  {'payload':>9}{'worst RMS resid %':>19}{'mean RMS N*m':>15}  (moving at "
      f"{WORK_FRAC:.0%} of limit)")
for kg in (0.0, 0.5, 1.0):
    author(kg)
    rms, pct, qdpk = sweep_run(WORK_FRAC)
    print(f"  {kg:>8.1f}kg{np.max(pct):>18.2f}%{np.mean(rms):>15.4f}")
    phase4.append({"kg": kg, "moving_worst_pct": float(np.max(pct)),
                   "moving_mean_rms": float(np.mean(rms))})
results["phase4"] = phase4

# ══════════════════════════════════════════════════════════════════════════════════════
banner("PHASE 5 — an UNKNOWN 2 kg object, weighed from torque alone")
print("  The twin's model is the no-payload one; the residual against it is the signal.")
print("  Measured at several poses, since the lever arm changes with configuration.\n")
UNKNOWN = 2.0
POSES = {"home": cfg.HOME_JOINTS,
         "folded": cfg.PROBE_POSES["folded"],
         "horizontal_reach": cfg.PROBE_POSES["horizontal_reach"]}
nominal = {}
author(0.0)
for name, q7 in POSES.items():
    settle(q7); s_ = read()
    nominal[name] = (s_["g"].copy(), s_["J"].copy())

print(f"  {'pose':<20}{'c(q)':>8}{'c_vert':>9}{'Fz (N)':>10}{'est kg':>9}"
      f"{'err %':>9}  gate")
author(UNKNOWN)
phase5 = []
for name, q7 in POSES.items():
    g_nom, _ = nominal[name]
    settle(q7); s_ = read()
    r_ = s_["tau"] - g_nom
    cond = conditioning(s_["J"])
    Js = np.diag([1., 1., 1., L_SCALE, L_SCALE, L_SCALE]) @ s_["J"]
    W = -np.linalg.pinv(Js.T) @ r_
    est = abs(W[2])/G
    gated = cond["vertical_gain"] > VERTICAL_GAIN_MAX
    print(f"  {name:<20}{cond['c']:>8.4f}{cond['c_vertical']:>9.4f}{W[2]:>10.3f}"
          f"{est:>9.4f}{100*(est-UNKNOWN)/UNKNOWN:>8.2f}%  "
          f"{'PAUSED' if gated else 'accepted'}")
    phase5.append({"pose": name, "est_kg": float(est), "true_kg": UNKNOWN,
                   "c": cond["c"], "c_vertical": cond["c_vertical"], "gated": bool(gated)})
results["phase5"] = phase5

# ══════════════════════════════════════════════════════════════════════════════════════
banner("PHASE 6 — O3 pose confidence (analysis side's scaled formulation)")
print(f"  Raw c = sigma_min/sigma_max on the 6x7 Jacobian tops out near 0.13 because its")
print(f"  linear rows are m/rad and angular rows rad/rad. Scaling the angular rows by")
print(f"  L = {L_SCALE} m makes every row metres. c_vertical is the direction-specific")
print(f"  score for O4, since the mass estimate uses only the vertical force.")
print(f"  Gates are physical: force error per N*m of torque error, limit {FORCE_GAIN_MAX:.0f} N/N*m.\n")
author(0.0)
print(f"  {'pose':<24}{'raw c':>8}{'c scaled':>10}{'c_vert':>9}"
      f"{'force gain':>12}{'vert gain':>11}  detector   payload")
phase6 = []
for name, q7 in cfg.PROBE_POSES.items():
    settle(q7); s_ = read()
    c = conditioning(s_["J"])
    det_ok = c["force_gain"] <= FORCE_GAIN_MAX
    pay_ok = c["vertical_gain"] <= VERTICAL_GAIN_MAX
    print(f"  {name:<24}{c['raw_c']:>8.4f}{c['c']:>10.4f}{c['c_vertical']:>9.4f}"
          f"{c['force_gain']:>12.2f}{c['vertical_gain']:>11.2f}  "
          f"{'ok' if det_ok else 'GATED':<11}{'ok' if pay_ok else 'GATED'}")
    phase6.append({"pose": name, **{k: float(v) for k, v in c.items()},
                   "detector_ok": bool(det_ok), "payload_ok": bool(pay_ok)})
results["phase6"] = phase6

with open("/workspace/out/experiment.json", "w") as f:
    json.dump(results, f, indent=2, default=str)
banner("DONE")
print(f"  physics rate {HZ:.0f} Hz | working speed {WORK_FRAC:.0%} of velocity limit")
print(f"  written: out/experiment.json")
if not HEADLESS:
    print("\n  Window open — holding for 20 s so the scene can be inspected.", flush=True)
    settle(); step(int(20*HZ))
simulation_app.close()
