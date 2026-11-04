"""Tasks as trajectories: pick-and-place and an elliptical path, built with IK. Pure numpy, no Isaac import.

A task is Cartesian (where the fingertips go); the controller needs joint trajectories. IK (pal_ik) turns the Cartesian waypoints into
joint configurations, and the trajectory STYLE (trapezoidal, S-curve, torque-aware, from pal_trajectories) moves between them. So one
IK serves every style and every payload: the style and the speed limits are properties of the joint-space legs, not of the IK.

  pick_place(style)   home -> above the cube -> descend -> grasp -> lift -> carry -> lower -> release -> retreat -> home
  ellipse(...)        the fingertips trace an ellipse in a horizontal plane; IK is solved at every millisecond along it

The "grasp" is modelled, not actuated: at the end of the grasp dwell the payload mass appears at the tool-centre point (the simulator's
inertials are rewritten, as for every payload run) and a visual cube, when drawn, follows the hand until the release. The gripper
fingers stay open; there is no contact model.
"""
import functools
from dataclasses import dataclass

import numpy as np

import pal_config as cfg
import pal_ik
import pal_trajectories as T
from pal_kinematics import Urdf

CUBE = cfg.PAYLOAD_SIZE                      # m, cube edge
TABLE_TOP_Z = 0.0
PICK_XY = (0.45, -0.25)                      # m, in the robot base frame
PLACE_XY = (0.45, 0.25)
APPROACH_Z = 0.22                            # m, tool height for the carry
GRASP_DWELL_S = 0.4
# Physical grasp. Measured in the simulator (pal_gripper_probe.py): the finger pads span 0.0584-0.1124 m along the hand's z axis, so a
# 0.05 m cube is held with its centre ~0.09 m from the hand origin, which leaves the fingertips ~2 mm above a table the cube rests on; the
# 4 mm lift below raises them a little more. (The modelled pick keeps the TCP at 0.1034 m.)
GRASP_TOOL_OFFSET = (0.0, 0.0, 0.09)
GRASP_LIFT = 0.004


def _urdf(path=None):
    """The Franka URDF: cfg.URDF_PATH inside the simulator container, the repository copy on the host."""
    import os
    from pathlib import Path
    if path is None:
        path = cfg.URDF_PATH if os.path.exists(cfg.URDF_PATH) else Path(__file__).resolve().parents[1] / "assets" / "franka_panda" / "panda_arm_hand.urdf"
    return Urdf(str(path))


@dataclass
class Task:
    traj: object
    duration: float
    t_attach: float = None            # when the payload appears at the tool-centre point
    t_release: float = None
    pick_xy: tuple = None
    place_xy: tuple = None
    waypoints: dict = None
    name: str = ""
    physical: bool = False            # a real cube and real finger contact (the fingers close at t_close and open at t_open)
    t_close: float = None
    t_open: float = None


def waypoints(urdf=None, pick_xy=PICK_XY, place_xy=PLACE_XY, approach_z=APPROACH_Z, offset=pal_ik.TOOL_OFFSET, lift=0.0):
    """Joint configurations for the pick-and-place, from IK. Raises if a target is unreachable or the arm jumps branch."""
    urdf = urdf or _urdf()
    R, _ = pal_ik.tool_target_from_home(urdf)
    zc = TABLE_TOP_Z + CUBE / 2 + lift                         # the grasp point is at the cube's centre when grasping
    pts = {"pre_pick": (*pick_xy, approach_z), "pick": (*pick_xy, zc), "pre_place": (*place_xy, approach_z),
           "place": (*place_xy, zc + 0.003)}
    order = ["pre_pick", "pick", "pre_place", "place"]
    q = cfg.HOME_JOINTS.copy()
    out = {"home": cfg.HOME_JOINTS.copy()}
    # pre_pick -> pick and pre_place -> place are short vertical moves: solve each from its predecessor so the branch is kept
    for name in ("pre_pick", "pick"):
        q, ok, err = pal_ik.solve(urdf, np.array(pts[name]), R, q, offset=offset)
        if not ok:
            raise ValueError(f"IK failed for {name}: {err}")
        out[name] = q.copy()
    for name in ("pre_place", "place"):
        q, ok, err = pal_ik.solve(urdf, np.array(pts[name]), R, out["pre_pick"] if name == "pre_place" else q, offset=offset)
        if not ok:
            raise ValueError(f"IK failed for {name}: {err}")
        out[name] = q.copy()
    for a, b in (("home", "pre_pick"), ("pre_pick", "pick"), ("pre_pick", "pre_place"), ("pre_place", "place")):
        if np.abs(out[b] - out[a]).max() > 2.0:
            raise ValueError(f"the arm changes branch between {a} and {b}: {np.round(out[b] - out[a], 2).tolist()}")
    return out


def pick_place(style, vscale=0.35, ascale=0.35, tfac=2.5, urdf=None, physical=False, **kw):
    """The pick-and-place in the given style. Speed limits (vscale, ascale as fractions of the joint limits) keep it inside the 50%
    regime where the residual is trustworthy; torque-aware legs take tfac seconds per radian of the largest joint move."""
    if physical:
        kw.setdefault("offset", GRASP_TOOL_OFFSET)
        kw.setdefault("lift", GRASP_LIFT)
    wp = waypoints(urdf, **kw)
    legs = [("home", "pre_pick"), ("pre_pick", "pick"), None, ("pick", "pre_pick"), ("pre_pick", "pre_place"),
            ("pre_place", "place"), None, ("place", "pre_place"), ("pre_place", "home")]
    parts, t, t_attach, t_release, n_dwell = [], 0.0, None, None, 0
    for item in legs:
        if item is None:
            q = wp["pick"] if n_dwell == 0 else wp["place"]
            part = T.dwell(q, GRASP_DWELL_S)
            n_dwell += 1
            t += part.duration
            if n_dwell == 1:
                t_attach = t
            else:
                t_release = t
        else:
            q0, q1 = wp[item[0]], wp[item[1]]
            Tleg = max(0.6, tfac * float(np.abs(q1 - q0).max())) if style == "torque_aware" else None
            part = T.leg(q0, q1, style, T=Tleg, vscale=vscale, ascale=ascale)
            t += part.duration
        parts.append(part)
    traj = T.sequence(parts)
    return Task(traj, traj.duration, t_attach, t_release, kw.get("pick_xy", PICK_XY), kw.get("place_xy", PLACE_XY), wp,
                f"pick_{style}" + ("_physical" if physical else ""), physical=physical,
                t_close=t_attach - GRASP_DWELL_S if physical else None, t_open=t_release - GRASP_DWELL_S if physical else None)


@functools.lru_cache(maxsize=4)
def ellipse(center=(0.42, 0.0, 0.30), a=0.12, b=0.08, cycles=2, duration=12.0, hz=1000.0, urdf=None):
    """The fingertips trace an ellipse (semi-axes a along x, b along y) in the horizontal plane z = center[2], gripper pointing down.

    The phase runs through a quintic so the path starts and ends at rest. A joint-space leg brings the arm from home to the ellipse's
    start and another returns it; IK is solved at every sample, warm-started, to a tolerance that keeps qdd smooth."""
    urdf = urdf or _urdf()
    R, _ = pal_ik.tool_target_from_home(urdf)
    n = int(duration * hz) + 1
    u = np.linspace(0.0, 1.0, n)
    phase = 2 * np.pi * cycles * (10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5)
    pts = np.stack([center[0] + a * np.cos(phase), center[1] + b * np.sin(phase), np.full(n, center[2])], axis=1)
    q_start, ok, err = pal_ik.solve(urdf, pts[0], R, cfg.HOME_JOINTS)
    if not ok:
        raise ValueError(f"IK failed at the ellipse start: {err}")
    q_path = pal_ik.solve_path(urdf, pts, R, q_start, tol_pos=1e-10, tol_rot=1e-9)
    approach = T.leg(cfg.HOME_JOINTS, q_path[0], "s_curve", vscale=0.35, ascale=0.35)
    ret = T.leg(q_path[-1], cfg.HOME_JOINTS, "s_curve", vscale=0.35, ascale=0.35)
    traj = T.sequence([approach, T.from_samples(q_path, hz), ret])
    return Task(traj, traj.duration, name="ellipse", waypoints={"start": q_path[0]})
