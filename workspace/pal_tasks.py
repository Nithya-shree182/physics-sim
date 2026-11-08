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

# The single-object pick takes its geometry from the real cell, so a one-box experiment and the
# three-box cycle are the same scene measured two ways rather than two sets of numbers that can
# drift apart. The object is the cell's first box; it is set down where the cycle sets boxes down.
import pal_cell_geom as _G

BOX_H = _G.BOX[2]                            # m, box height
TABLE_TOP_Z = _G.WORK_Z                      # the infeed table's top face
PICK_XY = _G.box_positions()[0]              # the first box on the infeed table
PLACE_XY = tuple(_G.PLACE_XY)                # over the belt, where the cycle releases
APPROACH_Z = _G.CARRY_Z                      # cup-plane height while carrying
GRASP_DWELL_S = 0.4
# A suction grip holds the box by its TOP FACE, so the point the IK aims is the cup plane and the
# target height is the face itself, not the object's centre. A finger grasp was different: it took
# the cube at its mid-height, which is why this used to be an offset of its own.
GRASP_TOOL_OFFSET = tuple(pal_ik.TOOL_OFFSET)
CUP_CONTACT_GAP = 0.002                      # m, how far above the box face the cups stop
GRASP_LIFT = CUP_CONTACT_GAP


def _urdf(path=None):
    """The active arm's URDF: cfg.URDF_PATH inside the simulator container, the repository copy on the host."""
    if path is None:
        path = cfg.ROBOT.urdf_file()
    return Urdf(str(path), tool_link=cfg.ROBOT.tool_frame)


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
    physical: bool = False            # a real box, really gripped, rather than a mass folded
                                      # into the tool link's inertia
    events: tuple = ()                # ((t, what, box index), ...) where what is one of
                                      # "vacuum_on" | "vacuum_off" | "picked" | "placed".
                                      # This is the ONE way a task drives the gripper. The arm
                                      # used to carry separate finger-close and finger-open
                                      # times, which only worked for a single-object pick; a
                                      # three-box cycle needs a list, and a suction seal is an
                                      # outcome the tool reports rather than a time to assume.
    n_boxes: int = 0

    def vacuum_schedule(self):
        """[(t, on)] from `events`, in time order: what the gripper is commanded to do."""
        return [(ev[0], ev[1] == "vacuum_on") for ev in self.events
                if ev[1] in ("vacuum_on", "vacuum_off")]


def waypoints(urdf=None, pick_xy=PICK_XY, place_xy=PLACE_XY, approach_z=APPROACH_Z, offset=pal_ik.TOOL_OFFSET, lift=0.0):
    """Joint configurations for the pick-and-place, from IK. Raises if a target is unreachable or the arm jumps branch."""
    urdf = urdf or _urdf()
    R, _ = pal_ik.tool_target_from_home(urdf)
    # The cups meet the box's TOP FACE. `lift` holds them a hair clear of it so a solver tolerance
    # cannot drive them into the carton.
    zc = TABLE_TOP_Z + BOX_H + lift
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
    # A physical pick commands the vacuum one dwell before the payload is expected to be on the
    # tool, so the cups have time to seal; the same offset the palletize cycle uses.
    events = ((t_attach - GRASP_DWELL_S, "vacuum_on", 0), (t_attach, "picked", 0),
              (t_release - GRASP_DWELL_S, "vacuum_off", 0), (t_release, "placed", 0)) if physical else ()
    return Task(traj, traj.duration, t_attach, t_release, kw.get("pick_xy", PICK_XY),
                kw.get("place_xy", PLACE_XY), wp,
                f"pick_{style}" + ("_physical" if physical else ""), physical=physical,
                events=events, n_boxes=1 if physical else 0)


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


# ── the conveyor cell's cycle: three boxes, infeed table to moving belt ───────────────────────
VACUUM_DWELL_S = 0.45            # held still while the cups seal, and again while they bleed


def palletize(style, boxes_xy, place_xy, box_top_z, place_top_z, cup_offset,
              approach_z=None, vscale=0.35, ascale=0.35, tfac=2.5, urdf=None, name=None):
    """Pick several boxes off the infeed table and set each one on the conveyor.

    One cycle, in order, for every box: above it, down onto it, vacuum on and hold while the cups
    seal, lift, across to the belt, down, vacuum off and hold while they bleed, lift clear. The arm
    returns home at the end. Each box is placed at the SAME spot on the belt, because the belt has
    carried the previous one away by then — which is what a real singulation cell does.

    All heights are absolute world z. `cup_offset` is the suction tool's own length, the vector
    from the robot's tool frame to the cup faces, and is what makes the IK aim the cups rather than
    the flange. Returns a Task whose `events` say when the vacuum switches, so the simulator can
    drive the gripper straight from the trajectory."""
    urdf = urdf or _urdf()
    R, _ = pal_ik.tool_target_from_home(urdf)
    approach_z = box_top_z + 0.17 if approach_z is None else approach_z
    lift_z = max(approach_z, place_top_z + 0.17)

    def ik(p, seed, what):
        q, ok, err = pal_ik.solve(urdf, np.asarray(p, float), R, seed, offset=cup_offset)
        if not ok:
            raise ValueError(f"IK failed for {what} at {np.round(p, 3).tolist()}: {err}")
        return q

    q = cfg.HOME_JOINTS.copy()
    parts, events, t = [], [], 0.0

    def move(q0, q1, tag):
        nonlocal t
        Tleg = max(0.6, tfac * float(np.abs(q1 - q0).max())) if style == "torque_aware" else None
        part = T.leg(q0, q1, style, T=Tleg, vscale=vscale, ascale=ascale)
        parts.append(part); t += part.duration
        return part

    def dwell(qd, seconds):
        nonlocal t
        part = T.dwell(qd, seconds)
        parts.append(part); t += part.duration

    q_home = cfg.HOME_JOINTS.copy()
    q_prev = q_home
    for i, (bx, by) in enumerate(boxes_xy):
        q_above = ik((bx, by, approach_z), q_prev, f"above box {i}")
        q_on = ik((bx, by, box_top_z + CUP_CONTACT_GAP), q_above, f"on box {i}")
        q_pre_place = ik((place_xy[0], place_xy[1], lift_z), q_above, f"above the belt for box {i}")
        q_place = ik((place_xy[0], place_xy[1], place_top_z + CUP_CONTACT_GAP), q_pre_place, f"placing box {i}")

        move(q_prev, q_above, "to above the box")
        move(q_above, q_on, "down onto the box")
        events.append((t, "vacuum_on", i))
        dwell(q_on, VACUUM_DWELL_S)
        events.append((t, "picked", i))
        move(q_on, q_above, "lift the box")
        move(q_above, q_pre_place, "across to the belt")
        move(q_pre_place, q_place, "down onto the belt")
        events.append((t, "vacuum_off", i))
        dwell(q_place, VACUUM_DWELL_S)
        events.append((t, "placed", i))
        move(q_place, q_pre_place, "clear of the belt")
        q_prev = q_pre_place
    move(q_prev, q_home, "home")

    traj = T.sequence(parts)
    return Task(traj, traj.duration, pick_xy=tuple(boxes_xy[0]), place_xy=tuple(place_xy),
                waypoints={"home": q_home}, name=name or f"palletize_{style}", physical=True,
                events=tuple(events), n_boxes=len(boxes_xy))


def cell_cycle(style="s_curve", n_boxes=None, speed=1.0, band=None, urdf=None, name=None):
    """THE cycle this project runs: the conveyor cell's boxes, picked and placed in order.

    One definition, shared by the watched demo (pal_cell_demo), the live server (pal_live) and the
    recorded suites, so a number quoted from one is a number about the others. Every height comes
    from pal_cell_geom; nothing here is a literal.

    `speed` is the user-facing knob, 0 to 1, and `band` is the fraction of the datasheet velocity
    limit that 1.0 on that knob means — cfg.SPEED_BAND, which is 0.5 (DECISIONS.md D1). So the arm
    cannot be commanded past half its velocity limit however the knob is set, which is the regime
    the residual is trustworthy in.
    """
    import pal_cell_geom as G
    import pal_suction as SU_const                    # constants only; this import pulls in no USD

    band = cfg.SPEED_BAND if band is None else band
    if not 0.0 < speed <= 1.0:
        raise ValueError(f"speed must be in (0, 1], got {speed}")
    # 1% under the band. An s-curve's velocity profile overshoots its nominal peak by a hair, so
    # commanding exactly the band puts the realised peak a fraction above it and the cycle is
    # rejected by its own speed check. Measured, not guessed: without this the s_curve cycle at
    # speed 1.0 lands at 50.0...% of a 50% band.
    v = band * speed * 0.99
    return palletize(
        style,
        tuple(G.box_positions(G.N_BOXES if n_boxes is None else n_boxes)),
        tuple(G.PLACE_XY),
        box_top_z=G.BOX_TOP_Z,
        place_top_z=G.PLACE_TOP_Z,
        approach_z=G.CARRY_Z,
        cup_offset=(0.0, 0.0, SU_const.CUP_PLANE_Z),
        vscale=v, ascale=v,
        # A torque-aware leg takes tfac seconds per radian of the largest joint move, so it has to
        # scale with the speed knob or the style stops responding to it at all.
        tfac=2.5 * band / v,
        urdf=urdf,
        name=name or f"cell_{style}")
