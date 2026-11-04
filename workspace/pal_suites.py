"""Every objective's scenarios, as data. No Isaac import, so this is unit-tested and the same list drives windowed and headless runs.

A suite is (output_directory, [Scenario, ...]). pal_run_suite.py plays suites in one process; tools/pal_suite.sh runs them and checks
the results. Names, seeds and parameters are the ones the earlier per-objective scripts used, so regenerated runs match the existing data.

  reference  O1/O2  slow reference at 0 and 1 kg, matched plant (no mass mismatch), no noise: calibration data for the model   -> data/
  o3         O3     holds at the five probe poses (two near-singular) and a pass through the extended singular pose, 0 and 1 kg -> out/o3
  o4         O4     known payloads held at three poses and carried along the reference motion                                  -> out/o4
  o5         O5     26 runs: six healthy noise seeds, friction x2/x4/x8 on four joints, locked joint, backlash, delay          -> out/o5
  clean      O5     noise-free twins of the key faults                                                                          -> out/clean
  fast       O5     faults at 40% of the velocity limit                                                                         -> out/fast
  extra      O4/O5  the analysis side's run list: excitation, matched C4 faults at 0 and 1 kg over three seeds, 60 s, pickup   -> out/extra
  push       7.7    pushes with a known force in low-gain torque mode, and a friction-off control                              -> out/push
  o6         O6     three movement styles over a speed grid at 0 and 2 kg (out-and-back between two joint configurations)       -> out/o6
  pick       O4/O5  a real pick-and-place from IK waypoints in three styles at 0.5 and 2 kg, an ellipse, faults after the grasp    -> out/pick
  o6pick     O6     the same pick cycle over a speed grid: cycle time, peak torque, energy per pick                              -> out/o6pick
  pickphys   O4     a REAL pick: fingers close on a physical cube of known mass, friction holds it, contact carries its weight   -> out/pickphys
"""
import numpy as np

import pal_config as cfg
import pal_tasks as K
import pal_trajectories as T
from pal_faults import FaultSpec
from pal_runner import PushSpec, Scenario

OFFSET = np.array([0.6, -0.3, 0.3, 0.4, 0.3, -0.4, 0.3])     # the O6 "pick" displacement
Q_HOME, Q_PICK = cfg.HOME_JOINTS, cfg.HOME_JOINTS + OFFSET


def _kg(kg):
    return f"{kg:g}kg".replace(".", "p")


def reference():
    return [Scenario(name=f"ref_matched_{_kg(kg)}", traj=T.multisine(0.20), traj_type="slow_reference", duration_s=6.0,
                     payload_kg=kg, mass_scale=1.0, noise_seed=None) for kg in (0.0, 1.0)]


def o3():
    runs = []
    for i, (pose, q) in enumerate(cfg.PROBE_POSES.items()):
        for kg in (0.0, 1.0):
            runs.append(Scenario(name=f"o3_hold_{pose}_{_kg(kg)}", traj=T.hold(q), traj_type="hold", duration_s=3.0,
                                 payload_kg=kg, noise_seed=4000 + 2 * i + int(kg)))
    # a slow pass home -> extended (elbow) singular pose -> home carrying 1 kg, so the confidence dip is seen in motion
    near = cfg.PROBE_POSES["near_singular_extended"]
    for kg in (0.0, 1.0):
        runs.append(Scenario(name=f"o3_pass_singular_{_kg(kg)}", traj=T.cycle(Q_HOME, near, "torque_aware", T=3.0, dwell=0.5),
                             traj_type="torque_aware", duration_s=T.cycle(Q_HOME, near, "torque_aware", T=3.0, dwell=0.5).duration + 0.3,
                             payload_kg=kg, noise_seed=4100 + int(kg)))
    return runs


def o4():
    runs, i = [], 0
    for pose in ("home", "horizontal_reach", "folded"):
        for kg in (0.0, 0.5, 1.0, 2.0, 3.0):
            runs.append(Scenario(name=f"o4_hold_{pose}_{_kg(kg)}", traj=T.hold(cfg.PROBE_POSES[pose]), traj_type="hold",
                                 duration_s=3.0, payload_kg=kg, noise_seed=2000 + i))
            i += 1
    for kg in (0.0, 0.5, 1.0, 3.0):
        runs.append(Scenario(name=f"o4_move_{_kg(kg)}", traj=T.multisine(0.20), traj_type="slow_reference", duration_s=6.0,
                             payload_kg=kg, noise_seed=2000 + i))
        i += 1
    return runs


def o5():
    onset = 3.0
    specs = [FaultSpec("none")] * 6
    specs += [FaultSpec("friction", j, m, onset) for m in (2.0, 4.0, 8.0) for j in (1, 2, 3, 5)]
    specs += [FaultSpec("stuck_joint", 3, 200.0, onset)]
    specs += [FaultSpec("backlash", 0, w, onset) for w in (0.02, 0.06, 0.12)] + [FaultSpec("backlash", 6, 0.12, onset)]
    specs += [FaultSpec("delay", 0, s, onset) for s in (0.010, 0.030, 0.050)]
    runs, nh = [], 0
    for f in specs:
        if f.type == "none":
            name, seed = f"o5_healthy_s{nh}", 1000 + nh        # s0 shares seed 1000 with every faulty run: its exact twin
            nh += 1
        else:
            name, seed = f"o5_{f.tag}", 1000
        runs.append(Scenario(name=name, traj=T.multisine(0.20), traj_type="slow_reference", duration_s=6.0, fault=f,
                             noise_seed=seed))
    return runs


def clean():
    specs = [FaultSpec("none")]
    specs += [FaultSpec("friction", j, m, 3.0) for j in (1, 3) for m in (2.0, 4.0, 8.0)]
    specs += [FaultSpec("stuck_joint", 3, 200.0, 3.0), FaultSpec("delay", 0, 0.050, 3.0)]
    return [Scenario(name=f"clean_{f.tag}", traj=T.multisine(0.20), duration_s=6.0, fault=f, noise_seed=None) for f in specs]


def fast(frac=0.40):
    specs = [FaultSpec("none"),
             FaultSpec("friction", 3, 2.0, 3.0), FaultSpec("friction", 3, 4.0, 3.0), FaultSpec("friction", 3, 8.0, 3.0),
             FaultSpec("backlash", 0, 0.02, 3.0), FaultSpec("backlash", 0, 0.12, 3.0), FaultSpec("backlash", 6, 0.12, 3.0),
             FaultSpec("delay", 0, 0.010, 3.0), FaultSpec("delay", 0, 0.030, 3.0), FaultSpec("stuck_joint", 3, 200.0, 3.0)]
    return [Scenario(name=f"fast_{f.tag}", traj=T.multisine(frac), traj_type="excitation", duration_s=6.0, fault=f, noise_seed=1000)
            for f in specs]


def extra():
    onset, slow = 3.0, T.multisine(0.20)
    runs = [Scenario(name=f"c2_excitation_{_kg(kg)}", traj=T.multisine(0.50), traj_type="excitation", duration_s=8.0,
                     payload_kg=kg, noise_seed=3001) for kg in (0.0, 1.0)]
    faults = [FaultSpec("friction", 2, 3.0, onset), FaultSpec("friction", 5, 3.0, onset),
              FaultSpec("stuck_joint", 3, 200.0, onset), FaultSpec("delay", 0, 0.050, onset)]
    for seed in (1000, 2001, 2002):
        runs.append(Scenario(name=f"c4_healthy_0kg_s{seed}", traj=slow, duration_s=6.0, noise_seed=seed))
        runs += [Scenario(name=f"c4_{f.tag}_0kg_s{seed}", traj=slow, duration_s=6.0, fault=f, noise_seed=seed) for f in faults]
    runs.append(Scenario(name="c4b_healthy_1kg_s1000", traj=slow, duration_s=6.0, payload_kg=1.0, noise_seed=1000))
    runs += [Scenario(name=f"c4b_{f.tag}_1kg_s1000", traj=slow, duration_s=6.0, payload_kg=1.0, fault=f, noise_seed=1000)
             for f in faults]
    runs.append(Scenario(name="c5_healthy_60s", traj=slow, duration_s=60.0, noise_seed=3000))
    runs.append(Scenario(name="c6_pickup_0to1kg", traj=slow, duration_s=6.0, noise_seed=3002, payload_events=((3.0, 1.0),)))
    return runs


def push():
    runs = []
    for friction_scale, label, poses, mags in ((1.0, "", ("home", "horizontal_reach", "folded"), (10.0, 20.0)),
                                               (0.0, "nofric_", ("home", "folded"), (10.0,))):
        for pose in poses:
            for axis, e in (("x", (1, 0, 0)), ("y", (0, 1, 0)), ("z", (0, 0, 1))):
                for mag in mags:
                    runs.append(Scenario(name=f"push_{label}{pose}_{axis}_{mag:g}N", traj=T.hold(cfg.PROBE_POSES[pose]),
                                         traj_type="push", duration_s=1.7, gains="low", mass_scale=1.0,
                                         friction_scale=friction_scale, push=PushSpec(tuple(mag * np.array(e, float)), 0.4, 1.5)))
    return runs


def o6():
    """Three styles over a speed grid, at 0 and 2 kg. Names and knobs as in the first benchmark."""
    grid = {"trapezoidal": [dict(ascale=a) for a in (1.0, 0.6, 0.4, 0.25)],
            "s_curve": [dict(ascale=a) for a in (1.0, 0.6, 0.4, 0.25)],
            "torque_aware": [dict(T=t) for t in (0.45, 0.6, 0.8, 1.1, 1.5)]}
    runs = []
    for kg in (0.0, 2.0):
        for kind, kws in grid.items():
            for i, kw in enumerate(kws):
                tr = T.cycle(Q_HOME, Q_PICK, kind, **kw)
                runs.append(Scenario(name=f"o6_{kind}_{_kg(kg)}_{i}", traj=tr, traj_type=kind, duration_s=tr.duration + 0.3,
                                     payload_kg=kg, mass_scale=1.0, noise_seed=None))
    return runs


def pick():
    """A real pick-and-place (IK waypoints, three trajectory styles) at 0.5 and 2 kg, an elliptical path with a held payload, and
    faults that start just after the grasp. Speeds stay under ~40% of the limit, inside the regime where the residual is trusted."""
    runs = []
    for kg in (0.5, 2.0):
        for style in ("trapezoidal", "s_curve", "torque_aware"):
            task = K.pick_place(style)
            runs.append(Scenario(name=f"pick_{style}_{_kg(kg)}", traj=task.traj, traj_type="pick", duration_s=task.duration + 0.3,
                                 payload_events=((task.t_attach, kg), (task.t_release, 0.0)), noise_seed=5000, task=task))
    task = K.pick_place("s_curve")
    # a healthy reference of the same task with a DIFFERENT noise seed: the detector can calibrate against it (a pick does not repeat, so
    # the first 3 s never sees the carry), and because the noise differs it cannot cancel against the run being tested
    runs.append(Scenario(name="pick_s_curve_1kg_ref", traj=task.traj, traj_type="pick", duration_s=task.duration + 0.3,
                         payload_events=((task.t_attach, 1.0), (task.t_release, 0.0)), noise_seed=5001, task=task))
    runs.append(Scenario(name="pick_s_curve_1kg", traj=task.traj, traj_type="pick", duration_s=task.duration + 0.3,
                         payload_events=((task.t_attach, 1.0), (task.t_release, 0.0)), noise_seed=5000, task=task))
    onset = task.t_attach + 0.4
    for f in (FaultSpec("friction", 3, 8.0, onset), FaultSpec("friction", 5, 8.0, onset),
              FaultSpec("stuck_joint", 3, 200.0, onset), FaultSpec("delay", 0, 0.050, onset)):
        runs.append(Scenario(name=f"pick_fault_{f.tag}_1kg", traj=task.traj, traj_type="pick", duration_s=task.duration + 0.3,
                             payload_events=((task.t_attach, 1.0), (task.t_release, 0.0)), fault=f, noise_seed=5000, task=task))
    ell = K.ellipse()
    for kg in (0.0, 1.0):
        runs.append(Scenario(name=f"ellipse_{_kg(kg)}", traj=ell.traj, traj_type="ellipse", duration_s=ell.duration + 0.3,
                             payload_kg=kg, noise_seed=5100, task=None))
    return runs


def pickphys(masses=(0.5, 1.0, 2.0)):
    """A pick with a REAL cube: the fingers close on it, friction holds it, its weight reaches the joints through contact. The cube's
    mass is the payload; the plant's inertials are not rewritten (the telemetry labels are). Three styles at each mass, then faults that
    start just after the grasp with a 1 kg cube (the healthy twin shares the noise seed). Needs the gripper plant."""
    runs = []

    def scen(name, task, kg, fault=None, seed=5200):
        return Scenario(name=name, traj=task.traj, traj_type="pick", duration_s=task.duration + 0.5, payload_physical=True,
                        payload_events=((task.t_attach, kg), (task.t_open, 0.0)), cube_kg=kg, noise_seed=seed, task=task,
                        **({"fault": fault} if fault else {}))
    for style in ("trapezoidal", "s_curve", "torque_aware"):
        task = K.pick_place(style, physical=True)
        runs += [scen(f"pickphys_{style}_{_kg(kg)}", task, kg) for kg in masses]
    task = K.pick_place("s_curve", physical=True)
    runs.append(scen("pickphys_s_curve_1kg_ref", task, 1.0, seed=5201))      # healthy reference, different noise seed (see pick())
    onset = task.t_attach + 0.4
    for f in (FaultSpec("friction", 3, 8.0, onset), FaultSpec("friction", 5, 8.0, onset),
              FaultSpec("stuck_joint", 3, 200.0, onset), FaultSpec("stuck_joint", 0, 200.0, onset),
              FaultSpec("delay", 0, 0.050, onset)):
        # joint 4 hardly moves on this path, so locking it is a mild fault (a test of sensitivity); locking joint 1, which does the
        # carry sweep, is the destructive one
        runs.append(scen(f"pickphys_fault_{f.tag}_1kg", task, 1.0, fault=f))
    return runs


FAR_PICKS = [(0.74, -0.25), (0.78, 0.0)]       # base-frame xy of a cube near the end of the arm's reach: the detector's pose gate closes there


def pickfar(masses=(0.5, 1.0, 2.0)):
    """A real-cube pick with the cube at the far end of the table, so the arm is stretched at the grasp (force noise gain 16 against
    a gate of 10; the payload gate, on the vertical gain, stays open). Same recipe as pickphys(): s_curve, grasp, carry, release."""
    runs = []
    for xy, kgs in ((FAR_PICKS[0], masses), (FAR_PICKS[1], (2.0,))):
        task = K.pick_place("s_curve", physical=True, pick_xy=xy)
        for kg in kgs:
            runs.append(Scenario(name=f"pickfar_x{round(xy[0] * 100)}y{round(xy[1] * 100):+d}_{_kg(kg)}".replace("+", "p").replace("-", "m"),
                                 traj=task.traj, traj_type="pick", duration_s=task.duration + 0.5, payload_physical=True,
                                 payload_events=((task.t_attach, kg), (task.t_open, 0.0)), cube_kg=kg, noise_seed=5300, task=task))
    # faults at the far spot, for the detector's gate: a healthy reference (another noise seed) and faults after the grasp; the 1 kg run
    # above (same seed 5300) is their healthy twin
    task = K.pick_place("s_curve", physical=True, pick_xy=FAR_PICKS[0])
    onset = task.t_attach + 0.4
    base = dict(traj=task.traj, traj_type="pick", duration_s=task.duration + 0.5, payload_physical=True,
                payload_events=((task.t_attach, 1.0), (task.t_open, 0.0)), cube_kg=1.0, task=task)
    runs.append(Scenario(name="pickfar_x74ym25_1kg_ref", noise_seed=5301, **base))
    for f in (FaultSpec("friction", 3, 8.0, onset), FaultSpec("friction", 5, 8.0, onset),
              FaultSpec("stuck_joint", 0, 200.0, onset), FaultSpec("delay", 0, 0.050, onset)):
        runs.append(Scenario(name=f"pickfar_fault_{f.tag}_1kg", noise_seed=5300, fault=f, **base))
    return runs


LIVE_SPEED_CAP = 0.5                                                                        # fraction of each joint's velocity limit


def peak_speed_fraction(task, step_s=0.02):
    """Highest joint speed of a task's trajectory, as a fraction of that joint's velocity limit."""
    ts = np.arange(0.0, task.duration, step_s)
    return float(max(np.max(np.abs(task.traj(t)[1]) / cfg.QD_MAX) for t in ts))


LIVE_FAULTS = {"friction": 8.0, "stuck_joint": 200.0, "delay": 0.050}       # what the dashboard's fault choices mean in the simulator


def random_pick_xy(rng, tries=60):
    """A random cube position on the table that is a little away from the fixed demo spot (0.51 m from the base) but well short of the
    arm's reach: 0.53-0.65 m out, on the side away from the place point, kept only if the IK solves. Never stretched far enough to
    close the confidence gates (tests/test_suites.py checks it over the whole pick)."""
    for _ in range(tries):
        r, th = rng.uniform(0.53, 0.65), rng.uniform(np.radians(-65.0), 0.0)
        xy = (round(float(r * np.cos(th)), 3), round(float(r * np.sin(th)), 3))
        if xy[0] < 0.35:
            continue
        try:
            K.waypoints(pick_xy=xy, offset=K.GRASP_TOOL_OFFSET, lift=K.GRASP_LIFT)
        except ValueError:
            continue
        return xy
    return tuple(K.PICK_XY)


def live_pick(style="s_curve", kg=1.0, fault="none", joint=3, name="live_pick", speed=0.7, pick_xy=None):
    """One real pick for the live dashboard: the physical cube at `kg`, the given style, and optionally a fault that starts 0.4 s after the grasp.
    Same scenario recipe as pickphys(), without sensor noise (the live server adds its own to what it streams).
    `speed` is the dashboard's speed bar as a fraction (1.0 = the bar's 100% = LIVE_SPEED_CAP of the velocity limit); 0.7 is the original 35%."""
    if style not in ("trapezoidal", "s_curve", "torque_aware"):
        raise ValueError(f"unknown style {style!r}")
    if not 0.2 <= kg <= 3.0:
        raise ValueError("cube mass must be 0.2-3 kg")
    if not 0.05 <= speed <= 1.0:
        raise ValueError("speed must be 0.05-1.0 of the speed bar")
    v = LIVE_SPEED_CAP * speed * 0.99                 # 1% under the cap: an s-curve overshoots its nominal peak by a hair
    task = K.pick_place(style, physical=True, vscale=v, ascale=v, tfac=2.5 * 0.35 / v, **({"pick_xy": tuple(pick_xy)} if pick_xy else {}))
    peak = peak_speed_fraction(task)
    if peak > LIVE_SPEED_CAP:                      # the live demo stays in the trusted regime (DECISIONS.md D1)
        raise ValueError(f"{style} pick peaks at {peak:.0%} of the velocity limit; the live cap is {LIVE_SPEED_CAP:.0%}")
    f = FaultSpec() if fault == "none" else FaultSpec(fault, 0 if fault == "delay" else int(joint), LIVE_FAULTS[fault], task.t_attach + 0.4)
    return Scenario(name=name, traj=task.traj, traj_type="pick", duration_s=task.duration + 0.5, payload_physical=True,
                    payload_events=((task.t_attach, kg), (task.t_open, 0.0)), cube_kg=kg, task=task, fault=f)


def o6pick():
    """O6 on a real pick cycle: cycle time, peak torque and energy per pick, for each style over a speed grid, at 0 and 2 kg."""
    grid = {"trapezoidal": [dict(vscale=v, ascale=v) for v in (1.0, 0.6, 0.35)],
            "s_curve": [dict(vscale=v, ascale=v) for v in (1.0, 0.6, 0.35)],
            "torque_aware": [dict(tfac=t) for t in (0.9, 1.5, 2.5)]}
    runs = []
    for kg in (0.0, 2.0):
        for style, kws in grid.items():
            for i, kw in enumerate(kws):
                task = K.pick_place(style, **kw)
                runs.append(Scenario(name=f"o6pick_{style}_{_kg(kg)}_{i}", traj=task.traj, traj_type=style,
                                     duration_s=task.duration + 0.3, mass_scale=1.0,
                                     payload_events=((task.t_attach, kg), (task.t_release, 0.0)) if kg else (), task=task))
    return runs


SUITES = {"reference": ("data", reference), "o3": ("out/o3", o3), "o4": ("out/o4", o4), "o5": ("out/o5", o5),
          "clean": ("out/clean", clean), "fast": ("out/fast", fast), "extra": ("out/extra", extra),
          "push": ("out/push", push), "o6": ("out/o6", o6), "pick": ("out/pick", pick), "o6pick": ("out/o6pick", o6pick),
          "pickphys": ("out/pickphys", pickphys), "pickfar": ("out/pickfar", pickfar)}
ORDER = ["reference", "o3", "o4", "o5", "clean", "fast", "extra", "push", "o6", "pick", "o6pick", "pickphys", "pickfar"]
NEEDS_GRIPPER = {"pickphys", "pickfar"}       # suites that need the gripper asset variant, finger drives and a physical cube; they cannot share
                                   # a process with the others (a different asset variant would change the other suites' physics)
