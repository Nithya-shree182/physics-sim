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
  payload    O4     heavy payload ladder (0-9 kg moving, to 12.5 kg held) at 20/35/50% speed                          -> out/payload
  pickphys   O4     a REAL pick: fingers close on a physical cube of known mass, friction holds it, contact carries its weight   -> out/pickphys
"""
import numpy as np

import pal_config as cfg
import pal_tasks as K
import pal_trajectories as T
from pal_faults import FaultSpec
from pal_runner import PushSpec, Scenario

OFFSET = np.array([0.6, -0.3, 0.3, 0.4, 0.3, -0.4, 0.3])[:cfg.DOF]   # the O6 "pick" displacement
Q_HOME, Q_PICK = cfg.HOME_JOINTS, cfg.HOME_JOINTS + OFFSET


# The wrist joint, by index. Written as the arm's last rather than a number so a fault aimed at
# "the far end of the chain" stays aimed there when the arm changes.
LAST_JOINT = cfg.DOF - 1


def _kg(kg):
    return f"{kg:g}kg".replace(".", "p")


def reference():
    return [Scenario(name=f"ref_matched_{_kg(kg)}", traj=T.multisine(0.20), traj_type="slow_reference", duration_s=6.0,
                     payload_kg=kg, mass_scale=1.0, noise_seed=None) for kg in (0.0, 1.0)]


def o3():
    """UR10e evidence suite: a plain hold at the active home pose with the two payloads the model
    still checks as a clean baseline. The older Panda configuration had several probe poses and a
    near-singular pass; the active UR10e configuration exposes a single home pose, so the suite keeps
    the same clinical purpose without inventing a stale robot-specific pose set."""
    return [Scenario(name=f"o3_hold_home_{_kg(kg)}", traj=T.hold(cfg.HOME_JOINTS), traj_type="hold",
                     duration_s=3.0, payload_kg=kg, noise_seed=1500 + i)
            for i, kg in enumerate((0.0, 1.0))]


def o4():
    runs, i = [], 0
    for pose in cfg.PROBE_POSES:          # the arm's named poses; the UR10e has only home
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
    # Backlash on the base joint across three widths, plus one on the LAST joint: the two ends of
    # the chain are the interesting cases, because backlash at the base moves the whole arm while
    # backlash at the wrist moves only the tool. `LAST_JOINT` rather than a literal 6, which was the
    # Panda's last index and is out of range on a six-joint arm — pal_faults now rejects it instead
    # of applying the fault to a joint that does not exist.
    specs += [FaultSpec("backlash", 0, w, onset) for w in (0.02, 0.06, 0.12)] + [FaultSpec("backlash", LAST_JOINT, 0.12, onset)]
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
             FaultSpec("backlash", 0, 0.02, 3.0), FaultSpec("backlash", 0, 0.12, 3.0), FaultSpec("backlash", LAST_JOINT, 0.12, 3.0),
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
    # Poses are whatever the arm names (cfg.PROBE_POSES). The UR10e has only home.
    named = tuple(cfg.PROBE_POSES)
    for friction_scale, label, poses, mags in ((1.0, '', named, (10.0, 20.0)),
                                               (0.0, 'nofric_', named, (10.0,))):
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


def payload():
    """O4 at the heavy end: known payloads, moving at 20/35/50% of the velocity limit and held at home.

    The ladder shortens as speed rises because the multisine from home is a stretched-out pose and
    torque saturates: the largest payload at each speed keeps the peak model torque under ~90% of the
    joint limits (host check, 2026-11-08), so a run never measures the drive's ceiling by accident.
    Rated payload is 12.5 kg; only the holds go that high (85% of limit at home). No noise, matched
    plant, like the reference runs, so any model error is the model's."""
    ladders = {0.20: (0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 6.0, 8.0, 9.0),
               0.35: (0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 6.0, 7.0),
               0.50: (0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 6.0)}
    runs = []
    for frac, kgs in ladders.items():
        tr = T.multisine(frac)
        for kg in kgs:
            runs.append(Scenario(name=f"payload_v{round(frac * 100)}_{_kg(kg)}", traj=tr, traj_type="slow_reference",
                                 duration_s=6.0, payload_kg=kg, mass_scale=1.0, noise_seed=None))
    for kg in (0.0, 2.0, 4.0, 8.0, 10.0, 12.5):
        runs.append(Scenario(name=f"payload_hold_{_kg(kg)}", traj=T.hold(cfg.HOME_JOINTS), traj_type="hold",
                             duration_s=3.0, payload_kg=kg, mass_scale=1.0, noise_seed=None))
    return runs


def _fault_grid():
    """Every fault the detector is asked about, on every joint the arm has, at magnitudes that scale with the arm.

    friction multiplies each joint's own baseline, which is 1% of that joint's effort limit, so x2/x4/x8 add
    about 1/3/7% of the limit on EVERY joint, wrist or shoulder alike. A stuck joint is locked hard. Backlash
    is the play between motor and link in radians. Delay is a command delay of every joint."""
    specs = []
    for j in range(cfg.DOF):
        specs += [FaultSpec("friction", j, m, 3.0) for m in (2.0, 4.0, 8.0)]
        specs.append(FaultSpec("stuck_joint", j, 200.0, 3.0))
        specs += [FaultSpec("backlash", j, w, 3.0) for w in (0.04, 0.12)]
    specs += [FaultSpec("delay", 0, d, 3.0) for d in (0.010, 0.030, 0.050)]
    return specs


def _detect_runs(prefix, speed, fault_seed, healthy_seeds):
    tr = T.multisine(speed)
    runs = [Scenario(name=f"{prefix}_healthy_s{fault_seed}", traj=tr, traj_type="slow_reference", duration_s=6.0, noise_seed=fault_seed)]
    runs += [Scenario(name=f"{prefix}_healthy_s{s}", traj=tr, traj_type="slow_reference", duration_s=6.0, noise_seed=s) for s in healthy_seeds]
    runs += [Scenario(name=f"{prefix}_{f.tag}", traj=tr, traj_type="slow_reference", duration_s=6.0, fault=f, noise_seed=fault_seed)
             for f in _fault_grid()]
    return runs


def detect():
    """Detector tuning and its held-out test, in one process. `dtune_*` is the only data thresholds may be fitted to;
    `dtest_*` has other noise seeds and is scored once, after the choice is made. 20% speed."""
    return (_detect_runs("dtune", 0.20, 1000, range(1010, 1018)) +
            _detect_runs("dtest", 0.20, 2000, range(2010, 2018)))


def detectfast():
    """The same faults at 40% of the velocity limit, a third noise seed: does a tuned detector survive faster motion."""
    return _detect_runs("dfast", 0.40, 3000, range(3010, 3013))


def detectlimits():
    """How big must backlash and delay be before the tuned detector sees them? The `detect` suite's widths (0.04, 0.12 rad)
    and its 10 ms delay were mostly missed; these go larger, on every joint, so the smallest visible size can be stated
    rather than guessed. Noise seed 4000, 20% speed."""
    tr = T.multisine(0.20)
    specs = [FaultSpec("backlash", j, w, 3.0) for j in range(cfg.DOF) for w in (0.20, 0.35, 0.50)]
    specs += [FaultSpec("delay", 0, d, 3.0) for d in (0.015, 0.020, 0.040)]
    runs = [Scenario(name="dlim_healthy_s4000", traj=tr, traj_type="slow_reference", duration_s=6.0, noise_seed=4000)]
    runs += [Scenario(name=f"dlim_{f.tag}", traj=tr, traj_type="slow_reference", duration_s=6.0, fault=f, noise_seed=4000) for f in specs]
    return runs


SPEED_GRID = (0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.75, 1.00)


def speedband():
    """The residual floor against speed on THIS arm (D1's evidence, which was the Panda's). The reference multisine at eight speeds
    from 10% to 100% of the velocity limit, empty gripper, matched plant, ideal sensing: whatever the residual does is the model
    against the simulator and nothing else. Run it at more than one physics rate (PAL_PHYSICS_HZ) to see whether the residual
    moves with the rate, which is what marked the Panda's artifact above ~50%."""
    return [Scenario(name=f"speed_v{round(f * 100):03d}", traj=T.multisine(f), traj_type="slow_reference", duration_s=6.0,
                     payload_kg=0.0, mass_scale=1.0, noise_seed=None) for f in SPEED_GRID]


def pickdetect(kg=1.0):
    """Fault detection on the PICK cycle, where a healthy run of the same task is the baseline. The multisine tuning (`detect`) does not
    carry over: a grasp and a release put transients in the residual that a fixed calibration window never sees, and the first look at
    the tuned multisine settings false-alarmed on every healthy pick. So this tunes the pick rule on its own data.

      pref_*           one healthy reference (seed 5299), the baseline every run is compared with
      ptune_*          faults at seed 5300 and eight healthy seeds   <- the only data a setting is fitted to
      ptest_*          faults at seed 5400 and eight other healthy seeds  <- scored after the choice

    A fault starts 0.4 s after the grasp, as in `pickphys`. Needs the cell and the suction tool."""
    task = K.pick_place("s_curve", physical=True)
    onset = task.t_attach + 0.4

    def scen(name, seed, fault=None):
        return Scenario(name=name, traj=task.traj, traj_type="pick", duration_s=task.duration + 0.5, payload_physical=True,
                        payload_events=((task.t_attach, kg), (task.t_release, 0.0)), box_kg=kg, noise_seed=seed, task=task,
                        **({"fault": fault} if fault else {}))
    specs = []
    for j in range(cfg.DOF):
        specs += [FaultSpec("friction", j, m, onset) for m in (2.0, 4.0, 8.0)] + [FaultSpec("stuck_joint", j, 200.0, onset)]
    specs.append(FaultSpec("delay", 0, 0.050, onset))
    runs = [scen("pref_healthy_s5299", 5299)]
    for pre, fseed, hseeds in (("ptune", 5300, range(5301, 5309)), ("ptest", 5400, range(5401, 5409))):
        runs.append(scen(f"{pre}_healthy_s{fseed}", fseed))
        runs += [scen(f"{pre}_healthy_s{h}", h) for h in hseeds]
        runs += [scen(f"{pre}_{f.tag}", fseed, f) for f in specs]
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
                        payload_events=((task.t_attach, kg), (task.t_release, 0.0)), box_kg=kg, noise_seed=seed, task=task,
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
                                 payload_events=((task.t_attach, kg), (task.t_release, 0.0)), box_kg=kg, noise_seed=5300, task=task))
    # faults at the far spot, for the detector's gate: a healthy reference (another noise seed) and faults after the grasp; the 1 kg run
    # above (same seed 5300) is their healthy twin
    task = K.pick_place("s_curve", physical=True, pick_xy=FAR_PICKS[0])
    onset = task.t_attach + 0.4
    base = dict(traj=task.traj, traj_type="pick", duration_s=task.duration + 0.5, payload_physical=True,
                payload_events=((task.t_attach, 1.0), (task.t_release, 0.0)), box_kg=1.0, task=task)
    runs.append(Scenario(name="pickfar_x74ym25_1kg_ref", noise_seed=5301, **base))
    for f in (FaultSpec("friction", 3, 8.0, onset), FaultSpec("friction", 5, 8.0, onset),
              FaultSpec("stuck_joint", 0, 200.0, onset), FaultSpec("delay", 0, 0.050, onset)):
        runs.append(Scenario(name=f"pickfar_fault_{f.tag}_1kg", noise_seed=5300, fault=f, **base))
    return runs


# The trusted speed band (DECISIONS.md D1), as a fraction of each joint's velocity limit. Lives in
# pal_config so the dashboard, the cell demo and the suites cannot disagree about it.
LIVE_SPEED_CAP = cfg.SPEED_BAND


def peak_speed_fraction(task, step_s=0.02):
    """Highest joint speed of a task's trajectory, as a fraction of that joint's velocity limit."""
    ts = np.arange(0.0, task.duration, step_s)
    return float(max(np.max(np.abs(task.traj(t)[1]) / cfg.QD_MAX) for t in ts))


LIVE_FAULTS = {"friction": 8.0, "stuck_joint": 200.0, "delay": 0.050}       # what the dashboard's fault choices mean in the simulator


def live_cycle(style="s_curve", kg=None, fault="none", joint=3, name="live_cycle", speed=0.7,
               n_boxes=None):
    """One real cell cycle for the live dashboard: the conveyor cell's boxes at `kg` each, in the
    given style, with an optional fault 0.4 s after the first box is gripped.

    This replaced live_pick, which was a single cube picked by two fingers. The cell's cycle is the
    run the project now has, so the live view and the recorded suites exercise the same motion.

    `speed` is the dashboard's speed bar as a fraction: 1.0 on the bar is cfg.SPEED_BAND of the
    datasheet velocity limit (DECISIONS.md D1), so the arm cannot be driven out of the trustworthy
    regime from the dashboard however the bar is set."""
    import pal_cell_geom as G
    if style not in ("trapezoidal", "s_curve", "torque_aware"):
        raise ValueError(f"unknown style {style!r}")
    kg = G.BOX_DEFAULT_KG if kg is None else float(kg)
    if not 0.2 <= kg <= cfg.PAYLOAD_RATED_MAX_KG:
        raise ValueError(f"box mass must be 0.2-{cfg.PAYLOAD_RATED_MAX_KG:g} kg, got {kg:g}")
    if not 0.05 <= speed <= 1.0:
        raise ValueError("speed must be 0.05-1.0 of the speed bar")
    task = K.cell_cycle(style, n_boxes=n_boxes, speed=speed)
    peak = peak_speed_fraction(task)
    if peak > cfg.SPEED_BAND + 1e-6:          # the live demo stays in the trusted regime (D1)
        raise ValueError(f"{style} cycle peaks at {peak:.0%} of the velocity limit; "
                         f"the band is {cfg.SPEED_BAND:.0%}")
    # The fault starts shortly after the FIRST box is on the tool, so the arm is loaded and moving
    # when it hits — a fault injected at rest shows up only as a step and is the easy case.
    t_first_grip = next((ev[0] for ev in task.events if ev[1] == "picked"), 0.0)
    f = (FaultSpec() if fault == "none"
         else FaultSpec(fault, 0 if fault == "delay" else int(joint), LIVE_FAULTS[fault],
                        t_first_grip + 0.4))
    return Scenario(name=name, traj=task.traj, traj_type="cycle", duration_s=task.duration + 0.5,
                    payload_physical=True, box_kg=kg, task=task, fault=f)


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
          "pickphys": ("out/pickphys", pickphys), "pickfar": ("out/pickfar", pickfar),
          "payload": ("out/payload", payload), "detect": ("out/detect", detect), "detectfast": ("out/detectfast", detectfast), "detectlimits": ("out/detectlimits", detectlimits), "speedband": ("out/speedband", speedband), "pickdetect": ("out/pickdetect", pickdetect)}
ORDER = ["reference", "o3", "o4", "o5", "clean", "fast", "extra", "push", "o6", "pick", "o6pick", "pickphys", "pickfar", "payload", "detect", "detectfast", "detectlimits", "speedband", "pickdetect"]
# Suites whose payload is a REAL gripped box rather than a mass folded into the tool link, so they
# need the conveyor cell and its suction tool built into the stage. They cannot share a process
# with the others: the cell adds collision geometry the bare-arm suites must not see.
NEEDS_CELL = {"pickphys", "pickfar", "pickdetect"}
