"""The suite definitions (workspace/pal_suites.py): what gets simulated, pinned so an edit cannot change it silently. No simulator."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "workspace"))
import pal_suites as S  # noqa: E402

BUILT = {name: mk() for name, (_, mk) in S.SUITES.items()}          # built once: the ellipse IK is the slow part
COUNTS = {"reference": 2, "o3": 12, "o4": 19, "o5": 26, "clean": 9, "fast": 10, "extra": 24, "push": 24, "o6": 26, "pick": 14,
          "o6pick": 18, "pickphys": 15, "pickfar": 9}


class Suites(unittest.TestCase):
    def test_counts(self):
        for name, n in COUNTS.items():
            self.assertEqual(len(BUILT[name]), n, name)
        self.assertEqual(set(COUNTS), set(S.SUITES))
        self.assertEqual(S.ORDER, [n for n in S.ORDER if n in S.SUITES])
        self.assertEqual(set(S.ORDER), set(S.SUITES))

    def test_names_and_output_directories_are_unique(self):
        dirs = [d for d, _ in S.SUITES.values()]
        self.assertEqual(len(dirs), len(set(dirs)))
        names = [sc.name for runs in BUILT.values() for sc in runs]
        self.assertEqual(len(names), len(set(names)))

    def test_o5_healthy_s0_is_the_exact_twin_of_every_faulty_run(self):
        runs = BUILT['o5']
        twin = next(r for r in runs if r.name == "o5_healthy_s0")
        for r in runs:
            if r.fault.type != "none":
                self.assertEqual(r.noise_seed, twin.noise_seed, r.name)
        self.assertEqual(len({r.noise_seed for r in runs if r.name.startswith("o5_healthy")}), 6)

    def test_no_attack_scenarios_exist(self):
        for runs in BUILT.values():
            for sc in runs:
                self.assertIn(sc.fault.type, ("none", "friction", "backlash", "stuck_joint", "delay"))

    def test_pick_faults_start_after_the_grasp_and_share_the_twins_seed(self):
        for suite, twin_name in (("pick", "pick_s_curve_1kg"), ("pickphys", "pickphys_s_curve_1kg")):
            runs = {r.name: r for r in BUILT[suite]}
            twin = runs[twin_name]
            faulty = [r for r in runs.values() if r.fault.type != "none"]
            self.assertTrue(faulty)
            for r in faulty:
                self.assertGreater(r.fault.onset_s, r.task.t_attach, r.name)
                self.assertEqual(r.noise_seed, twin.noise_seed, r.name)

    def test_physical_picks_use_a_real_cube_and_only_relabel_the_payload(self):
        for sc in BUILT['pickphys']:
            self.assertTrue(sc.payload_physical and sc.task.physical)
            self.assertEqual(sc.payload_kg, 0.0)                       # the plant's inertials are not rewritten
            self.assertEqual(sc.payload_events[0], (sc.task.t_attach, sc.cube_kg))
            self.assertEqual(sc.payload_events[1], (sc.task.t_open, 0.0))
            self.assertLess(sc.task.t_close, sc.task.t_attach)
            self.assertLess(sc.task.t_attach, sc.task.t_open)

    def test_only_the_physical_suite_needs_the_gripper_plant(self):
        self.assertEqual(S.NEEDS_GRIPPER, {"pickphys", "pickfar"})

    def test_modelled_picks_do_not_use_a_cube(self):
        for sc in BUILT['pick'] + BUILT['o6pick']:
            self.assertFalse(sc.payload_physical)
            self.assertIsNone(sc.cube_kg)

    def test_speeds_are_declared_through_the_trajectories_not_the_runner(self):
        # detection data stays inside the trusted regime: the pick suites use the 35% limits
        import numpy as np
        import pal_config as cfg
        for sc in BUILT['pickphys'][:3]:
            qd = np.array([sc.traj(t)[1] for t in np.linspace(0, sc.traj.duration, 2000)])
            self.assertLess((np.abs(qd).max(axis=0) / cfg.QD_MAX).max(), 0.42, sc.name)


if __name__ == "__main__":
    unittest.main()


def test_live_pick_builds_and_validates():
    import pal_suites as S
    sc = S.live_pick("s_curve", 1.5, "none")
    assert sc.cube_kg == 1.5 and sc.payload_physical and sc.fault.type == "none"
    f = S.live_pick("torque_aware", 1.0, "stuck_joint", joint=0).fault
    assert f.type == "stuck_joint" and f.joint == 0 and f.onset_s > 0
    assert S.live_pick("s_curve", 1.0, "delay").fault.joint == 0          # a delay carries the nominal joint 0
    for bad in (dict(style="zigzag"), dict(kg=9.0)):
        try:
            S.live_pick(**bad)
        except ValueError:
            continue
        raise AssertionError(bad)


def test_live_picks_stay_inside_the_trusted_speed_band():
    for style in ("trapezoidal", "s_curve", "torque_aware"):
        assert S.peak_speed_fraction(S.live_pick(style).task) <= S.LIVE_SPEED_CAP


def test_live_pick_speed_follows_the_speed_bar_and_never_passes_the_cap():
    for style in ("trapezoidal", "s_curve", "torque_aware"):
        peaks = [S.peak_speed_fraction(S.live_pick(style, speed=sp).task) for sp in (0.05, 0.3, 0.6, 1.0)]
        assert peaks == sorted(peaks) and peaks[0] < peaks[-1] / 4, (style, peaks)
        assert max(peaks) <= S.LIVE_SPEED_CAP
    for bad in (0.0, 1.5):
        try:
            S.live_pick("s_curve", speed=bad)
        except ValueError:
            continue
        raise AssertionError("an out-of-range speed was accepted")


def test_random_cube_positions_are_reachable_a_little_away_and_inside_the_speed_cap():
    import numpy as np
    seen = set()
    for seed in range(40):
        xy = S.random_pick_xy(np.random.default_rng(seed))
        seen.add(xy)
        reach = float(np.hypot(*xy))
        assert 0.52 < reach <= 0.66 and xy[0] >= 0.35 and xy[1] <= 0.01, (seed, xy, reach)
    assert len(seen) > 30                                    # genuinely different spots
    for seed in range(0, 40, 8):
        xy = S.random_pick_xy(np.random.default_rng(seed))
        for style in ("trapezoidal", "s_curve", "torque_aware"):
            sc = S.live_pick(style, 1.0, "none", pick_xy=xy)
            assert tuple(sc.task.pick_xy) == xy and S.peak_speed_fraction(sc.task) <= S.LIVE_SPEED_CAP


def test_random_cube_picks_never_close_the_detector_mask():
    import numpy as np
    root = Path(__file__).resolve().parents[1]
    sys.path[:0] = [str(root / "observers"), str(root / "model")]
    from pose_mask import FORCE_GAIN_MAX, force_noise_gain
    from pinocchio_model import load_panda
    model = load_panda()
    worst_f = 0.0
    for seed in range(60):
        task = S.live_pick("s_curve", pick_xy=S.random_pick_xy(np.random.default_rng(seed))).task
        for t in np.arange(0.0, task.duration, 0.1):                   # the whole pick, not just the grasp pose
            worst_f = max(worst_f, force_noise_gain(model.jacobian(task.traj(t)[0])))
    assert worst_f < FORCE_GAIN_MAX, worst_f
