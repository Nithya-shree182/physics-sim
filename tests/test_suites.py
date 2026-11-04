"""The suite definitions (workspace/pal_suites.py): what gets simulated, pinned so an edit cannot change it silently. No simulator."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "workspace"))
import pal_suites as S  # noqa: E402

BUILT = {name: mk() for name, (_, mk) in S.SUITES.items()}          # built once: the ellipse IK is the slow part
COUNTS = {"reference": 2, "o3": 12, "o4": 19, "o5": 26, "clean": 9, "fast": 10, "extra": 24, "push": 24, "o6": 26, "pick": 14,
          "o6pick": 18, "pickphys": 15}


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
        self.assertEqual(S.NEEDS_GRIPPER, {"pickphys"})

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
