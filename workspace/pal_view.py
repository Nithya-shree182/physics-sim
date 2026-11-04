"""What is drawn in the window beyond the arm: the cube that gets picked. Import only after SimulationApp has started.

Visual only: the cube has no collision and no mass. The payload's mass is in the arm's inertials (the plant rewrites them at the grasp
and release), and this cube just follows the fingertip centre between the two moments so the pick can be seen.
"""
import numpy as np

import pal_config as cfg


class PickScene:
    def __init__(self, urdf):
        import omni.usd
        from pxr import Gf, UsdGeom
        self.Gf, self.urdf = Gf, urdf
        stage = omni.usd.get_context().get_stage()
        cube = UsdGeom.Cube.Define(stage, "/World/PickCube")
        cube.CreateSizeAttr(cfg.PAYLOAD_SIZE)
        cube.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.12, 0.10)])
        self.op = UsdGeom.Xformable(cube).AddTranslateOp()
        self.park((0.45, -0.25))

    def park(self, xy):
        self.op.Set(self.Gf.Vec3d(xy[0], xy[1], cfg.PAYLOAD_SIZE / 2))

    def update(self, task, t, q):
        """Place the cube for time t of the scenario: at the pick point, in the hand, or at the place point."""
        if task is None or task.t_attach is None:
            return
        if t < task.t_attach:
            self.park(task.pick_xy)
        elif t < task.t_release:
            p = self.urdf.tool_jacobian(q)[1]
            self.op.Set(self.Gf.Vec3d(*p))
        else:
            self.op.Set(self.Gf.Vec3d(task.place_xy[0], task.place_xy[1], cfg.PAYLOAD_SIZE / 2 + 0.003))
