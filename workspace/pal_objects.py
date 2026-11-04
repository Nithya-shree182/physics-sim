"""The physical workcell: a table you can touch and a cube with real mass, friction and collision. Import after SimulationApp starts.

This is what makes a pick real. The arm's fingers close on the cube, friction holds it, and the cube's weight reaches the joints through
contact: the simulator's own M and g (which describe the arm's links only) do NOT contain it, which is exactly the situation the residual
is built for. Settings follow Physyk's validated recipe: TGS solver with CCD (pal_config), grasp friction 1.05/0.95, tight contact offsets
on the cube (4 mm contact, 1 mm rest), and a high-friction material on the finger pads.
"""
import numpy as np

import pal_config as cfg

CUBE_EDGE = cfg.PAYLOAD_SIZE
PARK_XY = (0.80, 0.55)               # where the cube waits when a run does not use it (on the table, far from every motion)


class Workcell:
    def __init__(self, world):
        from isaacsim.core.api.materials import PhysicsMaterial
        from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
        self.grasp = PhysicsMaterial(prim_path="/World/Physics/GraspFriction", static_friction=1.05, dynamic_friction=0.95,
                                     restitution=0.0)
        surface = PhysicsMaterial(prim_path="/World/Physics/SurfaceFriction", static_friction=0.70, dynamic_friction=0.60,
                                  restitution=0.0)
        world.scene.add(FixedCuboid(prim_path="/World/WorkTable", name="work_table", position=np.array(cfg.TABLE_POSITION),
                                    scale=np.array(cfg.TABLE_SCALE), color=np.array(cfg.TABLE_COLOR), physics_material=surface))
        world.scene.add(DynamicCuboid(prim_path="/World/PickCube", name="pick_cube",
                                      position=np.array([*PARK_XY, CUBE_EDGE / 2]), scale=np.array([CUBE_EDGE] * 3),
                                      color=np.array([0.92, 0.18, 0.18]), mass=0.5, physics_material=self.grasp))
        import omni.usd
        from pxr import PhysxSchema, UsdShade
        stage = omni.usd.get_context().get_stage()
        api = PhysxSchema.PhysxCollisionAPI.Apply(stage.GetPrimAtPath("/World/PickCube"))
        api.CreateContactOffsetAttr(0.004)
        api.CreateRestOffsetAttr(0.001)
        for finger in ("panda_leftfinger", "panda_rightfinger"):        # grasp friction on the pads too: PhysX averages the pair
            UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(f"{cfg.ROBOT_PRIM}/{finger}")).Bind(
                UsdShade.Material(stage.GetPrimAtPath("/World/Physics/GraspFriction")), UsdShade.Tokens.strongerThanDescendants,
                "physics")
        self.view = None

    def initialize(self):
        from isaacsim.core.prims import RigidPrim
        self.view = RigidPrim("/World/PickCube")
        self.view.initialize()

    def place(self, xy, kg):
        """Put the cube on the table at xy with mass kg, at rest. (Tensor writes: call after the final world.reset().)"""
        self.view.set_masses(np.array([float(kg)], np.float32))
        self.view.set_world_poses(np.array([[xy[0], xy[1], CUBE_EDGE / 2 + 0.0005]], np.float32),
                                  np.array([[1.0, 0.0, 0.0, 0.0]], np.float32))
        self.view.set_velocities(np.zeros((1, 6), np.float32))
        back = float(np.asarray(self.view.get_masses()).reshape(-1)[0])         # every write is read back (pal_plant does the same)
        assert abs(back - kg) < 1e-3, f"cube mass write did not take: asked {kg} kg, the simulator has {back} kg"
        self.mass_readback = back

    def position(self):
        return np.asarray(self.view.get_world_poses()[0]).reshape(3).copy()

    def park(self):
        self.place(PARK_XY, 0.5)
