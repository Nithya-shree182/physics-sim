# The UR10e port (2026-11-08)

Written for: whoever picks this up next. This page is the *reference* for the arm — where its
numbers come from, what is a citation and what is a choice, and how the cell is laid out. For what
is still to be done, read **[TODO.md](TODO.md)** first; for the telemetry contract,
**[TELEMETRY.md](TELEMETRY.md)**; for the history, [../changes.md](../changes.md).

Read all of these before `HANDOFF.md`, which still describes the project as it was with the Franka
Panda. Where it disagrees with this page, this page wins.

## What changed, in one paragraph

The arm was a Franka Panda. It is now a **Universal Robots UR10e**, and the Panda has been removed
rather than kept as an option. The reasons were reach (1300 mm against 855 mm, which is what makes
a real conveyor cell fit), six joints instead of seven (so IK is square and has no null space to
manage), and industrial recognisability. At the same time the project stopped treating sim-to-real
fidelity as a goal: the twin is given **ideal joint torque sensing**, with noise as a knob that is
off by default, because real UR hardware has no joint torque sensors at all — its reported torque is
derived from motor current, which would have blocked a torque-residual method outright. The sensing
model is a study parameter, not a fidelity claim. See `workspace/pal_sensors.py` and
[TELEMETRY.md](TELEMETRY.md).

## Where the arm's numbers live

`workspace/pal_robot.py`. One `RobotSpec` holds everything that changes when the arm changes:
URDF, joint names and order, home pose, limits, armature, friction (both the plant's
static/dynamic/viscous and the model's sigmoid), the sensor model, the tool frame and offset, the
Isaac USD path. Before the port these were spread over `pal_config.py`,
`model/pinocchio_model.py` and `model/poses.py`; that is why the port touched sixty-odd files, and
why it should not again.

Everything else reads through it: `cfg.HOME_JOINTS`, `cfg.TAU_MAX`, `engine.DOF` and so on still
work, they just resolve to the active arm.

### Provenance, and what is a guess

| Quantity | Source |
|---|---|
| Link masses, centres of mass, **full 3x3 inertia tensors**, link offsets, joint/velocity/effort limits | Universal Robots' own `Universal_Robots_ROS2_Description` (branch `rolling`, `config/ur10e/`). `tools/make_ur10e_urdf.py` generates the URDF from them, so every number is traceable. |
| Reflected rotor inertia (`armature`) | **[CHOICE]** — UR publishes none. Sized as `J_motor * N^2` for a ~100:1 harmonic drive. |
| Friction, all forms | **[CHOICE]** — no identified set exists for a UR10e. Coulomb at 1% of each joint's effort limit; the model-side sigmoid's steepness is one value for all joints, inside the range Gaz et al. measured on the Panda. |
| Sensor noise | **[CHOICE]** — zero by default (ideal sensing). The reference sigma for a sweep, `pal_sensors.REFERENCE_SIGMA`, holds the noise-to-limit ratio this project used on the Panda (0.02 N*m of 87) at this arm's larger torques, so results stay comparable: 0.076 N*m. |
| Drive gains | **[CHOICE]** — a multiple of each joint's effort limit (`cfg.KP_PER_TAU = 4.6`), taken from the Panda gain set this project used (400 N*m/rad against an 87 N*m joint). A fixed 400 would be a stiff drive on an 87 N*m joint and a soft one on a 330 N*m one. |

The friction numbers set the scale a friction fault is measured against, so they decide what the
detector can see. Treat them as a knob, and identify them from healthy data if it ever matters.

### Two cross-checks that the model is right

Both are in `tests/test_ur10e.py` and both pass:

- **Mass matrix**: the hand-written numpy composite-rigid-body model agrees with Pinocchio on the
  same URDF to **1e-15 kg·m²** over random poses. This matters more on the UR10e than it did on
  the Panda, because three of its links carry genuinely off-diagonal tensors expressed in *rotated*
  frames — ignoring that rotation would be the same class of error as the diagonal-only inertia bug
  of 2026-11-03 (REVIEW F11). Both `pal_kinematics.py` and `pal_plant.py` now apply it.
- **Total mass**: 33.10 kg against a 33.5 kg datasheet figure (Pinocchio reports 29.10 kg of moving
  mass; the 4.0 kg fixed base is attached to the world).

The arm-agnostic physics laws also pass on the new model: energy conservation in free fall,
skew-symmetry of `Mdot - 2C`, mass-matrix positive-definiteness, Jacobian against finite
differences, forward/inverse dynamics agreement.

## The cell

`./start_sim_docker.sh` runs `workspace/pal_cell_demo.py`: a UR10e on a pedestal picks three
cardboard boxes off an infeed table with a **four-cup suction tool** and sets them on a running
conveyor, inside a guarded cell on a factory floor.

The cycle itself is `pal_tasks.cell_cycle()`, and the layout it reads is `pal_cell_geom.py` — plain
numbers with no USD import. One definition, shared by the watched demo, the live server
(`pal_live.py`, which drives the dashboard) and the recorded suites, so a cycle time quoted from one
is a cycle time about the others. It also means the whole cycle builds on the host with no Isaac,
which is what lets `tests/test_suites.py` check it at all.

- **Suction, not fingers** (`pal_suction.py`). A grip is a USD fixed joint between the wrist and
  the box. One joint per box is created **before** `world.reset()` and left disabled; gripping
  writes the joint's frames from the measured relative pose and enables it. Creating or deleting a
  joint while the simulation runs changes stage topology, which tears down the articulation's
  physics view and kills the run on the next torque read — that is how this was found.
- **Why suction helps the measurement**: a finger grasp carries part of the box's weight through
  finger-contact friction, a load path whose stiffness depends on the solver timestep. That is the
  leading suspect for the Panda's unexplained 2 kg payload error at 1000 Hz (STATUS 11/18). A
  suction grip is modelled as a rigid attachment, so the whole load reaches the wrist.
- **Seal test**: lateral offset ≤55 mm and vertical gap ≤30 mm, judged separately. A single
  Euclidean distance made the tolerance depend on how the error split between sideways and down;
  the arm is still converging when the vacuum latches, and lateral error at that moment runs to
  about 32 mm.
- **Carry height**: box top + a full box height + `LIFT_CLEARANCE`. An approach height of one box
  height left the carried box's underside exactly level with its neighbours' lids.
- **Release**: `DROP_GAP` = 3.5 mm above the belt, so the belt catches the box rather than the cups
  scuffing it.
- **Belt**: flat surface whose top and collision plane are the same, with running seam lines. It
  used to be rollers, drawn 32 mm proud of the plane boxes rest on, so every box sat half sunk.
- **Camera**: named views in `pal_cell_geom.CAM_VIEWS`, chosen with `PAL_CAM`. The default `iso`
  is a three-quarter view looking down 30 degrees; it replaced an almost level 16-degree shot that
  flattened the infeed table and the belt into one another. Each view is a distance and two angles
  rather than three coordinates, and `tests/test_cell_geometry.py` checks every one against the
  real 22 mm / 1280x720 frustum — horizontally and vertically separately, because a 16:9 frame is
  much narrower vertically (15.0 degree half-angle against 25.5) and an isometric view is exactly
  the one that pushes things off the top. Measured: the arm spans 42% of the frame's height from
  `iso` against 27% from the old `side`, so raising the camera cost no apparent size.

### Performance

About **0.8x real time** at 250 Hz physics and 30 rendered fps on an RTX 5090 laptop. Lessons, all
measured:

- The old live script drew one frame every 25 physics steps — a 10 fps window on a 250 Hz
  simulation. That, not the physics, was the "laggy and stuck" feeling. `PAL_RENDER_HZ` is now a
  separate knob; `PAL_RENDER_HZ=20` buys real time if you prefer it to frame rate.
- `rendering_dt` stays pinned to the physics step, so one rendered call is still exactly one
  physics step and the time-warp bug of 2026-11-04 (REVIEW F13) cannot return.
- USD attribute writes are expensive. Animating the scene every physics step held the cell to half
  of real time; looking prims up through USD each frame cost another 15 s of a 24 s cycle. Visual
  updates now happen on rendered frames only, and every animated prim's xform ops are captured
  when it is built.

### Known flakiness

Isaac drops the articulation's physics view at random — twice in nine runs — the same class of
problem `tools/run_sim.sh` already worked around at boot. There is now mid-cycle recovery in the
demo and a retry in the launcher. It is a workaround, not a fix.

## Telemetry

Schema **v1**, `schema/telemetry_v1.yaml`, 205 columns at six joints. Beyond the joint-count change
it adds a logged `qdd`, the full per-joint torque decomposition, energy and power, headroom and
pose-conditioning channels, and replaces the finger columns with the suction tool's grip state.
`schema/validate.py` reads the joint count from the robot spec and checks the identities between
the new columns. **[TELEMETRY.md](TELEMETRY.md) is the page to read**; it covers what each group is
for, the sensor model, decimation, the timing conventions and how to add a channel.

## What is NOT done

Moved to **[TODO.md](TODO.md)**, so there is one list rather than two that drift apart. The short
version: the project still has no validated detection or payload result for the arm it runs,
because every recorded run in `out/` and `data/ref_matched_*` is Panda data and regenerating them
needs a GPU session. That blocks the residual-floor table, the <=50% speed band's evidence (D1),
the 250 Hz choice (D3) and the trajectory-style comparison.

For the record, since this page is where the port's own state was written down: the fifteen
individually-quarantined tests this port left behind were all ported the same day, and
`pytest` now reports **161 passed, 57 skipped** where it reported 110 / 71. The 57 are the four
recorded-Panda-evidence modules plus one missing-data skip.

## Running it

```bash
./start_sim_docker.sh                       # the cell, windowed
PAL_CYCLES=1 PAL_SHOTS=3,15 ./start_sim_docker.sh    # one cycle, save viewport PNGs to out/cell/
PAL_RENDER_HZ=20 ./start_sim_docker.sh      # real time, fewer frames
PAL_BOX_KG=2.5 PAL_STYLE=trapezoidal ./start_sim_docker.sh
PAL_SENSOR_NOISE=1 ./start_sim_docker.sh    # switch the sensor noise on (off by default)
PAL_CAM=iso_close ./start_sim_docker.sh     # iso (default) | iso_high | iso_close | side | top
./pal.sh                                    # the live cell plus the dashboard
tools/pal_suite.sh                          # record the data suites (GPU; see TODO.md item 1)
.venv-pal/bin/python -m pytest -q           # 161 pass, 57 skipped with reasons
.venv-tools/bin/python schema/validate.py out/<run>.parquet
tools/make_ur10e_urdf.py                    # regenerate the URDF from UR's published config
tools/make_sign_texture.py "Some Text"      # regenerate a sign texture
```

A trap worth keeping: sign and logo textures must have **transparent black** backgrounds, not
transparent white. The shader drives emission from the texture's RGB while opacity comes from its
alpha, and emission is not masked by opacity — a transparent-white background makes the whole quad
glow as a solid slab.
