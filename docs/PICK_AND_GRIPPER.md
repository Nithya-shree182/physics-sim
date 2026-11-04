# Picking: IK, the gripper, and what is real

Written for: anyone who watches the simulator run and asks "is that really picking something up?"

## What was wrong before

The first "pick" was not one. The arm moved along a joint-space sine around the home pose. There was no inverse kinematics, the fingers never
moved, the payload was an invisible mass added to the hand, and the cube in the window was a drawing that followed the hand. It was a calibration
motion with a story attached.

## What exists now

A task is Cartesian (where the fingertips go); the controller needs joint trajectories. The chain is:

    task (above the cube, descend, grasp, lift, carry, lower, release, retreat)
      -> IK (workspace/pal_ik.py)            joint waypoints, one elbow branch, gripper kept vertical
      -> a trajectory STYLE between them     trapezoidal, S-curve or torque-aware (workspace/pal_trajectories.py)
      -> the same computed-torque controller tracks it, whatever the style or the payload

So one IK serves every style and every payload: the style and the speed limits belong to the joint-space legs. An elliptical path is IK solved
at every millisecond along a Cartesian curve (`pal_tasks.ellipse`). 14 unit tests check the geometry without the simulator (9 more pin the suite definitions): targets reached
to 1e-7 m, gripper vertical, joint limits respected, unreachable targets reported, no branch flip, start and end at rest.

## Two kinds of pick

| | modelled (`pick`, `o6pick`) | physical (`pickphys`) |
|---|---|---|
| payload | a point mass written into the hand's inertials at the grasp, removed at the release | a real rigid cube on a real table, held by the fingers |
| cube | drawn, follows the hand (visual only) | a physical body: mass, friction, collision |
| gripper | not driven | both fingers driven: open for the approach, closed at the grasp, open at the release |
| payload is in the simulator's `M` and `g` | yes | **no**: the weight reaches the joints through contact, exactly what the residual is built to see |
| use | exact known mass, the ground truth for the estimator | realism; shows the estimator on a load it was not handed |

## The gripper recipe (from the Physyk project, checked here)

| item | value | source |
|---|---|---|
| gripper variant | `AlternateFinger` on `/World/Franka` before the articulation exists | Physyk, NVIDIA's `franka_pick_up` demo |
| finger drive | kp 2400 N/m, kd 60, effort limit 70 N for the pair, **measured**: 33 N squeeze per finger | this project (`pal_gripper_probe.py`, `pal_plant.configure_gripper`) |
| friction | cube and finger pads 1.05 static / 0.95 dynamic; table 0.70 / 0.60 | Physyk |
| cube | 0.05 m edge, contact offset 4 mm, rest offset 1 mm | Physyk |
| solver | TGS with CCD (already ours) | Physyk |
| finger pads | span 0.058-0.112 m along the hand's z axis | measured |
| grasp point | cube centre 0.09 m from the hand origin, tips 6.6 mm above the table | measured; the Franka's 0.1034 m TCP would drive the pads into the table |

Two things measured on this asset that the recipe did not say: even with the right variant the second finger has **no drive** (kp = kd = effort = 0)
and the first has only 7.2 N, so both are configured explicitly; and the effort limit is **shared by the two mimicked fingers** (a limit of 35 gave
17.5 N each), so 70 is what gives the datasheet's 35 N each.

## Results: 15 physical picks (`out/pickphys`)

Every run lifted the cube (peak height 0.21-0.24 m) and carried it across the table. Where it ended:

| run | cube | placed within |
|---|---|---|
| trapezoidal / S-curve / torque-aware | 0.5 kg | 3.7-4.0 mm |
| trapezoidal / S-curve / torque-aware | 1 kg | 6.2-6.4 mm |
| trapezoidal / S-curve / torque-aware | 2 kg | 10.2 mm |
| faults from just after the grasp, 1 kg: friction x8 joint 4 | | 6.5 mm (joint 4 barely moves on this path) |
| friction x8 joint 6 | | 16.0 mm |
| locked joint 4 | | 6.5 mm (same reason) |
| **locked joint 1 (the carry sweep)** | | **151 mm: the pick fails** |
| **50 ms command delay** | | **168 mm: the loop oscillates and the pick fails** |

So the faults change what the arm does, not only a residual: "how it copes and how it moves" is in the data.

## Windowed runs, and a bug they exposed

`tools/pal_suite.sh <suite> --window` draws the scene (ground, lights, a collidable table, the cube) in an Isaac Sim window.
**The first windowed runs were time-warped**: with the default 1/30 s rendering interval each rendered step advanced the simulation by 33 physics
steps, so the arm moved 33 times as far on every rendered row while the logger counted one step. The dynamics checks broke while moving and
passed at rest, which is how it was found. In a window the rendering interval now equals the physics step; the per-row motion ratio went from 33.3 to
1.00. A windowed run and the headless run of the same scenario agree to about 0.04 N*m in torque and a few N in finger contact force, not bit for
bit (`tools/compare_runs.py`).

## Not real, or not finished

- The grasp is **open-loop and timed**: the fingers close at a scheduled moment. There is no force or slip feedback and no regrasp.
- No gripper controller in the loop: the arm's computed torque does not feed forward the cube's weight (it is not in the articulation's terms), so
  the arm sags a few millimetres under 2 kg and feedback absorbs it.
- The cube's centre sits 0.09 m from the hand; the analysis side's payload model assumes 0.1034 m. At rest this does not matter; in motion it is a
  small error.
- One cube, one pick and one place location, one table.
- The estimator's accuracy on the real cube at 2 kg is poor (`DETECTION_RESULTS.md`); the cause is not established.

## Run it

```bash
tools/pal_suite.sh pickphys --window     # watch the real pick; 15 runs
tools/pal_suite.sh pick,o6pick           # the modelled picks, headless
PAL_FRAMES=pickphys_s_curve_2kg tools/pal_suite.sh pickphys --window    # also saves viewport frames to out/frames
```
