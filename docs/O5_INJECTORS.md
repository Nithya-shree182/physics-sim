# O5 fault injectors: what exists, how to run it, what it showed

Written for: whoever generates or reads the O5 datasets (both sides). Rebuilt 2026-11-03 after the
review (`docs/REVIEW.md`): the first version used the wrong friction units and the wrong signal.

Scope is faults, not attackers (`docs/PROPOSAL_AMENDMENTS.md`). The plant changes; the analysis side's
Pinocchio model never does. ("Plant" = the simulated arm being monitored; the model is the separate
prediction it is compared with.)

## What is built

| file | role |
|---|---|
| `workspace/pal_plant.py` | the plant: URDF inertials, 5% arm-mass mismatch, friction in N*m, payload, push |
| `workspace/pal_runner.py` | effort-mode controller and scenario runner; logs `tau_applied` |
| `workspace/pal_faults.py` | `FaultSpec` and `FaultInjector` (friction, stuck joint, delay, backlash) |
| `workspace/pal_sensors.py` | seeded sensor noise on the logged values |
| `workspace/pal_suites.py` | the suites `o5` (26 runs), `clean`, `fast`, `extra` (the analysis side's requested set), `pick`, `pickphys` |

| `fault_type` | `fault_param` | how it acts |
|---|---|---|
| `friction` | multiplier on the joint's static, dynamic and viscous friction | torque-valued friction API at onset |
| `stuck_joint` | static and dynamic friction in N*m (200, above the drive) | same call |
| `delay` | seconds | torque commands pass through a FIFO that long |
| `backlash` | total play in rad | a motor-side gap model: no torque reaches the link while the gap is open (approximation, no impact dynamics) |

`fault_joint` is a nominal 0 for `delay` because the schema needs a joint for every fault. Each run is
6 s with the fault on at 3 s, at 20% of the velocity limit, so every run carries its own healthy stretch.

## Why effort mode

`tau_measured` is the net joint torque and obeys Newton's law whatever produced it, so it cannot show
friction, backlash or a locked joint. The torque we apply can. The runner therefore drives the arm with
explicit torques (computed torque from the plant's own M, C, g) and logs `tau_applied`. The residual for
faults is `tau_applied - tau_model`. `tau_measured` stays the right signal for external loads (payload,
push).

## Run it

```bash
tools/pal_suite.sh o5                  # ~12 min: sim, Parquet, then checks on every run
```

## The set (what "defined fault set" means)

Six healthy runs (seeds 1000-1005; `healthy_s0` shares seed 1000 with every faulty run, so it is the exact fault-free twin of each);
friction x2 / x4 / x8 on joints 2, 3, 4, 6; a locked joint 4; backlash 0.02 / 0.06 / 0.12 rad on joint 1 and 0.12 rad on joint 7;
delay 10 / 30 / 50 ms. 26 runs. Runs are named
`o5_<fault tag>_1000hz`, healthy ones `o5_healthy_s0..s5`.

## What the plant did

Mean `|tau_applied - net|` and RMS tracking error on the faulted joint, before and after onset:

| scenario | applied-torque residual, N*m | tracking RMS, rad |
|---|---|---|
| healthy | 0.39 -> 0.40 (joint 4) | 0.0039 -> 0.0016 |
| friction x2 / x4 / x8, joint 4 | 0.39 -> 0.78 / 1.50 / 2.85 | 0.0039 -> 0.0032 / 0.0069 / 0.0142 |
| friction x8, joint 2 / joint 3 / joint 6 | 0.19 -> 1.47 / 0.19 -> 1.42 / 0.15 -> 0.92 | 0.0014 -> 0.0032 / 0.0004 -> 0.0060 / 0.0135 -> 0.0562 |
| locked joint 4 | 0.39 -> 9.31 | 0.0039 -> 0.0589 |
| backlash 0.02-0.12 rad | unchanged | unchanged |
| delay 10 / 30 ms | unchanged | unchanged |
| delay 50 ms | 0.15 -> 2.59 | 0.0135 -> 0.1220 (the loop oscillates) |

Friction and a locked joint are clearly visible. **Backlash and short delays are not visible in either
channel at this speed**: the applied torque stays consistent with the dynamics, and the controller rides
through a few percent of lost torque. They are in the set deliberately, because a detector's rate has to
be reported including the faults it cannot find.

## Detectability preview

`docs/PREVIEW_DETECTION.md`: with an independent numpy model and a copy of the analysis side's rule, friction x8, the locked
joint and the 50 ms delay are caught; friction x4 and below, short delays and backlash are not. Changing the torque signal
from `tau_measured` to `tau_applied` is the difference between missing and catching the friction faults.

## Not built

- backlash with impact dynamics, and faster-motion fault sets that might expose it
- attacks (out of scope by decision)

## Checks on every run

`tools/check_run.py`: schema; URDF-numpy gravity; identity on healthy rows; and a friction-scale guard
(`tau_applied - net` on healthy rows must be at the configured friction, which is the check that would have
caught the units bug).
