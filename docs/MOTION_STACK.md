# Motion generation: what to use for the pick-and-place, and why it is an attestation decision

Written for: both sides, before the pick-and-place is built. Nothing here is implemented
yet — this is a design read against what has been measured.

---

## The options

| | what it gives | cost |
|---|---|---|
| **Lula IK** (`LulaKinematicsSolver`) | Cartesian pose → joint angles, analytic, deterministic, stateless | no trajectory and no collision awareness; you supply the motion between waypoints |
| **RMPFlow** | reactive, collision-aware joint targets every step | the commanded trajectory is not available in closed form |
| **PickPlaceController** | RMPFlow + gripper, 10-phase state machine | open-loop timer; phase speed set by `events_dt` |
| **cuMotion** | GPU motion planner (`isaacsim.robot_motion.cumotion` is in the image) | heaviest; no need identified |

The Physyk AI project used Lula + `PickPlaceController`, so there is a working reference in
the sibling repo including its tuning history and its failure modes.

## Why this is not a free choice

### 1. The motion generator sets the speed, and speed decides whether attestation works

Measured (`workspace/pal_fast_random.py`, `out/residual_floor.json`), worst joint as a
fraction of its torque limit at 1000 Hz:

```
10% of velocity limit -> 0.39%      75% -> 8.80%
25%                   -> 0.96%     100% -> 27.79%
50%                   -> 1.81%
```

Below 50% every joint stays under 1.81%. Above it joints 5/6/7 collapse — they have 12 N·m
limits rather than 87 and much lower inertia, so the same absolute discretization error is
a far larger fraction *and* the lighter links make a stiffer ODE.

`PickPlaceController` moves at whatever `events_dt` dictates. Physyk tuned it to 1.25× and
recorded that 2.25× "made the gripper struggle" — but nobody tuned it against a residual
floor, because nobody knew one existed. **If it is adopted, `events_dt` becomes an
attestation parameter**, and the transit phases have to be checked against the envelope.

### 2. It decides what the telemetry schema can carry

The schema wants `q_cmd`, `qd_cmd`, `qdd_cmd`. An analytic trajectory supplies all three
exactly, which is what `pal_capture.py` writes today.

**RMPFlow supplies `q_cmd` and nothing else.** Getting `qd_cmd`/`qdd_cmd` means
differentiating the command stream, which reintroduces the finite-difference noise we
spent a session characterising — on the command side this time. And `q_cmd − q` is the
tracking-error signal that catches a faithfully-executed tampered waypoint, so it is not
optional.

### 3. It defines the attack taxonomy

```
Cartesian goal ──► Lula IK ──► trajectory ──► joint setpoint ──► PD drive ──► arm
      ▲                            ▲                  ▲
   attack A                    attack B           attack C
```

| | what happens | does the residual see it? |
|---|---|---|
| **A** tampered Cartesian goal | IK produces a perfectly consistent joint trajectory | **No.** Dynamics are self-consistent. Only tracking error or an intent check sees it. |
| **B** tampered trajectory | may violate velocity/acceleration limits | Partly — if the limits are breached. |
| **C** spoofed joint setpoint | dynamically inconsistent with the arm's state | **Yes.** |

So the motion stack determines which attack classes are detectable at all. Section 9 must
state which were injected, and a detection table that only covers C should say so.

## Recommendation

**Lula IK plus our own trajectory generators.** Not RMPFlow, unless collision avoidance
becomes a hard requirement.

- O6 needs trapezoidal / S-curve / torque-aware generators regardless. Writing them yields
  analytic `q_cmd`, `qd_cmd`, `qdd_cmd` for free and lets velocity be capped at the 50%
  envelope by construction.
- Lula is deterministic and stateless, so a replay reproduces exactly — which is the point
  of having enabled `enable_enhanced_determinism` on the PhysX scene.
- RMPFlow's reactivity is a liability here: it changes the command in response to the
  scene, so a residual anomaly and a planner reaction are hard to tell apart in a log.

**Use `PickPlaceController` only for the O7 demo run**, where a visually convincing pick
matters more than clean telemetry. Log it with `traj_type: pick` so the analysis side can
treat it separately.

Two gotchas from Physyk's notes if it is used: `PickPlaceController.reset()` clears
registered RMPFlow obstacles, and the phase machine advances on a time budget rather than
on a grip-tight signal, so it can begin lifting before the jaws have converged.

## Decide early

If a tampered **Cartesian goal** (attack A) is in scope for O5, the residual cannot catch
it by construction. That needs a geometric/workspace check on the goal, or a separate
intent layer. Better scoped now than discovered when the detection table has a column of
zeros.
