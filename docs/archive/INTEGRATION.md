# Integrating the `nithya` research with the Isaac Sim scene

Written for: both people on this project, before the schema is frozen.

The `nithya` branch carries `WORK_SPLIT.md`, `docs/SOURCES.md` and two reference PDFs. The
`dev` branch carries a working Isaac Sim scene and a physics audit. They were built
independently and they **disagree in two places that matter**. This file records what was
merged, what conflicts, and what has to be decided by a person.

---

## 1. Not a conflict: the residual definition is forced by the control mode

An earlier draft of this file claimed `τ_measured` and `τ_applied` "detect different
things". **Measured, and that was wrong** — but the measurement turned up something more
useful. Test in `workspace/pal_torque_test.py`, Isaac Sim 6.0.1.

### `τ_applied` is identically zero in position mode

| | `g(q)` | `τ_measured` | `τ_applied` |
|---|---|---|---|
| joint2, no payload | −1.6568 | −1.6572 | **0.0000** |
| joint4, no payload | 20.0787 | 20.0788 | **0.0000** |
| joint2, +3 kg | −11.2293 | −11.2317 | **0.0000** |
| joint4, +3 kg | 34.7665 | 34.7672 | **0.0000** |

Change caused by the 3 kg payload:

```
g(q)         : [0, -9.573, 0, 14.688, 0,  2.730, 0]
tau_measured : [0, -9.574, 0, 14.688, 0,  2.730, 0]    ← tracks it exactly
tau_applied  : [0,  0,     0,  0,     0,  0,     0]    ← does not move at all
```

`get_applied_joint_efforts()` does **not** return the PD drive's output. It returns what was
last written by `set_joint_efforts()`, which in position mode is nothing. It is not a weak
signal in position mode — it is not a signal.

So `r = τ_applied − τ_model` in position mode reduces to `−τ_model`. It would appear to
"detect" the 3 kg payload perfectly, because it is simply reporting the model torque back
with a sign flip. That is the most dangerous possible failure: a residual that looks
informative and contains no measurement.

### In effort mode they are the same signal, as expected

Commanding `τ = g(q)` and reading both back:

```
commanded     : [0, -2.1663, 0, 20.8698, 0, 1.0043, 0]
tau_applied   : [0, -2.1663, 0, 20.8698, 0, 1.0043, 0]   ← exactly the command
tau_measured  : [0.2439, -1.7248, -0.4341, 20.4301, -0.0094, 1.1450, 0.0001]
max |tau_measured - tau_applied| = 0.4415 N*m
```

The remaining gap is the arm drifting while settling, not a systematic difference between
the two quantities. **In effort mode the two residual definitions are equivalent**, which is
the point made against the earlier draft and it is correct.

`τ_applied` is arguably the *better* of the two there: it is exactly what was commanded, so
it carries no solver or sensor noise, while `τ_measured` does.

### What this actually means

The two definitions are not rival philosophies. They are a **control-mode dependency**:

> `WORK_SPLIT.md`'s residual requires effort-control mode. In position mode it is
> identically `−τ_model` and measures nothing.

`WORK_SPLIT.md` is already self-consistent about this — "effort-control mode" is in Person
A's ownership list. The risk is purely that someone runs the residual against a
position-mode log, sees plausible-looking numbers, and never finds out. **Add a schema
guard: record the control mode per run and have the residual code refuse a position-mode
log when the definition is `τ_applied`.**

The one real gap stands, and `WORK_SPLIT.md` had already identified it: **neither residual
catches a faithfully-executed tampered waypoint.** If the arm is told to go somewhere wrong
and goes there correctly, the dynamics are consistent and both residuals stay near zero.
That needs tracking error `q_cmd − q`, which is a third signal and not a choice between the
first two.

## 2. Conflict: the armature trap is live, not dormant

My [ARM_CONFIG.md](ARM_CONFIG.md) §4 warned that armature enters the *simulator's* mass
matrix, and said it was harmless "while the twin reads M(q) back from the articulation API".

**`WORK_SPLIT.md` makes it live.** Person B builds an **independent** model in Pinocchio
from the Panda URDF. A URDF-derived `M(q)` has **no armature in it**. The scene currently
sets 0.0061 kg·m² per joint, so B's model and A's plant will disagree by exactly that
reflected inertia — producing an acceleration-proportional residual that is
indistinguishable from an unmodelled payload.

Three ways out, in order of preference:

1. **Add armature to B's Pinocchio model** — `model.armature = [0.0061] * 7`. Pinocchio
   supports this directly. Keeps the plant physical.
2. **Set `APPLY_ARMATURE = False`** in `pal_config.py`. Costs some solver stability at
   stiff gains, but the two models then match by construction.
3. Leave it and calibrate it out — **do not do this.** It hides a known constant inside the
   signal being measured.

Whichever is chosen, the calibration run of week 2 will not close until it is. This is the
single most likely cause of "the identity doesn't close and we can't see why".

## 3. Merged in from `docs/SOURCES.md`

### Gaz et al. friction, now verified against the source PDF

The supplementary PDF was read directly rather than trusted from memory or from a text
extraction. Results:

- The viscous and Coulomb values I had been carrying were **exactly right**
  (`fv = [0.0665, 0.1987, 0.0399, 0.2257, 0.1023, −0.0132, 0.0638]`,
  `fc = [0.2450, 0.1523, 0.1827, 0.3591, 0.2669, 0.1658, 0.2109]`).
- **I was missing a third term.** Table III fits `τ_f = fv·q̇ + fc·sign(q̇) + fo`, and the
  constant offset `fo = [−0.1073, −0.1566, −0.0686, −0.2522, 0.0045, 0.0910, −0.0127]` is a
  real identified parameter. Omitting it biases the residual by up to **0.25 N·m on
  joint 4** — a standing error a payload estimator absorbs as ~25 g of phantom mass.
- `SOURCES.md` flagged its Table I transcription as "read from a garbled text extraction,
  check before use". **Checked — the model form and all 21 values are correct**, including
  the `−φ₁/(1+e^(−φ₂φ₃))` term that makes `τ_f(0) = 0` exactly.
- The **standard deviations** are now carried alongside the values, and they change how the
  numbers may be used: `fv,6 = −0.0132` has a standard deviation of **12.26**. That estimate
  is noise, and negative viscous friction is physically impossible. It is clamped to zero —
  now justified by its own error bar rather than by taste. `fo,5` (std 19.46) and `fo,7`
  (std 6.26) are likewise not meaningful.

### Which friction model to use, and the catch

Gaz gives two. The paper **fits** the sigmoid (Table I) and adopts the linear
viscous+Coulomb+offset form (Table III) only "because of their linearity in the defining
parameters", while warning it "may suffer from sudden discontinuities in the neighbourhood
of q̇ = 0".

For this project the sigmoid is clearly better: a `sign()` term chatters around zero
velocity, and the arm sits near zero velocity at every waypoint of a pick-and-place —
exactly where chatter would masquerade as a residual spike and be classified as a fault.

**The catch: PhysX cannot represent the sigmoid.** It takes one scalar per joint. So:

- the **plant** gets the scalar (what PhysX accepts),
- the **model** should use `friction_sigmoid()`, now in `pal_config.py`,
- and the difference between them is part of the deliberate twin/plant mismatch rather
  than an accident.

If the sigmoid is wanted in the plant too, it has to be injected as an applied torque each
step, proportional to the current joint velocity — which also happens to be the only way to
recover the Coulomb-vs-viscous fault distinction (see §5).

### Physics rate settled

`WORK_SPLIT.md` targets a fixed 1/1000 s step; my scene had 400 Hz as a GPU-headroom
compromise. **Changed to 1000 Hz.** The work split wins — it matches the real Franka control
loop, and the dashboard decimates for display anyway.

### Datasheet caveat adopted

`SOURCES.md` correctly warns that the FR3 datasheet is **not** the original Panda. This
scene loads `FrankaRobotics/FrankaPanda/franka.usd`, so the Panda limits in `pal_config.py`
(`τ_max = [87,87,87,87,12,12,12]`, `q̇_max = [2.175×4, 2.61×3]`) are the right ones. Anyone
switching to an FR3 asset must swap the whole limit block, not just the torque row.

## 4. What `dev` already answers for the week-1 questions

| `WORK_SPLIT.md` item | Status on `dev` |
|---|---|
| Isaac Sim version supporting the 5090 | **Answered.** Isaac Sim 6.0.1 in Docker, verified on an RTX 5090 Laptop. ~45 s full scene + audit. |
| Panda loaded, fixed step | Done, 1000 Hz, `pal_scene.py` |
| Effort-control mode | **Path found, not yet wired.** This build exposes `switch_dof_control_mode()`, `set_effort_modes()`, `set_joint_efforts()`. Better than zeroing position gains, which leaves the drive in position mode still fighting an applied force. |
| Dynamics terms from sim | **All available.** `get_mass_matrices()` (1,9,9), `get_generalized_gravity_forces()`, `get_coriolis_and_centrifugal_forces()`, `get_jacobians()` (1,10,6,9), `get_measured_joint_efforts()`, `get_applied_joint_efforts()`. Note `get_mass_matrices()`, **not** `get_generalized_mass_matrices()`. |
| Confidence score scaling | **Confirmed necessary, with numbers.** B's plan already says "scaled so position and orientation rows are unit-consistent" — correct. Measured: unscaled `c(q)` never exceeds **0.13 anywhere**, including the home pose. Without the scaling the gate suppresses every pose. |
| Deliberate model mismatch | Partly done: friction ×1.15 plant scale. Link-mass perturbation not yet implemented. |

## 5. Known limitation that affects the fault list

This Isaac Sim build has **no `PhysxJointAxisAPI`**, so PhysX's three-part friction model
(static Coulomb / dynamic Coulomb / viscous) is unreachable. `set_friction_coefficients()`
takes **one scalar per joint**.

Consequence for O5: a **stiction** fault (raises breakaway, Coulomb-like) and a **bearing
wear** fault (grows with speed, viscous-like) both arrive as a change in the same scalar and
**cannot be separated from the plant model alone**. Since `WORK_SPLIT.md` §4 lists "friction"
as a single fault class, this may be acceptable — but if the fault list is ever split into
stiction vs wear, the viscous term must be injected as an applied torque in the loop.

**DECIDED: one class, both names.** Stiction and bearing wear are kept indistinguishable.
Every mention of this fault class, anywhere in the code, the dashboard or the report, must
read `friction (stiction / bearing wear)` — never one of the two alone.

## 6. Structural divergence (not yet resolved)

`WORK_SPLIT.md` §4 specifies a layout that `dev` does not follow:

```
twin/      l0_dynamics.py, scene.py     (A)     dev has: workspace/pal_scene.py
configs/   *.yaml                       (shared) dev has: workspace/pal_config.py
schema/    telemetry_v0.yaml            (shared) dev has: nothing
model/ observers/ ui/ analysis/         (B)      dev has: nothing
```

Two real gaps beyond the renaming:

- **All tunable numbers should live in `configs/*.yaml`**, per §5. They currently live in
  `pal_config.py` as Python. The *provenance comments* in that file are the valuable part
  and must survive any move to YAML — a bare `friction_viscous: [0.0665, ...]` with no
  source tag is exactly the kind of unsourced number this project exists to avoid.
- **`schema/telemetry_v0.yaml` does not exist**, and `pal_scene.py` emits a one-off audit
  JSON rather than per-step Parquet frames. This is the week-1 handoff artifact and it
  blocks Person B entirely.

Suggested: keep `pal_config.py` as the sourced reference, generate the YAML from it, and
treat the YAML as the machine-readable view rather than the master. Needs agreement.

---

## Open items

### Must be decided by a person

1. **Residual definition** — largely settled (§1): the two are equivalent in effort mode,
   and `τ_applied` is unusable in position mode. Remaining decision is narrower: record the
   control mode in the schema and make the residual code refuse a mismatched log. Tracking
   error `q_cmd − q` is still needed as a separate third signal.
2. ~~**Armature**~~ — **DECIDED 2026-11-02.** Option 1: armature stays in the plant at the
   physical 0.0061 kg·m², and the analysis side adds `model.armature = 0.0061` on all 7
   joints in Pinocchio. Changes have been made on that side. The value is now a **contract
   between two codebases** — changing one side alone reintroduces ~27 g of phantom payload
   bias, systematic and not removable by averaging.
3. ~~**Is "friction" one fault class or two?**~~ — **DECIDED 2026-11-02.** One. Stiction and
   bearing wear stay **indistinguishable**; no velocity-proportional torque injector will be
   built. **Reporting rule:** wherever this value or a fault derived from it appears — fault
   list, O5 confusion matrix, dashboard alert panel, results tables — it must be labelled
   with **both** classes, `friction (stiction / bearing wear)`, never one alone. A single
   label would assert a distinction the plant cannot make and would make the detector look
   more specific than it is.
4. **Config format** — YAML as master, or generated from `pal_config.py`? (§6)
5. **Report ownership** — still open from `WORK_SPLIT.md` §10 (suggested there: B writes, A reviews).
6. **ROS 2 bridge on Windows, or a socket path?** — still open from §10. Note the sim here
   runs in **Linux Docker**, which may remove the problem entirely.

### Engineering, nobody is blocked

7. `schema/telemetry_v0.yaml` + per-step Parquet writer — **the week-1 handoff; B is blocked
   until this exists.**
8. Wire effort-control mode via `switch_dof_control_mode()`.
9. Rescale the Jacobian before `c(q)`, then sweep the workspace to set the threshold from a
   distribution. The current 0.08 is a guess and is demonstrably in the wrong place.
10. Link-mass perturbation for the twin/plant mismatch (friction scaling alone is done).
11. Verify `TORQUE_CONVENTION` — still `"unverified"`. Needs a **loaded, moving** test; the
    static check cannot resolve it because the unloaded joints give only noise.
12. Confirm the Jacobian link index. `get_jacobians()` returns `(1, 10, 6, 9)`; the code
    takes the last link, which has not been verified to be the TCP at `TCP_OFFSET = 0.0584`.
    A wrench resolved at the wrong point carries a spurious moment.
13. Trajectory generators respecting the datasheet `q̈` and jerk limits (already in
    `pal_config.py`, unused).

### Cosmetic

14. The table renders light grey instead of the dark `(0.16, 0.18, 0.22)` set on the
    `FixedCuboid` — the colour is not reaching the material.
15. The container logs `CPU performance profile is set to powersave`, which will cost
    throughput on the long headless dataset runs of week 5.

### Licensing

16. `SOURCES.md` already raises this: the two PDFs in `docs/references/` belong to their
    authors and to Franka. **If this repo is ever made public, they must come out** and the
    links alone kept. Worth settling now rather than at publication.
