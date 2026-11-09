# Reply to `docs/NOTES_FOR_DEV.md`

Written for: the analysis side. Answers each numbered item, says what changed on `dev`.
Status 2026-11-03.

---

## §1 Gaz values — acknowledged, nothing to do

Thanks for re-checking with `-raw`/`-table` and retracting it. We had read the same table
from the PDF on your branch and got the same numbers, so both sides independently agree
on `fv`, `fc` and `fo`. The pitfall is worth keeping in `SOURCES.md`.

## §2 The simulated arm is not the URDF's arm — **fixed**

Your inference from the exact zeros on joints 3 and 5 was right, and the cause is bigger
than the hand. Full per-body dump of the shipped `franka.usd`:

| quantity | asset | URDF |
|---|---|---|
| centre of mass, **every** body | exactly `[0, 0, 0]` | offsets up to 120 mm |
| inertia diagonal, **every** body | ~1e-6 kg·m² | e.g. `ixx = 0.703` on link1 |
| per-link mass | differs by up to 4.5× | — |
| total mass | exactly 18.0000 kg | 17.6218 kg |

The inertia figure is not a unit confusion: writing `[0.1, 0.2, 0.3]` through
`set_body_inertias` reads back `[0.1, 0.2, 0.3]` exactly, so the API is in kg·m² and the
asset genuinely has no rotational inertia on any link. Dynamically it was a stick figure
of point masses on the joint axes — which is precisely why no change of mass could put a
non-zero value on joints 3 and 5.

**Decision taken, matching your recommendation: the URDF's inertials are authored into the
plant.** `AUTHOR_INERTIAL_FROM_URDF` in `pal_config.py`, applied in `pal_scene.py` at
build time. Result:

```
your Pinocchio (URDF)  : [0, −4.584, −0.636, 22.881, 0.623, 2.497, 0]
Isaac Sim (authored)   : [0, −4.663, −0.636, 22.954, 0.623, 2.441, 0.001]
```

Agreement to rounding on all seven joints from two independent engines. The accidental
mismatch removed was 13.65% of joint 6's torque limit.

Per-body numbers for all 11 bodies are in `out/inertial_dump.csv` and `.json`
(`workspace/pal_inertia_dump.py` regenerates them).

**One caveat neither of us can settle:** this makes the twin and plant agree, not correct.
Franka publishes no dynamic parameters. We should not claim these are the real robot's.

### A related bug this exposed, worth knowing if you ever drive the simulator

`world.reset()` **silently discards everything written through the tensor API** — masses,
COMs, inertias, joint friction, armature. The writes succeed, the readbacks confirm them,
and a later reset reverts all of it. For a while our log reported URDF parameters applied
while every downstream number was the stock asset. Apply after the final reset and verify
after it; `pal_scene.py` now prints a `[survive]` line.

## §3 Schema — adopted unchanged, writer implemented

`schema/telemetry_v0.yaml` and `schema/validate.py` are copied onto `dev` as-is. No second
schema. The writer is `workspace/pal_telemetry.py`, driven by `workspace/pal_capture.py`.

Answers to your four questions:

**Label columns and allowed values — accepted as written.** `traj_type`, `drive_mode`,
`fault_type`, `attack_type` all match what we can produce. One note: `drive_mode` includes
`asset_stock`, which in our config means the asset's own 22918 N·m/rad (the per-degree
trap). We would rather never log a run in that mode; it is kept only so the value exists
if someone does.

**`qdd_cmd_*` and `push_force_*` — accepted, both written.** `qdd_cmd` comes free from the
analytic trajectory. `push_force_*` is null until the validation push is wired.

**M/J decimation — your encoding adopted.** `mj_every=N` reads `M` and `J` every Nth step
and holds the last value between; `mj_fresh` marks the rows where they were actually read.
`mj_every=1` (the default) disables it and omits the column. `mj_every` is also in the
manifest.

**Timing convention — measured, and it turns out not to matter.** We tested your question
directly by evaluating the model against pre-step, post-step and midpoint state:

```
post-step state   worst 27.98% of limit
pre-step state    worst 27.99%
midpoint state    worst 27.98%
```

Indistinguishable, so the half-step question is settled: it is not the source of our
fast-motion residual. The row is assembled **after** `world.step()`, so every quantity in
it is the post-step value and `tau_measured[k]` is the effort during the step that
produced `q[k]`. That is now written into the manifest as `timing_convention`.

### One deviation from the schema, in the delivery format only

The Isaac Sim container has **neither pandas nor pyarrow**, so the writer emits a
compressed `.npz` with numpy alone and `tools/npz_to_parquet.py` converts it on the host.
You still receive Parquet in your schema; only the conversion point moved. Flagging it in
case you expected the sim to emit Parquet directly.

## §4 Residual sign — agreed

`r = tau_measured − tau_model`. The position-mode guard stays: `tau_applied` is identically
zero in position mode, so a residual defined on it silently becomes `−tau_model`.

## §5 Confidence score — confirmed, it is yours

The analysis side takes `c(q)`: the row rescaling (0.472 m lever arm), the workspace sweep,
the threshold, and the direction-specific variant for O4. We will not duplicate it.

Our measurement for your baseline, with the corrected Jacobian row and the authored plant:
`c(q)` spans 0.0200 (elbow-singular) to 0.1245 (home) across five named poses, i.e. it
never approaches 1 anywhere, which is what makes the rescaling necessary rather than
cosmetic.

## §6 Payload placement — our 4.896 figure was wrong, **4.6306 is correct**

Cause: we attached the payload by raising `panda_hand`'s mass and wrote
`hand_mass + 3.0` assuming the hand was the URDF's 0.730 kg. The asset's hand was actually
0.558 kg, so that line added **3.172 kg**, not 3.0. 13.89 / 3.172 = 4.631. The
`INTEGRATION.md` figure has been corrected.

**Attachment point changed to match yours.** It was at the hand origin; `pal_capture.py`
now folds the payload in as a point mass at **0.1034 m along `panda_hand`'s z axis** —
combined mass, combined COM, and the payload's parallel-axis inertia contribution about
that combined COM. The attachment is recorded in the manifest under `payload_attachment`.
With that change our lever arm should match your 4.60 N·m/kg; we will confirm on the next
payload sweep.

## §7 Noted, with thanks

Your `gravity_check.py` self-tests (potential-energy gradient to 1e-8, forward/inverse
dynamics inverting to 3e-14) are a stronger validation than anything we have on the plant
side, and they are why we treated your gravity vector as the reference rather than ours.

## §8 `data/` — added to `.gitignore`

---

> **CORRECTION 2026-11-03:** the "time discretization, first order in dt" finding and its
> 200/500/1000/2000 Hz figures below are wrong (the sweep ignored `physics_dt`). The
> residual is rate-independent below ~50% of velocity limit and grows with rate above it.
> See `docs/RESIDUAL_FLOOR.md` and the correction in `docs/OPEN_PROBLEMS.md` (P1).

## From our side: one finding that affects your residual expectations

**The fast-motion residual is time discretization, not a modelling error.** Measured on a
100%-of-limit excitation trajectory, worst joint as a fraction of its torque limit:

```
200 Hz -> 50.91%     500 Hz -> 14.24%     1000 Hz -> 2.78%     2000 Hz -> 1.44%
```

Fitting RMS residual against `dt` gives an exponent of **1.049** — first order in `dt`.
Two controls confirm it: solver iteration count changes nothing (14.24% at 4 iterations
and at 128), and armature changes nothing (2.3692 vs 2.3892, so armature *is* correctly
inside the reported `M`). Residual also rises with drive stiffness (kp 80 → 12.17%,
kp 5000 → 33.87%), which is the same effect from the other side: a stiffer drive is a
stiffer ODE and discretizes worse at a given `dt`.

**What this means for you:** there is a residual floor set by the physics rate and the
drive stiffness, and it is not something your model can remove. At 1000 Hz with
`position_stiff` it is roughly 2.8% of a joint's torque limit on fast motion, and well
under 1% on slow motion. Detection thresholds for O5 should sit above it, and it should
be reported alongside any detection rate.

We are keeping 1000 Hz for this reason, even though a full telemetry frame costs ~1.29 ms
and therefore cannot stream in real time at that rate. Decimating `M`/`J` is the trade we
are making instead — hence adopting your `mj_fresh` encoding.


---

# The operating envelope — this sets your detection thresholds

Mapped on 2026-11-03 (`workspace/pal_fast_random.py`, `out/residual_floor.json`). The
discretization floor is **not one number**: it has a knee at ~50% of velocity limit, and
the knee is wrist-specific.

Worst-joint RMS residual as a fraction of that joint's torque limit, 1000 Hz:

| speed | j1 | j2 | j3 | j4 | j5 | j6 | j7 | worst |
|---|---|---|---|---|---|---|---|---|
| 10% | 0.09 | 0.31 | 0.39 | 0.20 | 0.23 | 0.24 | 0.01 | **0.39%** |
| 25% | 0.24 | 0.78 | 0.96 | 0.53 | 0.54 | 0.60 | 0.03 | **0.96%** |
| 50% | 0.54 | 1.57 | 1.81 | 1.14 | 1.03 | 1.27 | 0.07 | **1.81%** |
| 75% | 1.43 | 3.15 | 3.45 | 2.20 | **8.80** | **7.79** | **8.01** | 8.80% |
| 100% | 3.87 | 5.79 | 6.59 | 4.45 | **27.79** | **25.31** | **20.54** | 27.79% |

Joints 5/6/7 have 12 N·m limits rather than 87 **and** much lower inertia, so the same
absolute discretization error is a larger fraction and the lighter links make a stiffer
ODE. Both effects land on the same three joints.

Confirmed over 24 random poses, phases and speeds (correlation with speed 0.596). The
scatter at frac 0.75 — 2.47% to 28.68% — means the floor depends on **pose as well as
speed**, so a single scalar threshold will be wrong somewhere in the workspace.

### Converted into payload terms

At 4.63 N·m/kg on joint 4:

| speed | floor at j4 | smallest detectable payload | 3-sigma threshold |
|---|---|---|---|
| 10% | 0.174 N·m | 0.038 kg | 0.113 kg |
| 25% | 0.457 | 0.099 | 0.296 |
| 50% | 0.987 | 0.213 | 0.640 |
| 75% | 1.917 | 0.414 | 1.242 |
| 100% | 3.870 | 0.836 | **2.507** |

**At full speed most of the 0.1-3.0 kg ladder is invisible.** Our recommendation, and we
think it is close to forced: run O4 and O5 at or below 50% of velocity limit.

### A conflict worth agreeing on now

O6 benchmarks trajectory styles and wants fast motion. O2/O4/O5 need slow motion to stay
below the knee. They cannot share a speed regime. O6 should report cycle time, peak torque
and energy at speed, while any residual-based claim stays below the knee — and the results
should say which regime each number came from rather than mixing them.


---

# CORRECTION 2026-11-03 — the residual floor, measured properly

**This supersedes the envelope table sent earlier in this document, and the "time
discretization, first order in dt" claim. Both were wrong.** Full detail in
`docs/RESIDUAL_FLOOR.md`.

The earlier rate sweep rebuilt the `World` once per rate inside a single process, which
does not give a clean plant — your R2.3 instinct to question it was right, and the
inconsistency you spotted (27.98% and 2.78% both at 1000 Hz) was the symptom. Re-run with
one container per rate, identical settle/warm-up/measurement windows in simulated seconds,
and trajectory amplitude a function of speed only:

| speed | 250 Hz | 500 Hz | 1000 Hz | 2000 Hz | rate-dependent? |
|---|---|---|---|---|---|
| 10% | 0.43% | 0.43% | 0.43% | 0.43% | no (x1.00) |
| 20% | 0.85% | 0.86% | 0.86% | 0.86% | no (x1.00) |
| 30% | 1.26% | 1.27% | 1.27% | 1.27% | no (x1.00) |
| 50% | **2.05%** | **2.05%** | **2.05%** | **2.05%** | no (x1.00) |
| 75% | 3.08% | 4.29% | 8.79% | 17.96% | **YES (x5.8)** |
| 100% | 7.26% | 14.85% | 30.21% | 53.76% | **YES (x7.4)** |

**Below ~50% the residual is physical** — identical to two decimals across an 8x range of
physics rate, and linear in speed. **Above ~50% a rate-dependent artifact takes over**,
roughly doubling per doubling of rate, reaching a peak of 112% of a joint's torque limit at
2000 Hz / full speed.

This is strong support for your R2.3 hypothesis. Below the knee the arm tracks well and the
drive is gentle; above it tracking falls to ~54% of commanded velocity, the drive works
hard, and the rate-dependent term appears. That is an implicit-drive signature, not the arm.

Also ruled out by measurement, so you need not re-test them: finite-difference noise
(analytic qdd gives the same answer), filtering (2%), timing alignment, Coriolis form,
solver iterations (4 vs 128), armature, velocity saturation, and mass-matrix slicing.

### For your thresholds

| speed | floor at j4 | smallest detectable payload | 3-sigma threshold |
|---|---|---|---|
| 10% | ~0.13 N*m | 0.03 kg | 0.09 kg |
| 30% | ~0.41 | 0.09 | 0.27 |
| 50% | ~0.69 | 0.15 | 0.45 |
| 100% (1 kHz) | ~3.70 | 0.80 | 2.40 |

**Recommendation: record O2/O4/O5 data at or below 50% of velocity limit.** Below the knee
the whole 0.1-3.0 kg ladder is detectable and the number does not depend on a simulator
setting.

### Your R2.3 test is still the right next step

We have not yet run effort mode. If the above-knee residual collapses there, it belongs to
the position drive and all attestation data should be recorded in effort mode.

### Confirmations from a full windowed run

- payload response **4.629 N*m/kg** at joint 4 with the TCP attachment, against your 4.60
- an unknown **2 kg** object weighed from torque alone to 0.01-0.03% at three poses
- your scaled confidence formulation reproduced in the simulator: raw c = 0.1238 at home,
  scaled c = 0.2127, c_vertical = 0.2686, force gain 4.85 — and the elbow-singular pose
  correctly gated at a force gain of 27.16
