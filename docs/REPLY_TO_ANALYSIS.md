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
