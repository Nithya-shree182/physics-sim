# Context — pick up here

Physics Attestation Layer: a Franka Panda digital twin in Isaac Sim that audits the arm by
comparing measured torque against model torque. Proposal is the PDF in the repo root.

Last worked: 2026-11-03. Branch `dev`. **Nothing committed since the first commit.**

---

## Run it

```bash
./start_sim_docker.sh                # windowed (default); writes out/physics_audit.json
HEADLESS=1 ./start_sim_docker.sh     # no window
```

~60–90 s per run. **Isaac Sim hangs on boot roughly 1 start in 5** — empty `docker logs`,
container `unhealthy`, GPU idle. Fix is `docker rm -f pal-isaac-sim` and relaunch. Use a
watchdog loop when scripting it:

Run any probe script through the watchdog runner:

```bash
tools/run_sim.sh pal_fast_random.py            # -> logs/pal_fast_random.log
PAL_STEPS=5000 PAL_PAYLOAD_KG=2 tools/run_sim.sh pal_capture.py
```

It waits for the script's own `[PAL]` marker rather than log volume, allows 420 s for
boot, kills what it abandons, tells a crash from a hang, and prints the real boot time.
The earlier inline watchdog counted log lines and gave up at 200 s, which reported slow
boots as hangs — so the old "1 in 5" hang figures measured the detector, not the sim.

## Layout

```
workspace/pal_config.py        every parameter, each tagged with its source
workspace/pal_scene.py         builds the scene, then audits its own physics
workspace/pal_*_test.py        one-off probes, each answering one question (see below)
assets/franka_panda/*.urdf     copied in from the Physyk project
docs/PINOCCHIO_HANDOFF.md      for the analysis side
docs/OPEN_PROBLEMS.md          what is broken, what is next
ARM_CONFIG.md                  why each parameter value is what it is
INTEGRATION.md                 reconciliation with the nithya branch's WORK_SPLIT.md
out/                           regenerated every run, git-ignored
```

The probe scripts are kept deliberately — each one is the evidence for a claim in the docs:

| script | answers |
|---|---|
| `pal_torque_test.py` | is `tau_applied` the same signal as `tau_measured`? |
| `pal_model_test.py` | payload sweep, g(q) vs J(q) cross-check, slow calibration |
| `pal_jacobian_fix.py` | which Jacobian row is the end effector? |
| `pal_o1_test.py` | can telemetry run at 1 kHz? frame coherence? |
| `pal_payload_demo.py` | O4 end to end on known masses |
| `pal_dynamics_excite.py` | does the identity close under fast motion? |
| `pal_timing_rate.py` | pre/post/midpoint state; physics rate vs throughput |
| `pal_coriolis_test.py` | which model form matches tau_measured? |
| `pal_drive_test.py` | what does tau_measured actually represent? |
| `pal_residual_diag.py` | ablation: iterations, rate, armature, stiffness |
| `pal_capture.py` | writes a telemetry run in the agreed schema |
| `pal_inertia_dump.py` | per-body mass / COM / inertia across all three sources |
| `pal_author_plant.py` | authoring the plant from the URDF |

Run one with the same `docker run` pattern as `start_sim_docker.sh`, substituting the
script name.

---

## The one thing to understand first

**The shipped `franka.usd` is not a dynamically credible Franka.** Measured:

- every body's centre of mass is exactly `[0,0,0]`
- every body's inertia is ~1e-6 kg·m² (round-trip verified: the API is in kg·m², the
  asset really is inertia-free)
- per-link masses differ from the URDF by up to 4.5×, with the total forced to 18.0000 kg

Dynamically it is a stick figure of point masses on the joint axes. That is why its `g(q)`
is exactly zero on joints 3 and 5 at the home pose.

**The scene now authors mass, COM and inertia from `panda_arm_hand.urdf` at build time.**
After that the plant agrees with the analysis side's independent Pinocchio model:

```
Pinocchio (URDF)  : [0, −4.58,  −0.64,  22.88,  0.62,  2.50,  0]
Isaac Sim (authored): [0, −4.663, −0.636, 22.954, 0.623, 2.441, 0.001]
```

Two independent engines, same parameters, same answer. Do not revert this.

## Three traps, in order of time lost

### 1. `world.reset()` discards tensor-API writes

**`world.reset()` silently discards everything written through the tensor API** — body
masses, COMs, inertias, joint friction, armature. All of it reverts to the asset's values.

The writes succeed, the readbacks confirm them, and then a later reset throws it away. For
a while the log claimed URDF parameters were applied while every downstream number was the
stock asset.

**Rule: apply tensor-API values AFTER the final `world.reset()`, and verify AFTER it too.**
A readback taken immediately after the write proves nothing. `pal_scene.py` prints a
`[survive]` line for exactly this reason; if it says `False`, stop and fix the ordering
before trusting any number in the run.

### 2. The boot hang

Roughly 1 start in 5 early on, three in a row during the capture work. Empty `docker
logs`, `unhealthy`, GPU idle. Always use the watchdog loop above. A long-lived container
that scripts are sent into would remove this and the ~60 s boot cost per probe — worth
doing if the probe count keeps growing.

### 3. The container runs as uid 1234

Not root, not the host user. A host directory at the usual 755 is **not writable**, and it
fails at the *end* of a run after all the compute. `chmod 777 data out` before capturing.

---

## Decisions already made — do not relitigate

| | decision |
|---|---|
| **Armature** | Stays in the plant at 0.0061 kg·m². Analysis side sets `model.armature = 0.0061` in Pinocchio to match. A **contract**: changing one side alone adds ~27 g of phantom payload. |
| **Friction classes** | Stiction and bearing wear kept **indistinguishable** (one PhysX scalar). **Always label this class `friction (stiction / bearing wear)`**, never one alone. |
| **Friction source** | Gaz et al. 2019, not NVIDIA's sysid values. Both in `pal_config.py`. |
| **Physics rate** | 1000 Hz, matching `WORK_SPLIT.md` and the real Franka loop. |
| **Camera** | Isometric overview only. Wrist camera deliberately removed. |
| **Inertial params** | Authored from the URDF, not inherited from the asset. |

Dropped from the open list at the user's direction: config format, licensing, report
ownership.

---

## Where each objective stands

| | status |
|---|---|
| **O1** telemetry | All 7 quantities available. Writer implemented against the analysis side's schema (`pal_telemetry.py` + `pal_capture.py`). Cannot stream at 1 kHz; `M`/`J` decimation is the agreed trade. |
| **O2** residual | Closes at 0.15% at rest, <1% slow, ~2.8% fast at 1 kHz. The fast-motion residual is **time discretization, not model error** — first order in dt, confirmed. Wrench conversion fixed. |
| **O3** confidence | `c(q)` works directionally but never exceeds 0.13 anywhere; needs rescaling. Threshold is a guess. |
| **O4** payload | Chain verified to 0.04% at rest across 0.1–3.0 kg. Estimator not built. |
| **O5** detection | Plant now has real friction. Detector not built. |
| **O6** benchmark | Limits in config, generators not written. |
| **O7** dashboard | Not started. Blocked by O1 throughput at 1 kHz. |
| Validation push | `switch_control_mode("effort")` verified to work. |

---

## The operating envelope — know these numbers

**Run O4 and O5 at or below 50% of velocity limit.** Below the knee the floor is under 2%
on every joint and the whole 0.1–3.0 kg payload ladder is detectable; above it the wrist
joints collapse.

```
speed   worst joint   smallest detectable payload   3-sigma threshold
 10%       0.39%              0.038 kg                   0.113 kg
 25%       0.96%              0.099                      0.296
 50%       1.81%              0.213                      0.640
 75%       8.80%   <- knee    0.414                      1.242
100%      27.79%              0.836                      2.507
```

The knee is joints 5/6/7: 12 N·m limits instead of 87, and lower inertia, so the same
absolute error is a bigger fraction *and* a stiffer ODE. Confirmed over 24 random
poses — the floor depends on pose as well as speed (2.47% to 28.68% at frac 0.75).

**Objective conflict:** O6 wants fast motion; O2/O4/O5 need slow. They cannot share a
speed regime, and the results must say which regime each number came from.

## Why that floor exists

The fast-motion residual is **time discretization of a stiff drive**, first order in `dt`
(fitted exponent 1.049). Not a modelling error, and not removable by a better model:

```
200 Hz -> 50.91%     500 Hz -> 14.24%     1000 Hz -> 2.78%     2000 Hz -> 1.44%
   (worst joint, as a fraction of its torque limit, on a 100%-of-limit trajectory)
```

Controls: solver iterations change nothing (4 vs 128), armature changes nothing (so it
*is* inside the reported `M`). Residual rises with drive stiffness for the same reason.

**O5 detection thresholds must sit above this floor, and it must be reported alongside
any detection rate.** It is also why `PHYSICS_HZ` stays at 1000 despite telemetry not
streaming in real time there.

## Next three things

1. **Validate a captured run end-to-end** against `schema/validate.py`. The capture was
   still fighting boot hangs when the session ended; `data/` needed `chmod 777`.
2. **Rescale the Jacobian and sweep `c(q)`** — though the analysis side has now claimed
   this (their §5), so confirm before starting.
3. **Wire the fault injectors (O5)** and the effort-mode validation push.

Full list with evidence in `docs/OPEN_PROBLEMS.md`. Our reply to the analysis side's notes
is `docs/REPLY_TO_ANALYSIS.md`.
