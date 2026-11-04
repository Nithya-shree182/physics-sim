# Context — pick up here

Physics Attestation Layer: a Franka Panda digital twin in Isaac Sim that audits the arm by
comparing measured torque against model torque. Proposal is the PDF in the repo root.

Last worked: 2026-11-04. Branch `dev`, which now also contains the analysis side (merged from `nithya`; see `docs/README.md` and `docs/STATUS.md`). **Start with `README.md` and `docs/REVIEW.md`.** This file is the long-form
history; where it disagrees with REVIEW.md (written later the same day), REVIEW.md wins.

---

## Run it

```bash
tools/pal_suite.sh o5                            # current way: sim -> Parquet -> checks; --window to watch (see README.md)
tools/run_sim.sh pal_selftest.py                 # plant self-test; run after any change to the plant
.venv-pal/bin/python -m pytest -q                   # all 138 tests, both teams; no simulator needed
```

Older scripts and the notes below predate 2026-11-03 (the scene audit and early probes build the OLD plant, whose
friction was in the wrong units). Their friction-related numbers are superseded by `docs/REVIEW.md`.

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
tools/run_sim.sh pal_selftest.py
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
| `pal_selftest.py` | plant self-test: friction in N*m, identity, mid-run inertials, push recovery |
| `pal_suites.py`, `pal_run_suite.py` | every objective's scenarios (reference, o3, o4, o5, clean, fast, extra, push, o6, pick, o6pick, pickphys) and the runner; replaced the per-objective scripts |
| `pal_ik.py`, `pal_tasks.py`, `pal_objects.py` | IK, pick-and-place and ellipse tasks, the physical table and cube |
| `pal_gripper_probe.py` | measured finger geometry and drives (evidence for the physical grasp) |
| `pal_inertia_convention.py` | which convention `set_body_inertias` uses (full 3x3 in the link frame; evidence for the fix) |
| `pal_faults.py` | O5 fault injectors |
| `pal_friction_visibility.py` | is injected friction visible in `tau_measured`? in `tau_applied`? (only the latter) |
| `pal_effort_test.py` | does the residual artifact survive in effort mode? (yes) |
| `pal_rate_decouple.py` | does controller rate matter? (no) |
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

## Resolved 2026-11-03 (details and evidence in `docs/REVIEW.md`)

- Friction: the old `set_friction_coefficients` takes a dimensionless coefficient; we had passed N*m (about 200x too
  much on joint 2). `pal_plant.py` now uses the torque-valued `set_dof_friction_properties`; verified by
  `pal_selftest.py`.
- `tau_measured` is the net torque and cannot show friction, backlash or a locked joint. Fault data is now recorded in
  effort mode and the fault residual uses `tau_applied`. `tau_measured` is still right for external loads.
- Decided: faults, not attacks (`docs/PROPOSAL_AMENDMENTS.md`).

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
| **Speed regime** (decided 2026-11-03) | O2/O4/O5 record at **<= 50% of velocity limit**, where the floor is physical (< ~2% of a limit). O6 may run faster but is reported separately. |
| **O5 scope** (decided 2026-11-03) | **Faults, not attackers.** Nobody tampers with goals or object positions. Injected: friction / wear changes, mass or payload change, actuation delay, sensor noise. Tampered-goal attacks are out of scope (A2 stays a stated limit). O5 also records how the arm copes: tracking error and motion under each fault. |

Dropped from the open list at the user's direction: config format, licensing, report
ownership.

---

## Where each objective stands

| | status |
|---|---|
| **O1** telemetry | All 7 quantities available. Writer implemented against the analysis side's schema (`pal_telemetry.py`; the runs come from `pal_suites.py`). Cannot stream at 1 kHz; `M`/`J` decimation is the agreed trade. |
| **O2** residual | Closes at 0.15% at rest, <1% slow, <2% up to 50% of velocity limit; above it an artifact that grows with physics rate (see `docs/RESIDUAL_FLOOR.md`), not model error. Wrench conversion fixed. |
| **O3** confidence | `c(q)` works directionally but never exceeds 0.13 anywhere; needs rescaling. Threshold is a guess. |
| **O4** payload | Known-mass data recorded (0.5-3 kg, three poses + moving) and verified to 0.02% with independent URDF kinematics (`tools/check_payload.py`). The estimator is the analysis side's (`observers/payload.py`); not run on this data. |
| **O5** detection | Built: friction, stuck joint, delay, backlash injectors; 5% mass mismatch; seeded sensor noise; 26 labelled runs in effort mode, plus the analysis side's requested set in `out/extra` (`docs/O5_INJECTORS.md`). Backlash and short delays are invisible at this speed. Detector is the analysis side's; not yet run on our data. |
| **O6** benchmark | Built and run: trapezoidal, S-curve, quintic over a speed grid at 0 and 2 kg, 26 runs; table and chart in `docs/REVIEW.md`, `docs/o6_comparison.svg`. |
| **O7** dashboard | Dashboard is the analysis side's. Our parts (websocket stream, in-sim panel) not built. |
| Validation push | Built and run (24 runs). Exact with friction off (0.2%); with realistic friction 15.8% worst case against a 15% pre-set limit: strict verdict FAIL, cause identified (`docs/VALIDATION.md`). |

---

## The operating envelope — know these numbers

**Full detail in `docs/RESIDUAL_FLOOR.md`.** Worst-joint RMS residual as a fraction of that
joint's torque limit:

| speed | 250 Hz | 500 Hz | 1000 Hz | 2000 Hz | rate-dependent? |
|---|---|---|---|---|---|
| 10% | 0.43% | 0.43% | 0.43% | 0.43% | no (x1.00) |
| 20% | 0.85% | 0.86% | 0.86% | 0.86% | no (x1.00) |
| 30% | 1.26% | 1.27% | 1.27% | 1.27% | no (x1.00) |
| 50% | **2.05%** | **2.05%** | **2.05%** | **2.05%** | no (x1.00) |
| 75% | 3.08% | 4.29% | 8.79% | 17.96% | **YES (x5.8)** |
| 100% | 7.26% | 14.85% | 30.21% | 53.76% | **YES (x7.4)** |

**Two regimes.** Below ~50% the residual does not move when the physics rate changes by 8x
— it is physical, and no rate setting will reduce it. Above ~50% it roughly doubles per
doubling of rate, reaching a peak of 112% of a joint's torque limit at 2000 Hz / full
speed, which cannot be a real reading.

**Run O2, O4 and O5 at or below 50% of velocity limit.** That is not a judgement call: it
is where the measurement stops depending on a simulator setting.

**O6 conflicts with this** and cannot be fixed — benchmarking wants speed. Report its
timing and energy figures at speed, keep residual claims below the knee, and say which
regime each number came from.

Verified in practice at rest: an unknown 2 kg object weighed to 0.01-0.03% at three poses;
payload response 4.629 N*m/kg, matching the analysis side's independent 4.60.

## Why the above-knee artifact exists

The above-knee residual is an **artifact that grows with physics rate** (30.21% at 1 kHz,
53.76% at 2 kHz at full speed; rate-independent below ~50% of velocity limit) — see
`docs/RESIDUAL_FLOOR.md`. The earlier "time discretization, first order in dt" claim
(200/500/1000/2000 Hz -> 50.91/14.24/2.78/1.44%) was **wrong**: that sweep rebuilt `World`
in one process and the new `physics_dt` was ignored. Controller rate is irrelevant
(`pal_rate_decouple.py`: 2 kHz physics with a 1 kHz controller gives 35.31% vs 35.26%).
Not a modelling error, and not removable by a better model.

Controls: solver iterations change nothing (4 vs 128), armature changes nothing (so it
*is* inside the reported `M`). Residual rises with drive stiffness for the same reason.

**O5 detection thresholds must sit above this floor, and it must be reported alongside
any detection rate.** It is also why `PHYSICS_HZ` stays at 1000 despite telemetry not
streaming in real time there.

## What is left

1. Analysis side: run `observers/` on `data/ref_matched_*`, `out/o5`, `out/o4`; report detection and false-positive rates
   with and without the confidence gate.
2. Ours, not built: websocket stream and in-sim panel (O7); a grasped payload with gripper actuation; backlash with
   impacts; faster-motion fault sets.
3. Keep `docs/REVIEW.md` current when results change.

Full history with evidence in `docs/OPEN_PROBLEMS.md`.
