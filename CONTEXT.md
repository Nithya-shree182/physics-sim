# Context — pick up here

Physics Attestation Layer: a Franka Panda digital twin in Isaac Sim that audits the arm by
comparing measured torque against model torque. Proposal is the PDF in the repo root.

Last worked: 2026-11-02. Branch `dev`. **Nothing committed since the first commit.**

---

## Run it

```bash
./start_sim_docker.sh                # windowed (default); writes out/physics_audit.json
HEADLESS=1 ./start_sim_docker.sh     # no window
```

~60–90 s per run. **Isaac Sim hangs on boot roughly 1 start in 5** — empty `docker logs`,
container `unhealthy`, GPU idle. Fix is `docker rm -f pal-isaac-sim` and relaunch. Use a
watchdog loop when scripting it:

```bash
for attempt in 1 2 3; do
  docker rm -f pal-isaac-sim >/dev/null 2>&1
  HEADLESS=1 ./start_sim_docker.sh > logs/run.log 2>&1 &
  LPID=$!; booted=0
  for i in $(seq 1 36); do sleep 5
    [ "$(wc -l < logs/run.log 2>/dev/null || echo 0)" -gt 400 ] && { booted=1; break; }; done
  [ "$booted" = "1" ] && { wait $LPID; break; }
  kill $LPID 2>/dev/null
done
```

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

## The trap that cost the most time

**`world.reset()` silently discards everything written through the tensor API** — body
masses, COMs, inertias, joint friction, armature. All of it reverts to the asset's values.

The writes succeed, the readbacks confirm them, and then a later reset throws it away. For
a while the log claimed URDF parameters were applied while every downstream number was the
stock asset.

**Rule: apply tensor-API values AFTER the final `world.reset()`, and verify AFTER it too.**
A readback taken immediately after the write proves nothing. `pal_scene.py` prints a
`[survive]` line for exactly this reason; if it says `False`, stop and fix the ordering
before trusting any number in the run.

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
| **O1** telemetry | All 7 dynamics quantities available. **Cannot run at 1 kHz** — 0.42× real time, every step over budget. Schema and Parquet writer not written. |
| **O2** residual | Closes at 0.15% at rest and 0.49% on a slow run, but **FAILS at 5.98% under fast motion**. Wrench conversion fixed and verified. |
| **O3** confidence | `c(q)` works directionally but never exceeds 0.13 anywhere; needs rescaling. Threshold is a guess. |
| **O4** payload | Chain verified to 0.04% at rest across 0.1–3.0 kg. Estimator not built. |
| **O5** detection | Plant now has real friction. Detector not built. |
| **O6** benchmark | Limits in config, generators not written. |
| **O7** dashboard | Not started. Blocked by O1 throughput at 1 kHz. |
| Validation push | `switch_control_mode("effort")` verified to work. |

---

## Next three things

1. **Diagnose why the identity fails under fast motion** (5.98% on joint 3). Note the
   excitation test predates the inertial authoring, so **re-run it first** — the plant has
   changed substantially and the number may have moved.
2. **`schema/telemetry_v0.yaml` + Parquet writer.** The analysis side is blocked on this.
3. **Rescale the Jacobian, sweep the workspace, set a real `c(q)` threshold.**

Full list with evidence in `docs/OPEN_PROBLEMS.md`.
