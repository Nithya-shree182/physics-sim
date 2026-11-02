# Context: where the analysis side stands

Read this first when picking the work up again. Status as of 2026-11-02. The changes
themselves are listed in [changes.md](changes.md).

## The project in one paragraph

A Franka Panda digital twin in NVIDIA Isaac Sim that audits the arm from physics: for every
joint at every physics step, compare the torque the arm actually produced (measured) with the
torque an independent model says it should need (model). The gap is the **residual**. Near
zero means normal; a steady offset means an unmodelled load (payload); a spike means a
collision, fault or inconsistent command; a slow drift means wear. The full plan is in the
proposal PDF "Physics Attestation Layer: A Physics-Based Digital Twin for Robot Arm
Monitoring" (kept locally, not committed) and in [WORK_SPLIT.md](WORK_SPLIT.md).

## Who does what

| | Nithya (Person B) | dev (Person A) |
|---|---|---|
| Machine | RTX 4070 laptop, Windows | RTX 5090 laptop, Isaac Sim 6.0.1 in Linux Docker |
| Branch | `nithya` | `dev` |
| Owns | independent Pinocchio model, residual, confidence score, payload estimator, detector, dashboard, schema | Isaac Sim scene, telemetry capture and Parquet writer, trajectories, fault and attack injection |

Repo: https://github.com/Nithya-shree182/physics-sim. The two branches are deliberately
different so the work runs in parallel.

## Agreed conventions (do not change one side alone)

| Item | Value | Source |
|---|---|---|
| Residual | `r = tau_measured - tau_model` | proposal section 6; `WORK_SPLIT.md` 3.1; dev's code |
| Model torque | `M(q) qdd + C(q,qd) qd + g(q) + tau_f(qd)` | proposal section 6, plus friction per dev's handoff |
| Armature | 0.0061 kg·m² on all 7 joints, set as `model.armature` | contract with dev (`PINOCCHIO_HANDOFF.md` 3.1) |
| Friction | Gaz et al. 2019 Table I sigmoid (`FRICTION_PHI1..3`) | `docs/SOURCES.md`; dev's `friction_sigmoid()` |
| Gravity | (0, 0, -9.81) | both sides |
| Physics rate | 1000 Hz | dev changed from 400 to match `WORK_SPLIT.md` |
| Joints | arm = first 7 of the simulator's 9 DOF; fingers separate | dev's audit |
| End-effector Jacobian | `get_jacobians()[0, 7]` = `panda_hand` (not the last row) | dev's handoff section 6 |
| Units | SI, per radian (never USD's per-degree values) | dev's `ARM_CONFIG.md` section 1 |
| Friction fault label | always `friction (stiction / bearing wear)`, never one alone | dev's decision |
| Gripper | hand 0.73 kg + 2 fingers 0.1 kg are part of the robot, not payload | dev's handoff |
| Model must not know the payload | the twin is built from the empty-arm URDF only | dev's handoff section 1 |

## Environment (Windows)

```
conda env create -f environment.yml      # once; creates the "pal" env with pinocchio 4.1.0
conda activate pal
```

- `pip install pin` fails on Windows (it tries to build Boost); use conda-forge.
- If `conda activate` does nothing in PowerShell, run `conda init powershell` once and open a
  new terminal. Check with `python -c "import sys; print(sys.executable)"`: it must end in
  `envs\pal\python.exe`.
- Always run inside the activated env. Running the env's `python.exe` directly without
  activation makes numpy load an incompatible LAPACK DLL from elsewhere on PATH (MATLAB), and
  it crashes with Windows code `0xC06D007F` in `eigvalsh`.

## What exists on the analysis side

| File | What it does |
|---|---|
| `panda_arm_hand.urdf` | arm + hand URDF; the model is built from this one |
| `panda.urdf` | arm only; do not use for the model (misses the 0.93 kg gripper) |
| `model/pinocchio_model.py` | `load_panda()` → 7-DOF model, fingers locked open, armature, friction. Methods: `g`, `M`, `coriolis`, `friction`, `tau_model`, `forward_dynamics`, `jacobian`, `ee_pose`, `potential_energy` |
| `model/poses.py` | named poses (home, near-singular, folded, horizontal reach) and joint limits, copied from dev's `pal_config.py` |
| `model/gravity_check.py` | 10 self-tests that need no simulator, plus `--expect` comparison against simulator `g(q)` |
| `model/synth_log.py` | synthetic schema-valid logs from a Pinocchio "plant" (see below) |
| `schema/telemetry_v0.yaml` | the per-step telemetry frame and run manifest (draft, awaiting dev's sign-off) |
| `schema/validate.py` | checks a Parquet log and its manifest against the schema |
| `environment.yml` | conda env |
| `WORK_SPLIT.md` | the two-person plan |
| `docs/SOURCES.md`, `docs/references/` | parameter sources and the Gaz / datasheet PDFs |
| `docs/NOTES_FOR_DEV.md` | findings and requests for dev |

How to run:
```
python model/gravity_check.py                      # expect 10 PASS
python model/gravity_check.py --pose home --expect 2=-1.655,4=20.077,6=0.856   # currently NO MATCH, see open issues
python model/synth_log.py --batch                  # 9 standard runs into data/synthetic/ (~25 s)
python schema/validate.py data/synthetic/<run>.parquet
```

### The synthetic generator, briefly

- **Plant**: the same URDF with link masses scaled by U(0.97, 1.03), friction ×1.15, sensor
  noise (q 2e-5 rad, qd 2e-3 rad/s, tau 0.05 N·m), an optional payload sphere 0.1034 m along
  `panda_hand` z, and an optional fault from half way: `friction` (one joint ×3), `stuck_joint`
  (exact constrained dynamics), `delay` (commands 50 ms late).
- **Controller**: joint impedance with libfranka gains (Kp 600/600/600/600/250/150/50,
  Kd 50/50/50/20/20/20/10) plus feedforward from the clean twin, torque mode, 1 kHz,
  semi-implicit Euler.
- **Trajectories**: `hold`, `slow_reference` (0.1 Hz sine), `excitation` (5-harmonic
  multi-sine at 0.15 Hz, under 60% of speed limits).
- **Timing**: tau at row k moves qd[k] to qd[k+1], so a forward difference of qd gives that
  step's qdd.
- **Verified**: with no mismatch and no noise, `tau_measured - tau_model` is 1e-13 N·m. A
  1 kg payload at rest gives a residual of `[0, -3.00, 0, 4.60, 0, 0.83, 0]` N·m.
- **Limitation**: the plant comes from the same URDF as the twin, so results are optimistic.
  Synthetic logs prove the code works; reported numbers must come from Isaac Sim logs.

## Open issues

1. **The simulated arm is not the URDF's arm (blocking).** At home the simulator gives
   `g = [0, -1.655, 0, 20.077, 0, 0.856, 0]`; the model gives `[0, -4.584, -0.636, 22.881,
   0.623, 2.497, 0]`. The simulator's exact zeros on joints 3 and 5 mean the asset's centres
   of mass differ from the URDF's. Waiting on dev for per-body mass/COM/inertia from the
   articulation, then a joint decision (recommended: make the plant match the URDF).
2. **Schema awaiting dev's sign-off**, and dev must write the Parquet writer against it.
3. **Payload placement unclear**: dev's documents give 4.63 and 4.90 N·m/kg on joint 4; the
   synthetic plant gives 4.60. Waiting on dev to say where the payload is attached.
4. **Confidence score ownership**: proposed that Nithya takes it; awaiting dev's confirmation.
5. Dev-side problems that affect us later: P1 (the simulator's own torque identity fails at
   6% under fast motion), P2 (friction never reaches the plant), P4 (full telemetry is 0.42×
   real time at 1 kHz), P5 (torque convention unverified).

All requests to dev are written up in [docs/NOTES_FOR_DEV.md](docs/NOTES_FOR_DEV.md).

## Next steps (in order)

1. **Residual calculator** `observers/residual.py`: `r = tau_measured - tau_model` with qdd
   from a (filtered) forward difference of qd; end-effector wrench via `pinv(J^T)` at
   `panda_hand`; refuse a position-mode log when `tau_applied` is requested. Tests on synthetic
   logs: about zero with `--no-mismatch --no-noise`; a 1 kg payload shows about 9.81 N down.
2. **Confidence score** `observers/confidence.py`: scale the Jacobian's rotational rows by
   0.472 m, sweep a few thousand random poses inside the joint limits (joint 4 is always
   negative), set the threshold from the distribution, and prototype a vertical-direction
   confidence for the payload estimate.
3. **Payload estimator**, then **detector** (threshold baseline first, with and without the
   confidence gate), then a **dashboard** that replays logs.
4. When dev sends real logs: validate them, rerun the gravity comparison, and switch the
   pipeline from synthetic to real data.

## Working rules

- Commit as **Nithya** (`Nithya-shree182 <nithyashreer23cy@rnsit.ac.in>`; already set in this
  repo's git config). No Claude co-author line and no mention of Claude in commit messages.
- Never commit: the proposal PDF, dev's `ARM_CONFIG.md` (a local copy of dev's file),
  `.vscode/`, `__pycache__/`, or `data/`. Add files to commits explicitly.
