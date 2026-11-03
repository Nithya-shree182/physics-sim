# Context: where the analysis side stands

Read this first when picking the work up again. Status as of 2026-11-03. The changes
themselves are listed in [changes.md](changes.md); what is waiting on dev is in
[docs/NOTES_FOR_DEV.md](docs/NOTES_FOR_DEV.md).

> **Update, 2026-11-03 (late evening):** parts of this file are out of date. Read
> [setup.md](setup.md) first (setting up another machine, where we stopped, what is stale here,
> one open lead) and [docs/DASHBOARD_PLAN.md](docs/DASHBOARD_PLAN.md) (dashboard decisions).
> Dev's correction of the residual-floor claim (`docs/NOTES_FOR_NITHYA.md` section 1, on
> `dev`) supersedes the "residual floor" paragraph below.

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
different so the work runs in parallel. Dev keeps its own `CONTEXT.md` and
`docs/OPEN_PROBLEMS.md` on `dev`.

## Agreed conventions (do not change one side alone)

| Item | Value | Source |
|---|---|---|
| Residual | `r = tau_measured - tau_model` | proposal section 6; `WORK_SPLIT.md` 3.1; dev's code |
| Model torque | `M(q) qdd + C(q,qd) qd + g(q) + tau_f(qd)` | proposal section 6, plus friction per dev's handoff |
| External wrench | `W = -(J^T)^+ r` at `panda_hand`, acting on the robot (1 kg held gives fz = -9.81 N) | `observers/residual.py` |
| Inertial parameters | from `panda_arm_hand.urdf` on both sides (dev authors them into the plant) | dev commit `23822b8` |
| Armature | 0.0061 kg·m² on all 7 joints, set as `model.armature` | contract with dev |
| Friction | Gaz et al. 2019 Table I sigmoid (`FRICTION_PHI1..3`) | `docs/SOURCES.md`; dev's `friction_sigmoid()` |
| Gravity | (0, 0, -9.81) | both sides |
| Physics rate | 1000 Hz | both sides |
| Joints | arm = first 7 of the simulator's 9 DOF; fingers separate | dev's audit |
| End-effector Jacobian | `get_jacobians()[0, 7]` = `panda_hand` (not the last row) | dev's handoff section 6 |
| Units | SI, per radian (never USD's per-degree values) | dev's `ARM_CONFIG.md` section 1 |
| Friction fault label | always `friction (stiction / bearing wear)`, never one alone | dev's decision; `FAULT_LABELS` in `observers/detector.py` |
| Gripper | hand 0.73 kg + 2 fingers 0.1 kg are part of the robot, not payload | dev's handoff |
| Model must not know the payload | the twin is built from the empty-arm URDF only | dev's handoff section 1 |
| Compare at the logged pose | evaluate the model at the logged `q`, never the commanded pose (the arm sags in position mode) | 2026-11-03 finding |
| Row timing | Isaac Sim logs: row assembled after the step, `tau[k]` produced row k → backward difference for qdd (manifest `timing_convention`). Synthetic logs: forward (manifest `timing`) | dev's `REPLY_TO_ANALYSIS.md` §3; `observers/residual.py` |
| Payload placement | point mass 0.1034 m along `panda_hand` z on both sides; 4.6306 N·m/kg on joint 4 at home | dev's reply §6 |
| Log delivery | Isaac container writes `.npz`; dev converts to Parquet with `tools/npz_to_parquet.py`; logs travel outside git (`data/` is ignored) | dev's reply §3 |

**Important caveat (dev's P6b):** the URDF values make twin and plant agree; nobody knows the
real Panda's parameters (Franka publishes none). No result may claim they are the real robot's.

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
| `panda_arm_hand.urdf` | arm + hand URDF; the model is built from this one (identical to dev's `assets/franka_panda/`) |
| `panda.urdf` | arm only; do not use for the model (misses the 0.93 kg gripper) |
| `model/pinocchio_model.py` | `load_panda()` → 7-DOF model, fingers locked open, armature, friction. Methods: `g`, `M`, `coriolis`, `friction`, `tau_model`, `forward_dynamics`, `jacobian`, `ee_pose`, `point_acceleration`, `potential_energy` |
| `model/poses.py` | named poses and joint limits, copied from dev's `pal_config.py` |
| `model/gravity_check.py` | 10 self-tests that need no simulator, plus `--expect` comparison against simulator `g(q)` |
| `model/synth_log.py` | synthetic schema-valid logs from a Pinocchio "plant" (below) |
| `schema/telemetry_v0.yaml`, `schema/validate.py` | per-step telemetry frame + run manifest, and its validator (draft, awaiting dev) |
| `observers/residual.py` | `load_run`, `compute_residual` (r, wrench, filtered qdd), position-mode guard |
| `observers/confidence.py` | scaled `c(q)`, vertical score, physical gates, workspace sweep |
| `observers/payload.py` | payload mass from the residual wrench, acceleration-corrected, gated, tare |
| `observers/detector.py` | threshold baseline on residual and tracking-error channels, evaluation against labels |
| `tests/` | 30 pytest tests over all of the above, on in-memory synthetic runs |
| `environment.yml`, `pytest.ini` | conda env; pytest settings |
| `WORK_SPLIT.md`, `docs/SOURCES.md`, `docs/references/` | plan, parameter sources, reference PDFs |
| `docs/NOTES_FOR_DEV.md` | findings and requests for dev |

How to run (inside `conda activate pal`):
```
python -m pytest                                   # 30 tests, ~20 s
python model/gravity_check.py                      # 10 self-tests
python model/synth_log.py --batch                  # 9 standard runs into data/synthetic/ (~25 s)
python schema/validate.py data/synthetic/<run>.parquet
python observers/residual.py data/synthetic/<run>.parquet
python observers/confidence.py --sweep 5000
python observers/payload.py data/synthetic/synth_hold_1kg_none_s0.parquet [--tare 0.0267]
python observers/detector.py data/synthetic/*.parquet
```

### Key numbers and choices (all `[CHOICE]`s are marked in the code)

- **Residual filter:** 20 Hz zero-phase Butterworth (`DEFAULT_CUTOFF_HZ`). Without it,
  differentiating noisy qd gives 2–6% of the torque limits of noise; with it, under 0.5%. The
  first and last 3 filter time constants (0.15 s) of a record are dropped (filter edge effect).
  Offline only; a live dashboard needs a causal filter.
- **Confidence:** rotational Jacobian rows scaled by L = 0.472 m. Gates: force noise gain
  `1/sigma_min` ≤ 10 N per N·m (detector), vertical noise gain ≤ 10 (payload). Display scores
  normalised by the 99th percentile of a 5000-pose sweep (`C_REF` 0.2239, `C_V_REF` 0.4070).
  Home 0.96, elbow-singular 0.15 (gated).
- **Payload:** `m = F·d/|d|²`, `d = g − a_payload`; running mean with 1 s time constant
  (starts as a plain average); payload at 0.1034 m along `panda_hand` z (to confirm with dev).
  Synthetic: exact without mismatch; with the ±3% mass mismatch about ±25–35 g phantom mass;
  a tare from the empty gripper at the same pose removes it at rest (1 kg → 1.0012 kg), but
  not during motion (1 kg excitation → 0.965 kg).
- **Detector:** 3 s fault-free calibration window, |z| > 8 for 50 consecutive steps, gated by
  pose. Thresholds were set on the same synthetic data they are tested on; **re-tune on real
  logs**.

### The synthetic generator, briefly

- **Plant**: the same URDF with link masses scaled by U(0.97, 1.03), friction ×1.15, sensor
  noise (q 2e-5 rad, qd 2e-3 rad/s, tau 0.05 N·m), an optional payload sphere 0.1034 m along
  `panda_hand` z, and an optional fault from half way: `friction` (one joint ×3), `stuck_joint`
  (exact constrained dynamics), `delay` (commands 50 ms late).
- **Controller**: joint impedance with libfranka gains plus feedforward from the clean twin,
  torque mode, 1 kHz, semi-implicit Euler.
- **Trajectories**: `hold`, `slow_reference`, `excitation` (5-harmonic multi-sine).
- **Timing**: tau at row k moves qd[k] to qd[k+1] (declared in the manifest).
- **Limitation**: the plant comes from the same URDF as the twin, so results are optimistic.
  Synthetic logs prove the code works; reported numbers must come from Isaac Sim logs.

## Open items

Resolved (dev's `34fac9d` and `docs/REPLY_TO_ANALYSIS.md`, 2026-11-03): the simulated arm
matches the URDF; dev adopted our schema unchanged and wrote the writer
(`workspace/pal_telemetry.py`, `pal_capture.py`); the confidence score is ours; payload
placement agreed; timing convention stated; `data/` ignored; friction values agreed.

First real logs arrived (`data/real/`, slow_reference 0 kg and 1 kg): schema valid, identity
closes, tare from the 0 kg run gives 1.000 kg. Still need fast/excitation runs and fault runs.

Waiting on dev (round 2 in `docs/NOTES_FOR_DEV.md`):
1. **One real captured log (Parquet + JSON), sent outside git.** This is what unblocks
   everything below.
2. Check that `ee_pos_*` actually changes in a moving capture (USD pose read may be stale),
   and that `t` matches the post-step state.
3. The effort-mode run that decides whether the ~2.8% fast-motion residual floor is
   fundamental or comes from the position drive's reported effort.
4. Copy the small additive `schema/` update from `nithya`.

Dev's stated residual floor (their P1): ~2.8% of a joint's torque limit on fast motion at
1 kHz in `position_stiff`, under 1% on slow motion, first order in `dt`. Until item 3 is
settled, detector thresholds must sit above it and it must be reported with any detection
rate. Also: full telemetry is ~0.76× real time at 1 kHz, so dev decimates `M`/`J`
(`mj_every`, `mj_fresh`); our pipeline uses its own `M` and `J` from Pinocchio, so this does
not affect us unless `--jacobian logged` is used.

## Next steps

When dev's first real log arrives (it comes as Parquet + JSON; put both in `data/real/`):
1. `python schema/validate.py` on it. Check `ee_pos_*` varies on a moving run.
2. Gravity comparison at the **logged** `q` of a rest segment (expect agreement to a few
   hundredths of a N·m).
3. Residual on a slow and a fast run: is the identity closing? Re-tune `DEFAULT_CUTOFF_HZ`.
4. Payload on dev's known-mass runs; set the tare procedure.
5. Re-tune the detector thresholds on real fault runs.

Can be done before then, without dev:
- **Dashboard** that replays a log in the browser (Plotly Dash): torque breakdown, residual
  with alarm band, confidence trace, payload estimate with band, alerts.
- **Classifier** on windowed residual and tracking features, split by trajectory and pose,
  to tell payload, friction, stuck joint and delay apart (proposal 7.5, after the threshold
  baseline).
- More synthetic variety (seeds, payloads picked up mid-run, more fault strengths) for the
  above.

## Working rules

- Commit as **Nithya** (`Nithya-shree182 <nithyashreer23cy@rnsit.ac.in>`; already set in this
  repo's git config). No Claude co-author line and no mention of Claude in commit messages.
- Never commit: the proposal PDF, dev's `ARM_CONFIG.md` (a local copy of dev's file),
  `.vscode/`, `__pycache__/`, or `data/`. Add files to commits explicitly.
