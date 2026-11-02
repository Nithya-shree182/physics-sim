# Changes on the `nithya` branch

Newest first. Current status and next steps are in [context.md](context.md).

## 2026-11-02 — friction, synthetic logs, review of dev's update

**Model**
- `model/pinocchio_model.py`: added joint friction (Gaz et al. 2019 Table I sigmoid, same
  values as dev's `friction_sigmoid()`), scalable per joint via `friction_scale`; switch off
  with `load_panda(friction=False)`. `tau_model` now includes friction by default
  (`include_friction=False` gives rigid-body only). New `forward_dynamics` (aba with friction)
  and `ee_pose` (position and unit quaternion w, x, y, z). Docstring: residual sign is
  `tau_measured - tau_model`.
- `model/gravity_check.py`: two new self-tests, friction zero at rest, and forward dynamics
  exactly inverts `tau_model` (3e-14 N·m). 10/10 pass.

**Synthetic data**
- New `model/synth_log.py`: Pinocchio plant (masses ±3%, friction ×1.15, sensor noise,
  optional payload and faults: friction, stuck joint with exact constrained dynamics, command
  delay) under a libfranka-gain joint impedance controller with twin feedforward, 1 kHz,
  torque mode. Writes schema-valid Parquet + JSON manifest into `data/synthetic/` (not
  committed). `--batch` writes 9 standard runs.
- Fixed during testing: the first stuck-joint version froze the joint after each step, which
  let the saturated joint torque leak into the other joints (joint 6 reached 4.40 rad, past
  its 3.75 rad limit). Replaced with constrained dynamics; all joints now stay in limits.

**Schema and docs**
- `schema/telemetry_v0.yaml`: physics rate 1000 Hz (dev changed it); Jacobian row 7
  (`panda_hand`) per dev's measurement, never the last row; residual sign
  `tau_measured - tau_model`.
- `WORK_SPLIT.md` 3.1: residual is `tau_measured - tau_model`, following the proposal PDF
  (this reverses the earlier `tau_model - tau_measured` entry).
- `docs/SOURCES.md`: added the verified Gaz Table III values and the `pdftotext -layout`
  row-misalignment pitfall.
- New `docs/NOTES_FOR_DEV.md`: findings and requests for dev, including a retraction of our
  earlier claim that dev's Gaz friction arrays were shifted by one joint (they are correct).
- New `context.md` and this `changes.md`.

**Findings from dev's commit `c307c21`**
- At home, the simulator's gravity vector has exact zeros on joints 3 and 5 that no URDF
  reproduces: the simulated asset's centres of mass differ from the URDF's. Logged as the main
  open issue, with a request to dev.
- Dev planned to write the schema and the confidence sweep; both overlap with our work and
  are raised in `docs/NOTES_FOR_DEV.md`.

## 2026-11-02 — commit `7cfba9e`: independent model, gravity check, schema

- `model/pinocchio_model.py`: 7-DOF Panda from `panda_arm_hand.urdf`, fingers locked open
  (0.93 kg gripper kept as robot mass), armature 0.0061 set on `model.armature`.
- `model/gravity_check.py`: self-tests (moving mass equals URDF sum, torque limits, armature in
  `M` and `rnea`, `g(q)` equals dU/dq, `M` symmetric and positive definite) and the `--expect`
  comparison against simulator `g(q)`.
- `model/poses.py`: named poses and joint limits shared with dev's `pal_config.py`.
- `schema/telemetry_v0.yaml` and `schema/validate.py`: per-step telemetry frame, run manifest,
  and a validator; tested on a good and a deliberately broken log.
- `environment.yml`: conda env `pal` (pinocchio from conda-forge).
- `panda_arm_hand.urdf`, `panda.urdf` added to the repo.

## Earlier commits

- `e2c48f5` — `docs/SOURCES.md` and the reference PDFs (Gaz supplementary material, Franka
  Research 3 datasheet).
- `ec4df3a` — `WORK_SPLIT.md`, the two-person plan.
