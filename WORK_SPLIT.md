# Physics Attestation Layer: Work Split (2 people)

Project: a Franka Panda digital twin in Isaac Sim that audits the arm by comparing measured torque with model torque (the residual).

| | Person A | Person B |
|---|---|---|
| Machine | Omen, RTX 5090 | Alienware, RTX 4070 |
| Role | **Simulator side** | **Analysis side** |
| Runs Isaac Sim? | Yes, main machine | Smoke tests only |
| Final dataset generated on | **A's machine only** | uses A's data |

Timeline target: about 6 to 8 weeks.

## 1. Why this split works
Isaac Sim is only needed where data is produced. The dynamics model, residual math, confidence score, estimators, detector, and dashboard all run on recorded files, on either laptop. The handoff between A and B is a **versioned log file** with an agreed schema.

## 2. Ownership

### Person A: simulator side
- Scene setup: Panda in Isaac Sim, fixed physics step (target 1/1000 s), effort-control mode.
- L0 telemetry capture: one frame per physics step in the agreed schema.
- Trajectory generators (trapezoidal, S-curve, torque-aware) and the scripted pick-and-place.
- Fault injectors: friction, backlash, stuck joint, command delay.
- Attack injectors: altered waypoints, spoofed setpoints (ROS 2 or socket path).
- Realism layers: sensor noise and quantization, actuator delay and torque limits, deliberate model mismatch (perturbed masses, unmodelled friction).
- Large headless data generation runs and the run manifest.
- In-sim `omni.ui` panel and debug-draw overlays (last).

### Person B: analysis side
- Independent dynamics model (Pinocchio + Panda URDF + identified parameters), 7-DOF arm block.
- Calibration check: model vs simulator on a slow, unloaded run (error normalised by joint torque limit).
- Residual computation and conversion to end-effector wrench.
- Pose confidence score (Jacobian SVD, scaled so position and orientation rows are unit-consistent).
- Payload estimator (static first, then regressor-based during motion).
- Threshold detector, then the classifier (split by trajectory and pose, not by random window).
- Streamed dashboard (Plotly Dash + websocket), benchmarking plots, validation-push analysis.

### Done together
- Telemetry schema and the residual definition.
- Fault and attack list, and which attack classes are covered or not.
- Calibration sign-off (does the identity close?).
- Validation push test.
- Final demo run and the report.

## 3. Shared contract (agree on day 1)

### 3.1 Residual definition
`residual torque = model torque - measured torque`, that is
`r = tau_model - tau_measured`, with `tau_model = M(q) * qdd + C(q, qd) * qd + g(q)`.

- This is the agreed project definition. Every doc, script and plot uses this sign.
- `tau_measured` is the effort the arm actually produced, as reported by the simulator (`tau_measured_*` in the telemetry frame).
- `qdd` comes from filtered velocity differences.
- Sign consequence: an unmodelled payload makes the arm need more torque than the model predicts, so `r` comes out negative. A model that overestimates the load gives a positive `r`.
- Attack detection also uses `commanded - measured` tracking error. A tampered waypoint that is faithfully executed gives a near-zero residual, so the residual alone does not cover it. State this in results.

### 3.2 Telemetry frame (v0, per physics step, one row)
| Column group | Columns |
|---|---|
| Time | `t`, `step`, `dt` |
| State (7 arm joints) | `q_0..q_6`, `qd_0..qd_6` |
| Efforts | `tau_applied_0..6`, `tau_measured_0..6` |
| Dynamics terms (from sim) | `g_0..6`, `coriolis_0..6`, `M_ij` (7x7 flattened) |
| Kinematics | `J` (6x7 flattened), `ee_pos_xyz`, `ee_quat_wxyz` |
| Commands | `q_cmd_0..6`, `qd_cmd_0..6` |
| Gripper | `finger_pos`, `finger_force` |
| Labels | `run_id`, `traj_type`, `payload_kg`, `fault_type`, `fault_joint`, `fault_param`, `attack_type`, `attack_active` |

Format: Parquet, one file per run. The schema file lives in `schema/telemetry_v0.yaml` and is versioned. Changes need both people to agree and a version bump.

### 3.3 Run manifest (one JSON per run)
Git commit, Isaac Sim version, physics dt, seed, config file name, payload, fault and attack settings, machine name.

## 4. Repository layout and ownership
```
physics-based-robotic-arm/
  schema/        telemetry_v0.yaml            (shared)
  configs/       *.yaml                       (shared)
  twin/          l0_dynamics.py, scene.py     (A)
  motion/        trajectories.py, pick.py     (A)
  faults/        injector.py, attacks.py      (A)
  model/         pinocchio_model.py, calib.py (B)
  observers/     residual.py, confidence.py,
                 payload.py, detector.py      (B)
  ui/            dashboard/                   (B)
                 omni_panel/                  (A)
  analysis/      benchmarking, plots          (B)
  data/          (not in git; LFS or shared drive)
  docs/          results per phase            (shared)
```

## 5. Merging rules
- One Git repo; `main` is always runnable.
- Short-lived branches, small pull requests, reviewed by the other person.
- Do not edit the other person's folders without telling them.
- Pinned environments: `environment.yml` for B's side; A documents the Isaac Sim version and launch steps in `docs/setup_sim.md`.
- **Both machines run the same Isaac Sim version.** The 5090 (Blackwell) needs a recent Isaac Sim and driver. Confirm the supported combination before installing.
- Data stays out of Git. Name files by `run_id`, keep the manifest beside them.
- All tunable numbers (masses, friction, thresholds, noise, seeds) live in `configs/*.yaml`.
- Generate the final dataset on A's machine only. Physics output can differ slightly across GPUs and drivers.
- Weekly integration check: B runs the observers on A's latest logs; A confirms the schema has not drifted.

## 6. Unblocking each other
- B does not wait for the simulator: B generates a **synthetic log** from Pinocchio forward dynamics in the same schema and builds against it.
- A delivers a **short real sample log** within the first week so B can switch to real data early.
- Live integration (late): A's sim streams frames over a websocket; B's dashboard connects. B tests this earlier by replaying a log through the same websocket.

## 7. Week-by-week

| Week | Person A | Person B | Together |
|---|---|---|---|
| 1 | Install and pin Isaac Sim. Load Panda, fixed step, effort mode. First sample log. | Pinocchio model of the 7-DOF arm. Synthetic log generator. Schema validator. | Agree schema v0 and the residual definition. |
| 2 | Telemetry capture complete. Slow reference trajectory. | Calibration script and residual plots. Confidence score. | Close the identity (error within limit-normalised tolerance). |
| 3 | Trajectory bank. Scripted pick. Payload runs with known masses. | Payload estimator on real logs. Torque decomposition plot. | Review payload accuracy and unobservable poses. |
| 4 | Fault and attack injectors. Sensor noise and actuator delay layers. | Threshold detector. Dashboard v1 (live from replay). | Fix fault and attack list. |
| 5 | Generate full labelled dataset (single machine). Validation push setup. | Classifier. Evaluation with and without the confidence gate. | Validation push comparison (applied vs estimated force). |
| 6 | Websocket streaming. Benchmarking runs. | Benchmarking plots (time, peak torque, energy). Dashboard v2. | Integration and demo rehearsal. |
| 7 to 8 (buffer) | In-sim panel and overlays. | Report figures and tables. | Final demo and report. |

## 8. If time runs short (cut order)
1. Cut the in-sim panel (keep the browser dashboard).
2. Cut trajectory benchmarking (O6) to two trajectory types.
3. Cut command-attack scenarios, keep mechanical faults.
4. Never cut: telemetry, residual and calibration, confidence gate, payload estimate, validation push.

## 9. Definition of done per objective
| Objective | Done when |
|---|---|
| O1 Telemetry | No dropped frames over a full run; schema validator passes |
| O2 Residual | Slow unloaded run within tolerance, error normalised by joint torque limit |
| O3 Confidence | Score drops near singular poses; estimates are visibly suppressed |
| O4 Payload | Known masses recovered within stated tolerance; unobservable case flagged |
| O5 Detection | Rates reported with and without the gate; covered attack classes listed |
| O6 Benchmark | One chart of time vs peak torque vs energy for each trajectory style |
| O7 Presentation | One uninterrupted demo run |
| Validation | Applied push matches estimated force within tolerance |

## 10. Open items to confirm
- Isaac Sim version that supports the 5090 and runs on the 4070.
- Whether the ROS 2 bridge is used on Windows, or a socket path replaces it.
- Who owns the report writing (suggested: B writes, A reviews).
