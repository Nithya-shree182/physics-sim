# Reading the telemetry

Written for: anyone who has a run in `out/` or `data/` and wants to know what is in it, or who is
about to add a channel. The machine-readable contract is [`schema/telemetry_v1.yaml`](../schema/telemetry_v1.yaml)
and the validator is [`schema/validate.py`](../schema/validate.py); this page is the why.

```bash
.venv-tools/bin/python schema/validate.py out/<run>.parquet     # check a run against the schema
.venv-tools/bin/python tools/npz_to_parquet.py out/<run>.npz    # convert a capture
```

## The shape of it

One row per physics step — 1000 Hz for a recorded run, 250 Hz live — written as a compressed
`.npz` with a JSON manifest, then converted to Parquet on the host. The two-step exists only
because the Isaac Sim container ships neither pandas nor pyarrow.

205 columns at six joints. The per-joint ones are `name_0` .. `name_5` in the robot spec's joint
order, which is asserted against the simulator at start-up (`pal_plant._check_joint_order`) because
a silent permutation would put one joint's torque under another's name and poison every residual.

## The groups, and why each exists

### State, and the one that used to be missing

`q`, `qd`, `qdd`, plus the `q_cmd` / `qd_cmd` / `qdd_cmd` the trajectory asked for.

**`qdd` is logged.** Schema v0 did not log it and left the analysis side to difference `qd`. That
is worse in a way that is easy to miss: joint acceleration is the dominant noise source in the
residual, and differencing a *logged* velocity amplifies whatever the sensor model added to it by
1/dt. Differencing in-sim, before anything touches it, does not. Measured on a noisy synthetic run,
the residual floor is less than half what differencing gives
(`tests/test_residual.py::test_the_logged_acceleration_beats_differencing_a_noisy_velocity`).
`compute_residual(..., accel="difference")` still forces the old path, which is how that comparison
is made and how a v0 log is still readable.

### Two torques that are not interchangeable

`tau_applied` is what the controller commanded. `tau_measured` is the net joint torque PhysX
reports.

The net torque obeys Newton's law whatever produced it, so **friction, backlash and a locked joint
cannot appear in it** — measured at about 0.001 N·m with friction multiplied by eight
(`docs/REVIEW.md` F2). The applied torque shows all three, but exists only in effort mode: under a
position drive it is identically zero. So fault detection reads `applied`, while payload and wrench
estimation read `measured`, because an external load *is* in the net torque and friction is not.
`observers/residual.fault_torque()` makes that choice from the run's drive mode rather than leaving
it to a caller.

### The decomposition

`g`, `coriolis`, `tau_inertial` (= M·q̈), `tau_friction`, `tau_unexplained`. They sum to
`tau_applied` **exactly**, and `validate.py` checks it to 1e-6 N·m.

That check is the point. The identity is true by construction, so a violation can only mean two
columns in the same row were computed from different state — which is the failure mode a per-step
writer has, and it is invisible in a plot.

What the decomposition buys you: a residual stops being one number. A friction fault moves the
friction term. A payload moves gravity *and* inertial *and* Coriolis together. A stuck joint moves
none of them and lands entirely in what is left over.

`tau_unexplained` is **not** the attestation residual. It is the simulator checked against its own
decomposition — a much weaker statement. The residual is computed on the host against an
independent Pinocchio model, and it is deliberately not a column: the frame carries measurements,
and the residual is a measurement compared with a prediction.

### Energy and power

`E_kin`, `E_pot_rel`, `E_tot_rel`, `P_joint_i`, `P_mech`, `P_grav`, `W_mech`.

- `E_kin = ½ q̇ᵀ M q̇` through the mass matrix, so the armature's share of the energy is in it —
  about 1.8% at 1 rad/s on this arm.
- **`E_pot_rel` is relative to the first row, and only differences are meaningful.** The absolute
  potential needs link centres of mass, which the articulation does not hand back cheaply, so it is
  integrated from the identity `dV/dt = g(q)ᵀ q̇`. That identity is tested against the model's
  absolute potential (`tests/test_energy.py`), and the host-side `RobotModel.potential_energy()`
  will give you the absolute value when you want it.
- `E_tot_rel` is conserved on a frictionless, torque-free swing. That makes it the cheapest check
  there is on the mass matrix and the gravity term *together*: a wrong inertia tensor makes energy
  drift, and nothing else in the suite would notice.
- `P_joint` is positive when the motor drives the arm and negative when the arm drives the motor
  and the joint brakes. A payload the model does not know about shows up in the work budget before
  it shows up anywhere else.

### Headroom and pose quality

`tau_frac_i = |tau_measured_i| / tau_max_i`, `tau_frac_max`, then `sigma_min`, `sigma_max`,
`cond_J`, `manip`, `cond_M`.

The conditioning scalars come from the Jacobian's **linear block only**. The full 6×n Jacobian
mixes m/s rows with rad/s rows, so its singular values have no single unit and its condition number
is not a physical quantity — taking the SVD of the whole thing is the obvious wrong move.

A trap worth knowing, and it is pinned as a test: **the two measures disagree.** At the UR10e's
elbow singularity the Yoshikawa volume `manip` collapses by more than tenfold while `sigma_min`
falls only about fourfold, because J_v is 3×6 and keeps full row rank until the degeneracy is
exact. Reading `sigma_min` alone would call that pose merely mediocre. Anything that gates on pose
quality should use `manip`.

Why any of this is in a telemetry frame: a residual measured near a singularity is not the same
measurement as one taken in the middle of the workspace, and a reader cannot tell the two apart
from the residual alone.

### The grip

`grip_active`, `box_index`, `payload_kg`, `payload_attached`.

`payload_attached` means the run *intends* a load on the tool. `grip_active` means the suction tool
is *actually* holding something. They are separate columns because a commanded grip that failed to
seal is exactly the case worth being able to see, and it has `payload_attached` true with
`grip_active` false.

## The sensor model

**Off by default.** Nothing is added to any channel unless `PAL_SENSOR_NOISE` asks for it, and the
manifest's `sensor_noise` block always says what was injected — so "ideal sensing" and "nobody
recorded whether there was noise" cannot look the same.

```bash
PAL_SENSOR_NOISE=1 ./start_sim_docker.sh      # the reference sigmas in pal_sensors.py
PAL_SENSOR_NOISE=0.5 ./start_sim_docker.sh    # half of them
```

When noise *is* on it touches `q`, `qd` and both torques — what a sensor would corrupt — and never
the simulator's own `g`, `coriolis`, `M` or `J`, which are a cross-check rather than a reading.

Two consequences, both deliberate. With noise off, two runs of the same scenario are bit-identical,
which is what makes an energy or torque channel readable. It also means a false-alarm **rate**
cannot be measured from repeats; that measurement needs the noise switched on.

And the honest part: a real UR has no joint torque sensor at all — its reported torque is derived
from motor current. Clean joint torque sensing is a capability the real arm does not have. The
sensing model is a study parameter, not a fidelity claim.

## Decimation

A full frame costs about 1.29 ms to read and M and J are most of it, while both change very little
in a millisecond. `mj_every=N` reads them every Nth step and holds the last value between reads;
`mj_fresh` marks the rows where they were actually read.

Everything derived from M and J is held with them — the conditioning scalars included. Holding M
but recomputing a J-derived scalar from fresh state would make the row internally inconsistent,
which is tested
(`tests/test_telemetry.py::test_decimation_holds_M_and_everything_derived_from_it`). `E_kin` does
still move between fresh rows, because it uses the held M with the current `qd`, and that is the
point of decimating rather than skipping.

## Timing

Recorded in the manifest as `timing_convention`, and `observers/residual.timing_from_manifest()`
reads it rather than assuming.

- **Isaac runs:** the row is assembled *after* `world.step()`, so every quantity in it is the
  post-step value, `tau_measured[k]` is the effort during the step that produced `q[k]`, and
  `qdd[k] = (qd[k] − qd[k−1])/dt`. Step 0 has no predecessor, so its `qdd` is zero rather than NaN
  — a NaN there propagates through every downstream sum.
- **Synthetic runs:** `tau` at row k is applied during step k and moves `qd[k]` to `qd[k+1]`, and
  `qdd` is the acceleration the integrator actually used for that step.

Either way the acceleration in a row belongs to the torque in that row, which is all the residual
needs. [MEASURED] evaluating the model against pre-step, post-step and midpoint state gives
residuals of 27.99 / 27.98 / 27.98 percent, so the choice is not currently measurable — which is
why it is recorded rather than argued about.

## Adding a channel

1. Add the field to `schema/telemetry_v1.yaml`, with its unit and its source. Per-joint columns say
   `per_joint: true` rather than a literal range, so the schema does not carry the joint count.
2. Write it in `workspace/pal_telemetry.py` and, if a synthetic log should have it too,
   in `model/synth_log.py`.
3. If it stands in a fixed relation to another column, **add that relation to `validate.py`**. The
   existing identity checks are the only thing standing between the project and two columns in one
   row disagreeing about the state they came from.
4. `tests/test_telemetry.py` cross-checks the writer's column set against the schema in both
   directions, so a field added in one place and not the other fails immediately.
