# The residual floor: two regimes, and where the measurement stops being physical

The single most important result for O2, O4 and O5. Supersedes every earlier residual
figure in this repo, including two conclusions that turned out to be wrong.

Measured with `workspace/pal_sweep.py` — one physics rate per process, identical settle,
warm-up and measurement windows in *simulated* seconds, trajectory amplitude a function of
speed only. Plant authored from the URDF, no friction, `position_stiff` (400 N·m/rad).

---

## The table

Worst-joint RMS residual as a fraction of that joint's torque limit:

| speed | 250 Hz | 500 Hz | 1000 Hz | 2000 Hz | rate-dependent? |
|---|---|---|---|---|---|
| 10% | 0.43% | 0.43% | 0.43% | 0.43% | no (×1.00) |
| 20% | 0.85% | 0.86% | 0.86% | 0.86% | no (×1.00) |
| 30% | 1.26% | 1.27% | 1.27% | 1.27% | no (×1.00) |
| 50% | **2.05%** | **2.05%** | **2.05%** | **2.05%** | no (×1.00) |
| 75% | 3.08% | 4.29% | 8.79% | 17.96% | **YES (×5.8)** |
| 100% | 7.26% | 14.85% | 30.21% | 53.76% | **YES (×7.4)** |

Speed is a fraction of each joint's datasheet velocity limit.

## Two regimes

**Below ~50% — physical.** The residual is identical to two decimal places across an 8×
range of physics rate, and grows linearly with speed. A quantity that does not move when
the integrator's timestep changes by 8× is a property of the arm, not of the simulation.
This is the real floor, and no choice of rate will reduce it.

**Above ~50% — artifact.** The residual roughly doubles for each doubling of the physics
rate. At 2000 Hz and full speed the *peak* residual reaches 112% of a joint's entire torque
limit, which no physical reading can be.

The boundary is sharp. Between 50% and 75% the rate-dependence goes from ×1.00 to ×5.8.

## Why the artifact appears where it does

Below the knee the arm tracks its commanded trajectory well and the position drive is
gentle. Above it tracking degrades badly — measured at roughly 54% of commanded velocity —
so the drive works hard, and a rate-dependent term appears in the reported torque.

That is the signature of an **implicit position drive**, which is exactly the hypothesis
the analysis side raised (their R2.3): the `tau_measured` of a stiff implicit drive may not
be the torque that actually moved the arm, and the discrepancy grows with how hard the
drive is working and with how often it is solved.

It is not any of these, each ruled out by measurement:

| hypothesis | test | result |
|---|---|---|
| finite-difference noise in qdd | analytic qdd from the trajectory | same residual (29.16% vs 27.79%) |
| low-pass filtering would fix it | filter the qdd estimate | changes it by 2% |
| timing misalignment | model at pre-step / post-step / midpoint state | 27.99 / 27.98 / 27.98% |
| Coriolis sign or definition | try every model form | assumed form wins; C is exactly 0 at rest |
| solver non-convergence | 4 vs 128 position iterations | 14.24% vs 14.22% |
| armature missing from M | armature on vs off | 2.3692 vs 2.3892 |
| velocity-limit saturation | sweep never reaches the limit | not saturated at any point |
| mass-matrix slicing | full 9×9 vs 7×7 block | identical |

## What it means for the objectives

### O4 payload estimation

A payload contributes 4.629 N·m/kg at joint 4 (measured, and matching the analysis side's
independent 4.60). The floor converts directly into a detection limit:

| speed | floor at j4 | smallest detectable | 3-sigma threshold |
|---|---|---|---|
| 10% | ~0.13 N·m | 0.03 kg | 0.09 kg |
| 30% | ~0.41 | 0.09 | 0.27 |
| 50% | ~0.69 | 0.15 | 0.45 |
| 100% (1 kHz) | ~3.70 | 0.80 | 2.40 |

Below the knee the whole 0.1–3.0 kg ladder is detectable. At full speed most of it is not.

Confirmed in practice: an unknown 2 kg object was recovered to **0.01–0.03%** at three
poses with the arm at rest.

### O5 detection

Thresholds must sit above the floor at the speed the data was recorded at, and the speed
must be reported with every detection rate. A rate quoted without its speed is meaningless.

### O6 benchmarking

O6 wants fast motion to compare trajectory styles; O2/O4/O5 need to stay below the knee.
**They cannot share a speed regime.** O6 should report cycle time, peak torque and energy
at whatever speed it likes, while any residual-based claim stays under 50%, and the results
must say which regime each number came from.

## Two earlier conclusions that were wrong

Both are corrected here, and both are instructive.

**"Time discretization, first order in dt."** Derived from a rate sweep that rebuilt the
`World` once per rate inside a single process. Re-run with one container per rate, the
trend **inverts**. Repeatedly constructing a World and re-referencing the same prim path in
one Isaac process does not give a clean plant.

**Cross-script comparisons that crossed over.** `pal_fast_random` at 1000 Hz gave 1.81% at
half speed while `pal_experiment` at 500 Hz gave 6.47%, yet the rate sweep at full speed
said 500 Hz beat 1000 Hz. Both were sampling different regimes with different warm-up and
window lengths, so neither was comparable to the other. Pinning the methodology — duration
in simulated seconds, identical settle and warm-up, amplitude independent of rate — removed
the contradiction entirely.

The lesson for anything measured from here: **vary one thing per process, and fix the
measurement window in simulated time rather than in steps.**

## Still open

The deciding test for the artifact is the analysis side's R2.3: run the same trajectory in
**effort mode**, commanding `M q̈ + C q̇ + g` as feedforward with a light PD. If the
above-knee residual collapses, it belongs to the position drive and O2/O5 data should be
recorded in effort mode. If it survives, it is a property of the plant.

Until then the 50% envelope stands on its own evidence: below it the measurement does not
depend on a simulator setting, which is the only definition of "physical" available here.
