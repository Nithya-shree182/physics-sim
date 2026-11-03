# Possible references, assessed

Candidate repositories and literature, with what each is actually worth to this project.
Every repo below was checked to exist on 2026-11-03; star counts and last-push dates are
from the GitHub API that day.

---

## The candidate repositories

| No. | Repository | Objectives | Scale | Verdict |
|---|---|---|---|---|
| 1 | [isaac-sim/IsaacLab](https://github.com/isaac-sim/IsaacLab) | O1, O2, O3, O7 | 8558 stars, active | **Already mined.** Worth one more pass for OSC. |
| 2 | [rezahendi/ros2-franka-torque-control](https://github.com/rezahendi/ros2-franka-torque-control) | O1, O2, O4, O5, O6 | 1 star, Sep 2026 | **Most useful of the eight.** Direct template for O5. |
| 3 | [TrigunAI/franka-peg-in-hole](https://github.com/TrigunAI-Innovations-Private-Limited/franka-peg-in-hole) | O2, O5, O7 | 2 stars, May 2026 | Contact-rich tasks; out of scope for now. |
| 4 | [hucebot/multipanda_ros2](https://github.com/hucebot/multipanda_ros2) | O1, O2, O5, O7 | 5 stars, Oct 2026 | Real-hardware torque control; useful only at sim-to-real. |
| 5 | [zainali3174/panda-compliant-manipulation](https://github.com/zainali3174/panda-compliant-manipulation) | O2, O5, O6 | 0 stars, Jun 2026 | Impedance control; we already have libfranka's gains. |
| 6 | [fida-121/panda_impedance_control](https://github.com/fida-121/panda_impedance_control) | O2, O5, O6 | 2 stars, Jun 2026 | Overlaps 5. |
| 7 | [franklinselva/isaac-sim-franka](https://github.com/franklinselva/isaac-sim-franka) | O1, O7 | 1 star, Jul 2024 | Predates Isaac Sim 5/6; our scene is further along. |
| 8 | [raphaeldeimel/panda_mti](https://github.com/raphaeldeimel/panda_mti) | O1, O2, O5 | 10 stars, Aug 2020 | No published values, but one good lead. |

**A caution on scale.** Seven of the eight have 0–10 stars. That does not make them wrong,
but this project has already had two conclusions overturned by trusting an unverified
source, so nothing from these should enter the model or the config without being measured
against our own plant first. Treat them as **design references, not parameter sources**.

### No. 2 — the one to actually read

A MuJoCo ROS 2 project, simulation only, that implements almost exactly the O5 injector set
we have not built yet, all switchable:

- viscous **and** Coulomb friction in the plant (`friction_torque`)
- link mass scaling for deliberate model mismatch (`link_mass_scale: 1.15` — the same 1.15
  we independently chose for our friction scale)
- actuation delay (`actuation_delay_s: 0.003`)
- payload variation (`payload_kg`)
- sensor noise on position and velocity
- **computed torque (feedback linearisation) against PD-plus-gravity**, with an offline
  sweep over every effect and a results table

Two things to take from it. First, `scripts/offline_experiment.py` is the structure our O5
dataset generation should have — sweep every effect for both controllers, write a results
table. Second, it runs **physics at 1 kHz and the controller at 500 Hz**, which is a
decoupled-rate design we have not tried and which is directly relevant to the above-knee
artifact.

**Tested 2026-11-03 (`workspace/pal_rate_decouple.py`): decoupling the control rate does
not help the above-knee artifact.** 2 kHz physics with a 2 kHz / 1 kHz / 500 Hz controller
gives 35.26 / 35.31 / 35.45% worst-joint residual at full speed; at 1 kHz physics, 27.79%
vs 27.93%. The artifact follows the physics rate only. Not worth pursuing further.

What it cannot give us: it is MuJoCo, so none of the code transfers, and its parameter
values are README examples rather than identified constants.

### No. 8 — one lead, no values

No numeric parameters are published; friction is calibrated per robot by hand against real
hardware. The useful part is the citation of an **IROS'19 system identification paper** for
its dynamics parameters, which is worth chasing as a cross-check on Gaz et al.

---

## The gap: the standard method for this exact problem is missing

The list has no reference to the **generalized momentum observer**, which is the
established technique for residual-based collision and payload detection on manipulators —
precisely what O2, O4 and O5 are.

Instead of our formulation

```
r = tau_measured - ( M(q) qdd + C(q,qd) qd + g(q) )
```

it works from the generalized momentum `p = M(q) qd` and forms

```
r = K_o ( p - integral( tau - n(q,qd) + r ) dt - p(0) )
```

**It never computes `qdd`, and it never inverts `M`.** The residual behaves like a
low-pass-filtered copy of the true external joint torque, with the observer gain `K_o`
trading convergence speed against noise.

### Why that matters here specifically

- **It removes the finite-difference acceleration entirely.** We currently estimate `qdd`
  by differencing velocity. We measured that this is *not* the dominant error (analytic
  `qdd` gives the same residual), but it is still a term we would rather not carry, and
  section 11 of the proposal lists it as a named risk.
- **It is the method the field actually uses**, so our detection rates would be comparable
  with published ones instead of being a bespoke number.
- **Same lineage as parameters we already use.** De Luca is a co-author of the Gaz et al.
  2019 identification we take our friction values from — same group, same robot.
- **Known limitations are documented**: hard to extend to multiple simultaneous contacts,
  and unreliable for contacts on the first links. Worth stating in our section 9 rather
  than discovering.

### Worth reading, in order

1. De Luca & Mattone (2005), momentum residual — `pdot = tau + tau_ext - alpha(q,qd)`
2. [Gaz et al., Mechatronics 2018](https://iris.uniroma1.it/bitstream/11573/1116211/2/Gaz_Preprint-Mechatronics_2018.pdf) — sensorless collision detection and hybrid force/motion control **on the Panda**, same group as our friction parameters
3. [Galbally & Jorda, arXiv 1802.00546](https://arxiv.org/pdf/1802.00546) — real-time collision detection and identification; states the method's limits plainly
4. [Iskandar et al. (DLR)](https://iris.uniroma1.it/retrieve/e383532d-e779-15e8-e053-a505fe0a3de9/Iskandar_preprint_Collision-Detection_2021.pdf) — notes it also avoids inverting the inertia matrix
5. Zurlo, Heitmann, Morlock & De Luca, ICRA 2023 — momentum **and** energy residuals combined, with a particle filter for contact location

---

## What to do with this

**Now, cheap and high value**
- Read No. 2's `offline_experiment.py` as the template for O5 dataset generation.
- Try its decoupled rate split (physics 1 kHz, controller 500 Hz) against our above-knee
  artifact — our controller currently runs at the physics rate.

**Next, a real decision for both sides**
- **Should the analysis side implement a momentum observer alongside the current residual?**
  It is a different estimator of the same quantity, needs no `qdd`, and would let us report
  detection rates comparable with the literature. It is their code, so it is their call,
  but the case is strong enough to put in the next notes.

**Do not do**
- Take friction, mass or gain values from repos 3–8 into `pal_config.py`. Every parameter
  in that file carries a source tag for a reason, and a 0-star repo's untraceable constant
  would be the weakest link in the model.
