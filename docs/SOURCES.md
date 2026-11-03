# Reference sources: Franka Panda physics parameters

Where to get realistic parameters (inertia, friction, damping, limits, contact) for the Panda twin. Put every value you adopt into `configs/*.yaml` and note the source here.

## Local copies (`docs/references/`)
| File | What it is |
|---|---|
| `gaz2019_panda_dynamic_identification_supplementary.pdf` | Supplementary material to Gaz et al. 2019: friction fit (Table I), identified dynamic coefficients (Table III), excitation trajectory (Table II) |
| `franka_research3_datasheet_v2.6_2026-07.pdf` | Franka Research 3 datasheet v2.6 (payload, limits, 1 kHz control interface). FR3, not the original Panda; use the matching sheet for the model you load |

Copyright note: these PDFs belong to their authors and Franka. Check the licence before keeping them in a public repo. If in doubt, delete them and keep only the links below.

## Online sources
| Need | Source |
|---|---|
| Real-Panda inertial parameters, friction model | [Gaz et al. 2019, IEEE RA-L (Sapienza postprint)](https://iris.uniroma1.it/bitstream/11573/1356777/2/Gaz_Postprint_Dynamic-Identification_2019.pdf) |
| Friction values, identified coefficients | [Gaz et al. supplementary material](https://panda-safe-hrc.readthedocs.io/en/latest/_downloads/af8082ee8504cc070ab037e7f5ef0641/PandaDynamicIdentification_SupplementaryMaterial.pdf) |
| Panda inertials cross-check (public URDF) | [RoboVerse MJCF README](https://huggingface.co/datasets/RoboVerseOrg/roboverse_data/blob/refs%2Fpr%2F10/robots/franka/mjcf/README.md) |
| Datasheet limits (payload, velocity, torque sensing, 1 kHz) | [Franka Research 3 datasheet v2.6](https://franka.de/hubfs/Datasheet%20Franka%20Research%203_R02212_2.6_EN_July%202026.pdf?hsLang=en), [Panda Research datasheet](https://centers.ulbsibiu.ro/incon/wp-content/uploads/Datasheet-Franka-Emika-Panda-Research-collaborative-robot.pdf) |
| Isaac Sim friction/damping fitting procedure | [Isaac Sim tutorial: system identification of a Franka](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/robot_setup_tutorials/tutorial_franka_system_identification.html) |
| Drive gains (position, velocity, effort modes) | [Isaac Sim 4.2 Core API docs](https://docs.isaacsim.omniverse.nvidia.com/4.2.0/py/source/extensions/omni.isaac.core/docs/index.html) |
| Armature, actuator setup | [Isaac Sim actuators tutorial](https://docs.isaacsim.omniverse.nvidia.com/latest/newton_actuators_tutorials/newton_actuators_python.html) |
| PhysX articulation joint friction model | [PhysX joint friction paper](https://nvidia-omniverse.github.io/PhysX/physx/5.6.1/_downloads/59254688fd359ea998f192999f5206ec/jointFriction.pdf) |
| Collision contact/rest offsets | [PhysxCollisionAPI schema docs](https://docs.omniverse.nvidia.com/kit/docs/omni_usd_schema_physics/104.2/class_physx_schema_physx_collision_a_p_i.html) |
| Franka model without hardware (libfranka + URDF) | [TechOverflow mass-matrix example](https://techoverflow.net/2025/01/27/how-to-use-libfranka-without-fci-compute-mass-matrix-example/) (uses the FR3 URDF) |

## Friction values from Gaz et al. 2019 (supplementary Table I)
Model per joint (a = amplitude, b = steepness, c = velocity offset): `f(qd) = a / (1 + exp(-b * (qd + c))) - a / (1 + exp(-b * c))`. This form was read from a garbled text extraction, so check it and the values against the PDF before use.

| Joint | 1 | 2 | 3 | 4 | 5 | 6 | 7 | Units |
|---|---|---|---|---|---|---|---|---|
| Amplitude | 0.5462 | 0.8722 | 0.6407 | 1.2794 | 0.8390 | 0.3030 | 0.5649 | N·m |
| Steepness | 5.118 | 9.066 | 10.136 | 5.590 | 8.347 | 17.133 | 10.336 | s/rad |
| Velocity offset | 0.0395 | 0.0259 | -0.0461 | 0.0362 | 0.0262 | -0.0210 | 0.0036 | rad/s |

The form and all 21 values above were cross-checked by dev against the PDF and are correct.

The viscous + Coulomb + offset version, `tau_f = fv*qd + fc*sign(qd) + fo`, is in Table III of the same PDF (classical-identification column):

| Joint | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|
| fv (N·m·s/rad) | 0.0665 | 0.1987 | 0.0399 | 0.2257 | 0.1023 | −0.0132 | 0.0638 |
| fc (N·m) | 0.2450 | 0.1523 | 0.1827 | 0.3591 | 0.2669 | 0.1658 | 0.2109 |
| fo (N·m) | −0.1073 | −0.1566 | −0.0686 | −0.2522 | 0.0045 | 0.0910 | −0.0127 |

Extraction pitfall: `pdftotext -layout` misaligns this table's row labels by one row. `pdftotext -raw` and `-table` keep each label with its own numbers, and give the values above, which match dev's `pal_config.py`. `fv,6` (std 12.26), `fo,5` (std 19.46) and `fo,7` (std 6.26) are not meaningful; dev clamps `fv,6` to zero.

## Caveats
- The manufacturer publishes no dynamic parameters for the Panda. Gaz et al. is one robot's identification from 2019, so treat it as a realistic starting point, not ground truth.
- The Isaac Sim tutorial is a method, not a set of values. It states its result is not a measurement of a physical Franka.
- Isaac Sim 6.x docs use the Newton/MuJoCo path. Confirm which physics engine your version uses, since friction and armature settings differ.
- Joint torque limits are not extracted here. Read them from the datasheet table directly.
- No authoritative source found for real-arm collision stiffness or damping. PhysX docs describe simulated contact only.
