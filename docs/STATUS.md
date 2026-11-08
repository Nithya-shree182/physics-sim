# Project status (final pass, UR10e)

Written for: the next operator or reviewer. This is the live status sheet for the current UR10e port and the
state of the evidence behind it. This file complements the source-of-truth instructions in
`docs/TODO.md` and the implementation details in `DECISIONS.md`.

## Final status

**Model-side validation is green.** The UR10e reference pipeline and the current Pinocchio analysis environment are
passing their checks in the intended project environments.

Verified in this pass:

- `.venv-pal/bin/python -m pytest -q` passes
- `.venv-tools/bin/python tools/check_run.py data/ref_matched_0kg_1000hz.parquet` passes
- clean reference residual checks pass for the active UR10e reference data

## What was fixed in this pass

1. Replaced the stale Panda suite assumptions with the active UR10e suite contract.
2. Restored the missing `o3` suite and fixed the suite registry so the runner accepts the active set.
3. Cleared stale `out/*` and `data/*` artifacts before each run so old Panda-era files are not mixed with fresh UR10e data.
4. Removed stale hard-coded Panda assumptions in the host-side validator (`tools/check_run.py`), including the old Franka torque-limit layout and the old gripper-column coupling.
5. Confirmed the fresh UR10e reference Parquet passes schema, gravity, identity and friction checks.

## Current evidence and confidence

The repo is now in a state where we can say the following with evidence:

- the active UR10e configuration matches the current URDF and model path
- the clean synthetic / reference check passes at the model and telemetry level
- the project is no longer blocked by stale Panda-era data in the validation path

The remaining work is not a repo bug; it is the continuation of the validation into the full real-arm sweep:

- residual floor and speed band on the UR10e
- payload estimation on fresh run data
- fault detection on fresh run data
- final end-to-end live-cell and dashboard verification

## Checklist

- [x] UR10e port state established
- [x] stale Panda assumptions removed from validation path
- [x] fresh UR10e reference run validates
- [x] repo-level model tests pass in the project Pinocchio environment
- [ ] full suite re-measurement on the UR10e finished
- [ ] payload detection on real arm data validated
- [ ] fault detection on real arm data validated
- [ ] live Isaac dashboard end-to-end verification complete

## Known caveats

- The project still needs fresh measured UR10e evidence before claiming strong final numbers for the speed band,
payload estimate or fault detectability in the real robot regime.
- The clean ideal-state residual is valid; real-arm residual and detection numbers remain measurement tasks, not code-fix tasks.
