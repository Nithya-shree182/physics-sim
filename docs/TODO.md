  # What is left to do

  Written for: whoever picks this up next. Read this first. It supersedes the "What is NOT done"
  section of [UR10E_PORT.md](UR10E_PORT.md), which describes the port itself.

  State as of **2026-11-08, final pass**: the arm is a UR10e, the Panda is gone from the runtime,
  telemetry is at schema v1 with energy and torque-decomposition channels, `pal_live.py` drives the
  conveyor cell, the viewport is isometric, and the project is in a validated UR10e state for the
  relevant model/telemetry checks. The history remains in [../changes.md](../changes.md).

  ---

  ## Completed in this pass

  This round closed the key blockers that were keeping the repo in a stale Panda-era state:

  - The project was re-grounded to the active UR10e configuration rather than the removed Panda robot.
  - The missing `o3` suite and stale suite registry were restored to the active UR10e contract.
  - Fresh generated reference data was validated successfully by the host-side checker.
  - The full validation path now passes in the intended project environments:
    - `.venv-pal/bin/python -m pytest -q` passes
    - the reference Parquet passes schema, gravity, identity and friction checks via
      `.venv-tools/bin/python tools/check_run.py data/ref_matched_0kg_1000hz.parquet`

  The key validation result from the fresh UR10e reference run was:

  - SCHEMA: PASS
  - GRAVITY: PASS
  - IDENTITY: PASS
  - FRICTION: PASS
  - OVERALL: PASS

  This means the current UR10e reference and model pipeline are valid in the ideal clean regime, and the
  remaining work is not a repo-wide logic failure but the extension of that validation into the real
  sweep conditions on the arm (speed band, payloads, and fault detection on fresh capture data).

  ---

  ## 1. Regenerate the remaining UR10e suite sets

  The original blocked item remains the full suite set, but the repo is no longer blocked by stale Panda
  assumptions or a broken suite registry. The data-generation path is now valid and can be rerun.

  Use:

  ```bash
  ./tools/pal_suite.sh all
  ```

  This should be treated as the active proof step for the remaining measurements, especially for:

  - the fresh `o3`, `o4`, `o5`, `clean`, `fast`, `extra`, `push`, `o6`, `pick`, `o6pick`, `pickphys`, and `pickfar` runs
  - re-measuring the residual floor and speed band on the actual UR10e
  - validating payload estimation against fresh arm logs
  - validating fault detection against fresh arm logs

  ## 2. Re-measure the residual floor and speed band on the UR10e

  The project already contains the correct logic for the residual and speed-cap checks; the remaining
  requirement is fresh evidence from the active robot. The code path is valid, but the numbers must be
  re-measured on the UR10e rather than copied from the Panda-era docs.

  This should be treated as the main measurement item before any final detection claims are made.

  ## 3. Validate payload and fault detection on real UR10e data

  The project has a working model-side pipeline and a successful clean ideal-state pass, but the factual
  claim "payload detection works on the current UR10e" still requires fresh UR10e captures and direct
  validation on those logs.

  This remains the final proving step before claiming a real arm result.

  ## 4. Verify the live Isaac Sim + dashboard path end to end

  The dashboard and live server are ported and the stream bug was fixed. The remaining end-to-end check is
  to verify the full live cell cycle from the dashboard button press through the run, the fault onset and
  box placement, including the Isaac-side runtime recovery behavior.

  ## 5. Final housekeeping

  The remaining small cleanup tasks are mechanical and non-blocking:

  - remove stale Panda-era documentation references where they remain visible
  - confirm the live dashboard items align with the current UR10e flow
  - keep the remaining benchmark docs clearly marked as UR10e-only evidence and not Panda-era assumptions

  ## 6. Nothing is committed

  Branch `dev` remains the working branch for this pass. The current UR10e port and documentation updates are
  still in the working tree. This is fine for the current end-of-pass state, but a commit is recommended
  before continuing into a new measurement or detection run set.


  ## 7. Dashboard items (2026-11-08, late)

  Done: item 4 (twin beside the Isaac picture, toggle removed), item 6 (both Replay buttons removed;
  pick style, load and speed sit under the twin), item 7 (Analysis tab with per-box facts and an
  offline rule-based narrator), and the load slider now runs to 8 kg (was 3). Checked on the
  stand-in plant by screenshot; **not yet checked against live Isaac** (GPU was busy with the fault suites).
  Still blocked: item 8 (plain-language explanation of the three styles) until `o6pick` is re-run on the UR10e.
