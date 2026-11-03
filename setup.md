# Setup: continuing the analysis side on another machine

Written for: Nithya (Person B), picking the work up on a different laptop, and for any Claude
Code session started there. Written 2026-11-03. Do this once, then read
[context.md](context.md).

## 1. Hardware

**No GPU is needed.** Everything on the analysis side runs on the CPU: the Pinocchio model,
the residual, confidence, payload and detector code, the schema validator, the synthetic log
generator, the tests and the future Dash dashboard. A laptop with an RTX 3050 (or no GPU at
all) is fine. Isaac Sim needs an RTX card, but it only runs on dev's machine; this side
reads the logs it produces. Windows, Linux and macOS all work; commands below are for
Windows PowerShell.

## 2. Install (about 15 minutes)

1. **Git** and **Miniconda** (https://docs.conda.io/en/latest/miniconda.html).
2. Get the code:
   ```
   git clone https://github.com/Nithya-shree182/physics-sim.git
   cd physics-sim
   git checkout nithya
   git pull
   ```
   (Already cloned? Just `git checkout nithya` and `git pull`.)
3. Create the Python environment (pinocchio comes from conda-forge; `pip install pin` fails
   on Windows because it tries to build Boost):
   ```
   conda env create -f environment.yml
   conda activate pal
   ```
4. **Set your identity for this repo**, so commits are not made under another name:
   ```
   git config user.name "Nithya-shree182"
   git config user.email "nithyashreer23cy@rnsit.ac.in"
   git config user.name        # check it printed back
   ```
   (Without `--global`, this affects only this repo.)

**If `conda activate pal` does nothing** in PowerShell, run `conda init powershell` once and
open a new terminal. The prompt must start with `(pal)`. Check with:
```
python -c "import sys; print(sys.executable)"      # must end in envs\pal\python.exe
```
Always run inside the activated env: calling the env's `python.exe` directly, without
activating it, can load an incompatible LAPACK DLL from elsewhere on the PATH (for example
MATLAB) and crash with Windows code `0xC06D007F`.

## 3. Check that it works

```
python -m pytest                       # 30 passed, 5 skipped (the 5 need data/real/, section 4)
python model/gravity_check.py          # 10 PASS
```

## 4. Restore the files that are not in git

`.gitignore` keeps `data/` out of the `nithya` branch, so these have to be recreated or copied.

| What | How | Needed? |
|---|---|---|
| **Real logs from dev** (2 Parquet + 2 JSON, about 7 MB) | see below | **Yes**, for real-data work and 5 of the tests |
| Synthetic logs (`data/synthetic/`, about 270 MB) | `python model/synth_log.py --batch` (about 25 s) | For observer and detector work |
| Confidence sweep (`data/analysis/`) | `python observers/confidence.py --sweep 5000` | Optional |
| Proposal PDF | copy by USB/cloud from the old laptop (`Physics Attestation Layer A Physics-Based Digital Twin for Robot Arm Monitoring.pdf`, repo root; never commit it) | Optional reading |
| `ARM_CONFIG.md` | already on dev's branch: `git show origin/dev:ARM_CONFIG.md` | Optional; never commit it on `nithya` |
| Claude Code memory | **not transferred** (it is stored per machine). `context.md`, `changes.md` and `docs/DASHBOARD_PLAN.md` hold the same facts | Use section 7 instead |
| Dashboard design canvases | claude.ai links in `docs/DASHBOARD_PLAN.md`; open from the same account | When the dashboard work starts |

**Real logs.** Dev tracks them in git on the `dev` branch (`data/slow_reference_0kg_1000hz.*`
and `data/slow_reference_1p0kg_1000hz.*`). Fetch them without touching your branch or staging
anything:
```
git fetch origin
git worktree add ..\physics-sim-dev origin/dev
mkdir data\real
copy ..\physics-sim-dev\data\slow_reference_* data\real\
git worktree remove ..\physics-sim-dev
```
Check them (both should print `VALID`), then rerun the tests (35 pass with them):
```
python schema/validate.py data/real/slow_reference_0kg_1000hz.parquet
python schema/validate.py data/real/slow_reference_1p0kg_1000hz.parquet
python -m pytest
```
Dev may add more runs (fast, fault) later: look in `data/` on `origin/dev`. The fallback is
copying `data/real/` from the old laptop; the files are small.

## 5. Where we stopped (2026-11-03, late evening)

- **Your branch:** `nithya` is pushed and clean. The latest work commit is `8f947d8` (real-data
  tests, first analysis of dev's logs, data requests to dev); this setup commit follows it.
  Nothing is waiting uncommitted except files that are meant to stay out of git.
- **Waiting on dev:** `docs/NOTES_FOR_DEV.md` now goes up to **Round 4** (answers to dev's
  questions, plus a revised run list in its section C). Dev has not answered it: as of
  `cffda31`, `data/` on `dev` still holds only the two slow-reference logs.
- **Dev's branch:** at `cffda31`. Since your last review it added `docs/NOTES_FOR_NITHYA.md`,
  `docs/RESIDUAL_FLOOR.md`, an effort-mode test (`workspace/pal_effort_test.py`) and the real
  logs in `data/`.
- **First real logs** (two 3 s slow-reference runs, 0 kg and 1 kg, position drive, 1 kHz):
  schema valid; with an empty gripper the identity closes (residual 1 to 2% of a torque limit on
  joints 5 to 7, well under 1% elsewhere); 1 kg adds about 3.0 and 4.6 N·m on joints 2 and 4
  as predicted; the tare from the empty run gives 1.000 kg. No false alarms.

**What dev told us that changes our plans** (read `git show origin/dev:docs/NOTES_FOR_NITHYA.md`
in full: only its sections 1 to 3 were read so far):
- The "fast-motion residual is time discretization, about 2.8% at 1 kHz" claim was **wrong**
  (the rate sweep was invalid). What is true: below about 50% of the velocity limit the
  residual is physical and the same at 250 to 2000 Hz (about 2% of a limit at 50% speed);
  above about 50% an artifact appears that grows with the physics rate.
- Effort mode does not remove that artifact, so the plan is **record at 50% of the velocity
  limit or less, in position mode.**
- Their reply to our round-2 questions is in sections 3 onwards.

## 6. Stale in `context.md`, and one lead not yet written down

`context.md` has not caught up with the above. Until it is updated:
- Ignore its "dev's stated residual floor (about 2.8%, first order in dt)" paragraph. Dev has
  retracted it (section 5).
- Ignore "Waiting on dev" items 1 to 4: items 1 to 3 are answered and item 4 is done.
- "Logs travel outside git (`data/` is ignored)" is only true on `nithya`; dev tracks `data/`.

**A lead from the first real-log look, not recorded elsewhere (unverified).** At rest, the
simulator's own mass matrix is slightly lower than ours: diagonal J7 is 0.0072 against our
0.0133 kg·m², and that difference, 0.0061, equals the armature exactly. J2 to J6 differ by
0.006 to 0.010 (partly the same effect, partly the diagonal-only inertia dev authors).
Dev believes `get_mass_matrices()` includes armature; this suggests it may not. A quick test:
compare the logged `M_*` columns with our `M` built at `armature=0.0`. It does not hurt the
slow-motion identity (the error is proportional to acceleration), but it matters for fast runs
and for the payload estimate during motion.

## 7. Starting Claude Code on the new machine

Claude's memory does not travel, so begin the first session with something like:

> Read setup.md, context.md, changes.md, docs/NOTES_FOR_DEV.md and docs/DASHBOARD_PLAN.md,
> then dev's docs/NOTES_FOR_NITHYA.md (`git show origin/dev:docs/NOTES_FOR_NITHYA.md`). Commit
> as Nithya, with no Claude co-author and no mention of Claude in commit messages. Never commit
> the proposal PDF, ARM_CONFIG.md, .vscode, __pycache__ or data/; add files by name. Do not
> build the dashboard until I confirm the design.

## 8. Next steps

1. Read dev's `NOTES_FOR_NITHYA.md` in full and update `context.md` (section 6).
2. Check the mass-matrix lead in section 6.
3. Follow up with dev on the run list in **Round 4, section C of `docs/NOTES_FOR_DEV.md`**,
   all at 50% of the velocity limit or less in position mode: excitation runs at 0 kg and
   1 kg (C2), faults from half way with a matching fault-free baseline for each (C4: friction
   on joints 3 and 6, stuck joint 4, 50 ms delay), one fault-free run of 60 s or more (C5),
   then the lower-priority payload ladder, per-pose holds, mid-run pick-up and seeds. It also
   asks dev to log `speed_fraction` in the manifest. Short runs can come through `data/` on
   `dev`; larger ones outside git.
4. Re-tune the detector thresholds on real fault runs. They were set on synthetic data.
5. Classifier, then the dashboard (design confirmed first, dev's HTTP feed in place).

## 9. Working rules

- Commit as **Nithya** (`Nithya-shree182 <nithyashreer23cy@rnsit.ac.in>`). No Claude co-author
  line and no mention of Claude in commit messages.
- Never commit: the proposal PDF, `ARM_CONFIG.md`, `.vscode/`, `__pycache__/`, `data/`. Add
  files by name, never `git add .`.
- Residual is `tau_measured - tau_model`. Armature 0.0061 kg·m² and the Gaz friction values are
  contracts with dev: do not change one side alone.
