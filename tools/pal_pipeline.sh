#!/usr/bin/env bash
# One command: run a sim script, convert its runs to Parquet, validate and sanity-check each.
#
#   (superseded for the suites by tools/pal_suite.sh; this stays for running any other script that writes .npz runs)
#   tools/pal_pipeline.sh some_script.py out/some_dir   # run, convert and check its runs
#
# Steps: tools/run_sim.sh (watchdogged Isaac run) -> tools/npz_to_parquet.py (host) ->
# tools/check_run.py on every Parquet produced (schema, URDF gravity, identity).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT="${1:?usage: pal_pipeline.sh <sim script> <output dir>}"
OUT="${2:?usage: pal_pipeline.sh <sim script> <output dir>}"
PY="$ROOT/.venv-tools/bin/python"
export PAL_GIT_COMMIT="${PAL_GIT_COMMIT:-$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo uncommitted)}"
[ -x "$PY" ] || { echo "missing $PY (host venv with pandas, pyarrow, pyyaml)"; exit 1; }

"$ROOT/tools/run_sim.sh" "$SCRIPT"
"$PY" "$ROOT/tools/npz_to_parquet.py" "$ROOT/$OUT"/*.npz
fail=0
for f in "$ROOT/$OUT"/*.parquet; do
  echo "== $(basename "$f")"
  "$PY" "$ROOT/tools/check_run.py" "$f" | sed 's/^/   /' || fail=1
done
exit $fail
