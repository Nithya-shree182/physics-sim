#!/usr/bin/env bash
# Run suites of simulator tests, then convert and check every run they wrote.
#
#   tools/pal_suite.sh o5                    # one suite, headless
#   tools/pal_suite.sh o3,o4 --window        # several, drawn in an Isaac Sim window (slower: rendering)
#   tools/pal_suite.sh all --window          # every objective's tests, the arm moving on screen
#
# Suites: reference o3 o4 o5 clean fast extra push o6 pick o6pick (workspace/pal_suites.py). Steps: tools/run_sim.sh (watchdogged Isaac run) ->
# npz to Parquet on the host -> tools/check_run.py on every Parquet of the suites that ran.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SUITES="${1:-all}"; WINDOW=0; [ "${2:-}" = "--window" ] && WINDOW=1
PY="$ROOT/.venv-tools/bin/python"
[ -x "$PY" ] || { echo "missing $PY (host venv with pandas, pyarrow, pyyaml)"; exit 1; }
export PAL_GIT_COMMIT="${PAL_GIT_COMMIT:-$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo uncommitted)}"
export PAL_SUITE="$SUITES"
export PAL_RUN_TIMEOUT="${PAL_RUN_TIMEOUT:-7200}"
if [ "$WINDOW" = 1 ]; then export HEADLESS=0; fi

declare -A DIR=([reference]=data [o3]=out/o3 [o4]=out/o4 [o5]=out/o5 [clean]=out/clean [fast]=out/fast [extra]=out/extra [push]=out/push [o6]=out/o6 [pick]=out/pick [o6pick]=out/o6pick [pickphys]=out/pickphys [pickfar]=out/pickfar)
if [ "$SUITES" = "all" ]; then LIST=(reference o3 o4 o5 clean fast extra push o6 pick o6pick pickphys); else IFS=',' read -ra LIST <<< "$SUITES"; fi
# The container runs as uid 1234: it must be able to write the directories AND overwrite files the host user created (e.g. a patched manifest).
for s in "${LIST[@]}"; do
  mkdir -p "$ROOT/${DIR[$s]}"; chmod 777 "$ROOT/${DIR[$s]}" 2>/dev/null || true
  chmod a+rw "$ROOT/${DIR[$s]}"/* 2>/dev/null || true
done

# The gripper suites need a different plant (asset variant, finger drives, a physical cube), so they run in their own process.
PLAIN=(); GRIP=()
for s in "${LIST[@]}"; do if [ "$s" = "pickphys" ] || [ "$s" = "pickfar" ]; then GRIP+=("$s"); else PLAIN+=("$s"); fi; done
run_group() {
  PAL_SUITE="$(IFS=,; echo "$*")" "$ROOT/tools/run_sim.sh" pal_run_suite.py
  # run_sim.sh reports success when the container exits, even if the script inside crashed: look for the traceback ourselves.
  if grep -q "py stderr.*Traceback\|py stderr.*Error:" "$ROOT/logs/pal_run_suite.log"; then
    echo "the simulator script crashed; first lines of the error:"
    grep "py stderr" "$ROOT/logs/pal_run_suite.log" | grep -v Warning | head -14 | cut -c1-200
    exit 1
  fi
}
[ ${#PLAIN[@]} -gt 0 ] && run_group "${PLAIN[@]}"
[ ${#GRIP[@]} -gt 0 ] && run_group "${GRIP[@]}"
fail=0
for s in "${LIST[@]}"; do
  d="${DIR[$s]}"
  "$PY" "$ROOT/tools/npz_to_parquet.py" "$ROOT/$d"/*.npz >/dev/null
  n=0; bad=0
  for f in "$ROOT/$d"/*.parquet; do
    n=$((n + 1))
    "$PY" "$ROOT/tools/check_run.py" "$f" >/tmp/check_$$.txt 2>&1 || { bad=$((bad + 1)); echo "FAILED $f"; grep FAIL /tmp/check_$$.txt | cut -c1-180; }
  done
  rm -f /tmp/check_$$.txt
  echo "== $s ($d): $n runs, $bad failed checks"
  [ "$bad" -eq 0 ] || fail=1
done
exit $fail
