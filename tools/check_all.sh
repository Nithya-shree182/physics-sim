#!/usr/bin/env bash
# Re-run every host-side check on the outputs that already exist. No simulator needed.
#
#   tools/check_all.sh            # unit tests, then every run in data/ out/o5 out/o4 out/extra out/o6 out/push,
#                                 # then the payload and push scores
#
# Exit code 0 only if all of them pass. The push check prints a STRICT and an AMENDED verdict; the
# STRICT one is expected to FAIL by one run (docs/VALIDATION.md), so it is reported but only the
# AMENDED verdict and the friction-OFF control decide this script's exit code.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="$ROOT/.venv-tools/bin/python"
cd "$ROOT"
rc=0

echo "== unit tests"
"$PY" -m unittest discover tests 2>&1 | tail -3 || rc=1

for d in data out/o5 out/o4 out/extra out/o6 out/push; do
  n=0; bad=0
  for f in "$d"/*.parquet; do
    [ -e "$f" ] || continue
    n=$((n + 1))
    "$PY" tools/check_run.py "$f" >/tmp/check_run_$$.txt 2>&1 || { bad=$((bad + 1)); echo "FAILED: $f"; grep -E "FAIL" /tmp/check_run_$$.txt | cut -c1-200; }
  done
  echo "== $d: $n runs, $bad failed"
  [ "$bad" -eq 0 ] || rc=1
done
rm -f /tmp/check_run_$$.txt

echo "== payload data (O4)"
"$PY" tools/check_payload.py out/o4 | tail -2 || rc=1

echo "== validation push"
"$PY" tools/check_push.py out/push/*.parquet | grep -E "== |runs \||STRICT|AMENDED" || true
"$PY" tools/check_push.py out/push/*.parquet >/dev/null || rc=1

echo; [ "$rc" -eq 0 ] && echo "ALL CHECKS PASS" || echo "SOME CHECKS FAILED"
exit $rc
