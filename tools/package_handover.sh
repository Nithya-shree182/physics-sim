#!/usr/bin/env bash
# Bundle every run for the analysis side into one tarball outside git (handover/, git-ignored).
#
#   tools/package_handover.sh            # -> handover/pal_runs_<commit>.tar.gz and a MANIFEST.txt inside it
#
# Contents: data/ref_matched_*, out/o5, out/o4, out/extra, out/push, out/o6 (Parquet + JSON each),
# plus a MANIFEST.txt listing every file with its size and sha256 so the receiver can verify the copy.
# Parquet in git history is permanent, which is why bulk data does not go through git.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
COMMIT="$(git rev-parse --short HEAD)"
OUT="handover"; mkdir -p "$OUT"
STAGE="$(mktemp -d)"; trap 'rm -rf "$STAGE"' EXIT
mkdir -p "$STAGE/pal_runs"
for d in data out/o5 out/o4 out/extra out/push out/o6; do
  [ -d "$d" ] || continue
  mkdir -p "$STAGE/pal_runs/$d"
  cp "$d"/*.parquet "$d"/*.json "$STAGE/pal_runs/$d/" 2>/dev/null || true
done
( cd "$STAGE/pal_runs" && find . -type f ! -name MANIFEST.txt | sort | xargs sha256sum > MANIFEST.txt
  echo "built from dev commit $COMMIT on $(date -u +%FT%TZ)" >> MANIFEST.txt )
tar -C "$STAGE" -czf "$OUT/pal_runs_${COMMIT}.tar.gz" pal_runs
echo "wrote $OUT/pal_runs_${COMMIT}.tar.gz  ($(du -h "$OUT/pal_runs_${COMMIT}.tar.gz" | cut -f1), $(grep -c parquet "$STAGE/pal_runs/MANIFEST.txt") Parquet files)"
