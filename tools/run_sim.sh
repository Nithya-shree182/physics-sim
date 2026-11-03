#!/usr/bin/env bash
# Run one script inside the Isaac Sim container, with a watchdog that can tell a slow
# boot from a real hang.
#
#   tools/run_sim.sh pal_capture.py [logfile]
#   HEADLESS=0 tools/run_sim.sh pal_experiment.py      # with a window on the host display
#   PAL_STEPS=5000 PAL_PAYLOAD_KG=2 tools/run_sim.sh pal_capture.py logs/cap.log
#
# Why this exists: the previous inline watchdog detected "booted" by counting lines in
# docker logs and gave up after 200 s. Isaac's extension loading alone can exceed that on
# a cold shader cache, so slow boots were reported as hangs — one run logged "attempt 1-4
# hung" when attempt 4 had actually succeeded and finished after the loop abandoned it.
# It also never killed the container it gave up on, so abandoned runs kept using the GPU.
#
# This version instead waits for the SCRIPT's own first output (every pal_*.py prints a
# line containing "[PAL]"), allows a much longer boot, kills whatever it abandons, and
# reports the real boot time so the hang rate can actually be measured.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT="${1:?usage: run_sim.sh <script.py> [logfile]}"
LOG="${2:-$ROOT/logs/$(basename "$SCRIPT" .py).log}"
NAME="pal-$(basename "$SCRIPT" .py | tr '_' '-')"
IMAGE="${ISAAC_IMAGE:-nvcr.io/nvidia/isaac-sim:6.0.1}"
CACHE="${PAL_CACHE:-$ROOT/../Physyk_AI-master/.isaac-cache}"
BOOT_TIMEOUT="${PAL_BOOT_TIMEOUT:-420}"     # seconds to wait for the script's first line
RUN_TIMEOUT="${PAL_RUN_TIMEOUT:-1800}"      # seconds for the script itself
ATTEMPTS="${PAL_ATTEMPTS:-3}"

mkdir -p "$ROOT/logs" "$ROOT/data" "$ROOT/out"
# The container runs as uid 1234 — neither root nor the host user — so host directories at
# the usual 755 are not writable by it, and that fails only at the END of a run.
mkdir -p "$ROOT/out/o5"
chmod 777 "$ROOT/data" "$ROOT/out" "$ROOT/out/o5" 2>/dev/null || true

cleanup() { docker rm -f "$NAME" >/dev/null 2>&1 || true; }
trap 'echo; echo "interrupted, stopping $NAME"; cleanup; exit 130' INT TERM

for attempt in $(seq 1 "$ATTEMPTS"); do
  cleanup
  X11_ARGS=()
  if [ "${HEADLESS:-1}" = "0" ]; then
    xhost +local:docker >/dev/null 2>&1 || true
    X11_ARGS=(-v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY="${DISPLAY:-:0}")
  fi
  docker run -d --name "$NAME" --gpus all --network host \
    -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e OMNI_KIT_ACCEPT_EULA=YES \
    -e NVIDIA_DRIVER_CAPABILITIES=all -e ISAAC_SIM_HEADLESS="${HEADLESS:-1}" \
    "${X11_ARGS[@]}" \
    $(env | grep -E '^PAL_(STEPS|TRAJ|PAYLOAD_KG|PHYSICS_HZ|MJ_EVERY|RUN_ID|DRIVE_MODE|FRAC|SEED|GIT_COMMIT|MACHINE|NO_FRICTION|FRICTION_API|DURATION_S|ONSET_S)=' | sed 's/^/-e /') \
    -v "$CACHE/kit:/isaac-sim/kit/cache:rw" -v "$CACHE/ov:/root/.cache/ov:rw" \
    -v "$ROOT/workspace:/workspace:rw" \
    -v "$ROOT/assets/franka_panda:/workspace/assets:ro" \
    -v "$ROOT/data:/workspace/data:rw" -v "$ROOT/out:/workspace/out:rw" \
    -w /workspace --entrypoint /isaac-sim/python.sh \
    "$IMAGE" "$SCRIPT" >/dev/null || { echo "docker run failed"; exit 1; }

  # Boot watchdog: wait for the SCRIPT's own marker, not for log volume.
  t0=$SECONDS; booted=0
  while (( SECONDS - t0 < BOOT_TIMEOUT )); do
    if docker logs "$NAME" 2>&1 | grep -q '\[PAL\]'; then
      booted=1; break
    fi
    # If it exited before printing the marker, it crashed rather than hung — stop retrying
    # and let the caller read the log.
    if ! docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
      docker logs "$NAME" > "$LOG" 2>&1; cleanup
      echo "attempt $attempt: container exited during boot (crash, not a hang) — see $LOG"
      exit 1
    fi
    sleep 3
  done

  if (( booted )); then
    echo "attempt $attempt: script started after $((SECONDS - t0))s"
    if timeout "$RUN_TIMEOUT" docker wait "$NAME" >/dev/null 2>&1; then
      docker logs "$NAME" > "$LOG" 2>&1; cleanup
      echo "done -> $LOG"
      exit 0
    fi
    docker logs "$NAME" > "$LOG" 2>&1; cleanup
    echo "attempt $attempt: script exceeded PAL_RUN_TIMEOUT=${RUN_TIMEOUT}s"
    exit 2
  fi

  echo "attempt $attempt: no script output in ${BOOT_TIMEOUT}s — genuine boot hang, killing"
  cleanup
done
echo "all $ATTEMPTS attempts hung during boot"
exit 3
