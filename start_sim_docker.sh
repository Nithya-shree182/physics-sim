#!/usr/bin/env bash
# Runs one script inside the official Isaac Sim container, so the host OS does not matter.
# The default is the conveyor-cell demo: a UR10e picking three cardboard boxes off an infeed
# table with a suction tool and setting them on a running belt.
#   ./start_sim_docker.sh                      # windowed cell demo (default)
#   HEADLESS=1 ./start_sim_docker.sh           # headless, no window
#   PAL_BOX_KG=2.5 ./start_sim_docker.sh       # heavier boxes
#   PAL_STYLE=trapezoidal ./start_sim_docker.sh        # trapezoidal | s_curve | torque_aware
#   PAL_SPEED=0.4 PAL_RENDER_HZ=30 ./start_sim_docker.sh
#   PAL_SCRIPT=pal_scene.py ./start_sim_docker.sh      # the old physics audit, writes out/physics_audit.json
#   PAL_ROBOT=panda PAL_SCRIPT=pal_scene.py ./start_sim_docker.sh   # the original Franka scene
#   PAL_DRIVE_MODE=torque ./start_sim_docker.sh
#   PAL_PHYSICS_HZ=1000 PAL_PAYLOAD_KG=3.0 ./start_sim_docker.sh
#
# Deliberately a different container name and no published port, so it cannot collide with
# the Physyk AI sim (container physyk-isaac-sim, port 8100) if that one is running.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${ISAAC_IMAGE:-nvcr.io/nvidia/isaac-sim:6.0.1}"
NAME=pal-isaac-sim

docker rm -f "$NAME" >/dev/null 2>&1 || true
xhost +local:docker >/dev/null 2>&1 || true

# The Isaac cache is shared with the Physyk project on purpose: it holds the downloaded
# Nucleus assets and the shader cache, and a cold cache costs several minutes of start-up.
CACHE="${PAL_CACHE:-$ROOT/../Physyk_AI-master/.isaac-cache}"
[ -d "$CACHE" ] || CACHE="$ROOT/.isaac-cache"
mkdir -p "$CACHE"/{kit,ov,pip,glcache,computecache} "$ROOT/out" "$ROOT/logs"
chmod -R a+rwX "$CACHE" "$ROOT/out" "$ROOT/logs" 2>/dev/null || true

cleanup() { trap - INT TERM; echo; echo "Stopping $NAME ..."; docker stop -t 5 "$NAME" >/dev/null 2>&1 || docker rm -f "$NAME" >/dev/null 2>&1 || true; exit 130; }
trap cleanup INT TERM

# Isaac drops its physics view or hangs on roughly one start in five (tools/run_sim.sh carries the
# same workaround for the data runs). PAL_ATTEMPTS retries rather than leaving you to notice.
ATTEMPTS="${PAL_ATTEMPTS:-2}"
for attempt in $(seq 1 "$ATTEMPTS"); do
docker rm -f "$NAME" >/dev/null 2>&1 || true
[ "$attempt" -gt 1 ] && echo "--- attempt $attempt of $ATTEMPTS ---"
docker run --name "$NAME" --init --rm --gpus all --network host \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e OMNI_KIT_ACCEPT_EULA=YES \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY="${DISPLAY:-:0}" -e ISAAC_SIM_HEADLESS="${HEADLESS:-0}" \
  ${PAL_PHYSICS_HZ:+-e PAL_PHYSICS_HZ="$PAL_PHYSICS_HZ"} \
  ${PAL_ROBOT:+-e PAL_ROBOT="$PAL_ROBOT"} \
  ${PAL_RENDER_HZ:+-e PAL_RENDER_HZ="$PAL_RENDER_HZ"} \
  ${PAL_STYLE:+-e PAL_STYLE="$PAL_STYLE"} \
  ${PAL_BOX_KG:+-e PAL_BOX_KG="$PAL_BOX_KG"} \
  ${PAL_SPEED:+-e PAL_SPEED="$PAL_SPEED"} \
  ${PAL_CYCLES:+-e PAL_CYCLES="$PAL_CYCLES"} \
  ${PAL_SHOTS:+-e PAL_SHOTS="$PAL_SHOTS"} \
  ${PAL_REALTIME:+-e PAL_REALTIME="$PAL_REALTIME"} \
  ${PAL_DRIVE_MODE:+-e PAL_DRIVE_MODE="$PAL_DRIVE_MODE"} \
  ${PAL_BACKGROUND:+-e PAL_BACKGROUND="$PAL_BACKGROUND"} \
  ${PAL_PAYLOAD_KG:+-e PAL_PAYLOAD_KG="$PAL_PAYLOAD_KG"} \
  -e PAL_OUT=/workspace/out \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$CACHE/kit:/isaac-sim/kit/cache:rw" \
  -v "$CACHE/ov:/root/.cache/ov:rw" \
  -v "$CACHE/pip:/root/.cache/pip:rw" \
  -v "$CACHE/glcache:/root/.cache/nvidia/GLCache:rw" \
  -v "$CACHE/computecache:/root/.nv/ComputeCache:rw" \
  -v "$ROOT/workspace:/workspace:rw" \
  -v "$ROOT/assets/franka_panda:/workspace/assets:ro" \
  -v "$ROOT/assets:/workspace/assets_all:ro" \
  -v "$ROOT/out:/workspace/out:rw" \
  -v "$ROOT/../Physyk_AI-master/workspace:/physyk_workspace:ro" \
  -w /workspace --entrypoint /isaac-sim/python.sh \
  "$IMAGE" "${PAL_SCRIPT:-pal_cell_demo.py}" "$@" &
wait $! && break || true
done
