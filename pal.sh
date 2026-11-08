#!/usr/bin/env bash
# One command for the whole project: the Isaac Sim simulator (in its own window) and the dashboard that shows and drives it.
#
#   ./pal.sh                  start the simulator (headless) and the dashboard; the simulator's picture is inside the dashboard
#   ./pal.sh --window         same, plus an Isaac Sim window on this screen
#   ./pal.sh --dash-only      only the dashboard, on its stand-in plant (no GPU, no Isaac)
#   ./pal.sh --stop           stop the simulator and the dashboard
#   ./pal.sh --status         what is running
#   ./pal.sh --logs           follow the simulator and dashboard logs
#   ./pal.sh --test [suite]   run the simulator test suites (default: all) and then the host-side checks; no dashboard needed
#
# After starting: the simulator is up within a few minutes (the first start is slow while shaders compile); the dashboard is at
# http://127.0.0.1:8050 straight away and says "waiting for the simulator" until the arm is up. Then press "Run pick".
# Logs: logs/pal_live.log (simulator), logs/pal_dashboard.log. State: logs/.pal_dashboard.pid, the container pal-pal-live.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
PORT_DASH="${PAL_DASH_PORT:-8050}"
# The arm. pal_live.py and the dashboard both read their joint count from this spec, so there is
# nothing else to change when it changes. ./start_sim_docker.sh watches the same cell without the
# dashboard attached.
export PAL_ROBOT="${PAL_ROBOT:-ur10e}"
# Sensor noise on the streamed rows: 0 (the default) is ideal sensing. See workspace/pal_sensors.py.
export PAL_SENSOR_NOISE="${PAL_SENSOR_NOISE:-0}"
# Viewport framing for the streamed picture: iso (default), iso_high, iso_close, side, top.
# Defined in workspace/pal_cell_geom.py (CAM_VIEWS).
export PAL_CAM="${PAL_CAM:-iso}"
export PAL_LIVE_PORT="${PAL_LIVE_PORT:-8765}"
export PAL_LIVE_URL="http://127.0.0.1:$PAL_LIVE_PORT"
CONTAINER="pal-pal-live"
PIDS="$ROOT/logs/.pal_dashboard.pid"; SIMPID="$ROOT/logs/.pal_sim.pid"
PYD="$ROOT/.venv-pal/bin/python"; [ -x "$PYD" ] || PYD="$(command -v python3)"
mkdir -p "$ROOT/logs"

alive() { [ -f "$1" ] && kill -0 "$(cat "$1")" 2>/dev/null; }
sim_up() { curl -fs --max-time 1 "$PAL_LIVE_URL/state?since=0" >/dev/null 2>&1; }

stop_all() {
  local did=0
  if alive "$PIDS"; then kill "$(cat "$PIDS")" 2>/dev/null; did=1; fi
  rm -f "$PIDS"
  if alive "$SIMPID"; then kill "$(cat "$SIMPID")" 2>/dev/null; did=1; fi
  rm -f "$SIMPID"
  if docker ps -a --format '{{.Names}}' 2>/dev/null | grep -qx "$CONTAINER"; then docker rm -f "$CONTAINER" >/dev/null 2>&1; did=1; fi
  pkill -f "dashboard/app.py" 2>/dev/null && did=1
  [ "$did" = 1 ] && echo "stopped the simulator and the dashboard" || echo "nothing was running"
}

status() {
  alive "$PIDS" && echo "dashboard : running  http://127.0.0.1:$PORT_DASH  (pid $(cat "$PIDS"))" || echo "dashboard : stopped"
  if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$CONTAINER"; then
    sim_up && echo "simulator : running and answering at $PAL_LIVE_URL" || echo "simulator : container up, still booting (logs/pal_live.log)"
  else echo "simulator : stopped"; fi
}

case "${1:-}" in
  --stop) stop_all; exit 0 ;;
  --status) status; exit 0 ;;
  --logs) exec tail -n 30 -F "$ROOT/logs/pal_live.log" "$ROOT/logs/pal_dashboard.log" ;;
  --test) shift; exec "$ROOT/tools/pal_suite.sh" "${1:-all}" ${2:+"$2"} ;;
  -h|--help) sed -n 2,16p "$0"; exit 0 ;;
esac

MODE=headless
case "${1:-}" in --window) MODE=window ;; --headless) MODE=headless ;; --dash-only) MODE=dash ;; "") ;; *) echo "unknown option $1 (see ./pal.sh --help)"; exit 2 ;; esac

stop_all >/dev/null 2>&1          # a clean start: no stale container, no second dashboard on the port
sleep 1

if [ "$MODE" != dash ]; then
  command -v docker >/dev/null || { echo "docker not found"; exit 1; }
  if [ "$MODE" = window ]; then export HEADLESS=0; else export HEADLESS=1; fi
  export PAL_PHYSICS_HZ="${PAL_PHYSICS_HZ:-250}"      # live view only: 250 Hz keeps up with real time (DECISIONS.md D3); data runs stay at 1000 Hz
  export PAL_RUN_TIMEOUT=604800 PAL_BOOT_TIMEOUT="${PAL_BOOT_TIMEOUT:-900}"
  export PAL_GIT_COMMIT="$(git rev-parse --short HEAD 2>/dev/null || echo uncommitted)"
  nohup "$ROOT/tools/run_sim.sh" pal_live.py "$ROOT/logs/pal_live.log" >"$ROOT/logs/pal_sim_launcher.log" 2>&1 &
  echo $! >"$SIMPID"
  echo "simulator : starting ($MODE); first start can take several minutes"
fi

if [ "$MODE" = dash ]; then export PAL_DASH_SOURCE=live; else export PAL_DASH_SOURCE=isaac; fi
nohup "$PYD" "$ROOT/dashboard/app.py" >"$ROOT/logs/pal_dashboard.log" 2>&1 &
echo $! >"$PIDS"
for _ in $(seq 1 40); do curl -fs --max-time 1 "http://127.0.0.1:$PORT_DASH" >/dev/null 2>&1 && break; sleep 1; done
if curl -fs --max-time 1 "http://127.0.0.1:$PORT_DASH" >/dev/null 2>&1; then
  echo "dashboard : http://127.0.0.1:$PORT_DASH"
  (command -v xdg-open >/dev/null && xdg-open "http://127.0.0.1:$PORT_DASH" >/dev/null 2>&1 &) || true
else
  echo "dashboard did not start; see logs/pal_dashboard.log"; tail -5 "$ROOT/logs/pal_dashboard.log"; exit 1
fi
echo "stop everything with: ./pal.sh --stop      watch the logs with: ./pal.sh --logs"
