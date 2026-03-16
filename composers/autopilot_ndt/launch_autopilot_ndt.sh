#!/bin/bash
# =============================================================================
# launch_minithex.sh
# Minithex simulation system launcher
# Starts Docker containers and boots each system component.
#
# Usage:
#   ./launch_minithex.sh              # start all components
#   ./launch_minithex.sh --sim-only   # start simulation container only
#   ./launch_minithex.sh --attach     # attach to all containers after launch
#   ./launch_minithex.sh --stop       # stop all system containers
# =============================================================================

set -euo pipefail

# ─── Colors ───────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# ─── Configuration ─────────────────────────────────────────────────────────────
# Container names
SIM_CONTAINER="genom_gz_classic_react_cont"
LOC_CONTAINER="ros_slam_noetic_cont"
EXP_CONTAINER="frontier_exp"

# Commands to run inside each container
SIM_CMD="tmuxinator start sim_minithex_demo"
LOC_CMD="roslaunch fast_livo mapping_sim.launch"
EXP_CMD="./start.sh"

# Working directory for exploration start script (adjust if needed)
EXP_WORKDIR="/root/uav_ws/src/uav_frontier_exploration_3d/scripts/startup"

# Seconds to wait between launching each component (let ROS master init)
LAUNCH_DELAY=3

# ─── Flags ─────────────────────────────────────────────────────────────────────
SIM_ONLY=false
ATTACH_AFTER=false
STOP_MODE=false

for arg in "$@"; do
  case $arg in
    --sim-only)  SIM_ONLY=true ;;
    --attach)    ATTACH_AFTER=true ;;
    --stop)      STOP_MODE=true ;;
    --help|-h)
      grep '^# ' "$0" | head -12 | sed 's/^# //'
      exit 0
      ;;
    *)
      echo -e "${RED}Unknown argument: $arg${NC}" && exit 1 ;;
  esac
done

# ─── Logging helpers ───────────────────────────────────────────────────────────
log()     { echo -e "${GREEN}[$(date '+%H:%M:%S')]${NC} $1"; }
info()    { echo -e "${CYAN}[$(date '+%H:%M:%S')]${NC} $1"; }
warn()    { echo -e "${YELLOW}[$(date '+%H:%M:%S')] WARN:${NC} $1"; }
error()   { echo -e "${RED}[$(date '+%H:%M:%S')] ERROR:${NC} $1" >&2; exit 1; }
section() { echo -e "\n${BOLD}${BLUE}── $1 ${NC}"; }

# ─── Container helpers ─────────────────────────────────────────────────────────

# Returns 0 if container is currently running
is_running() {
  docker ps --format '{{.Names}}' | grep -q "^${1}$"
}

# Returns 0 if container exists (running or stopped)
container_exists() {
  docker ps -a --format '{{.Names}}' | grep -q "^${1}$"
}

start_container() {
  local name="$1"
  if is_running "$name"; then
    warn "Container '${name}' is already running — skipping docker start."
  elif container_exists "$name"; then
    log "Starting container '${name}'..."
    docker start "$name" > /dev/null
  else
    error "Container '${name}' does not exist. Please create it first (docker run ...)."
  fi
}

# Wait until we can exec into the container (up to $2 seconds)
wait_ready() {
  local name="$1"
  local timeout="${2:-15}"
  local elapsed=0
  info "Waiting for '${name}' to be exec-ready..."
  until docker exec "$name" true &>/dev/null; do
    sleep 1
    ((elapsed++))
    if [ "$elapsed" -ge "$timeout" ]; then
      error "Container '${name}' did not become ready within ${timeout}s."
    fi
  done
  log "Container '${name}' is ready."
}

# Run a command detached inside a container, wrapped in a login shell
run_detached() {
  local container="$1"
  local cmd="$2"
  local workdir="${3:-}"

  local exec_flags=(-d)
  [ -n "$workdir" ] && exec_flags+=(-w "$workdir")

  log "  → [${container}] $ ${cmd}"
  docker exec "${exec_flags[@]}" "$container" bash -lc "$cmd"
}

stop_container() {
  local name="$1"
  if is_running "$name"; then
    log "Stopping container '${name}'..."
    docker stop "$name" > /dev/null
  else
    info "Container '${name}' is not running."
  fi
}

# ─── Stop mode ─────────────────────────────────────────────────────────────────
if $STOP_MODE; then
  section "Stopping all system containers"
  stop_container "$SIM_CONTAINER"
  stop_container "$LOC_CONTAINER"
  stop_container "$EXP_CONTAINER"
  log "All containers stopped."
  exit 0
fi

# ─── Banner ────────────────────────────────────────────────────────────────────
echo -e "${BOLD}${BLUE}"
echo "╔══════════════════════════════════════════════════╗"
echo "║        Minithex System Launcher                  ║"
echo "║        $(date '+%Y-%m-%d %H:%M:%S')                       ║"
echo "╚══════════════════════════════════════════════════╝"
echo -e "${NC}"

# ─── Phase 1: Start containers ─────────────────────────────────────────────────
section "Starting containers"
start_container "$SIM_CONTAINER"

if ! $SIM_ONLY; then
  start_container "$LOC_CONTAINER"
  start_container "$EXP_CONTAINER"
fi

# ─── Phase 2: Wait for readiness ───────────────────────────────────────────────
section "Waiting for containers"
wait_ready "$SIM_CONTAINER"

if ! $SIM_ONLY; then
  wait_ready "$LOC_CONTAINER"
  wait_ready "$EXP_CONTAINER"
fi

# ─── Phase 3: Launch components ────────────────────────────────────────────────
section "Launching components"

log "[1/3] Simulation — tmuxinator"
run_detached "$SIM_CONTAINER" "$SIM_CMD"

if ! $SIM_ONLY; then
  log "Waiting ${LAUNCH_DELAY}s for ROS master to initialise..."
  sleep "$LAUNCH_DELAY"

  log "[2/3] Localization — FAST-LiVO"
  run_detached "$LOC_CONTAINER" "$LOC_CMD"

  sleep "$LAUNCH_DELAY"

  log "[3/3] Exploration — frontier"
  run_detached "$EXP_CONTAINER" "$EXP_CMD" "$EXP_WORKDIR"
fi

# ─── Done ──────────────────────────────────────────────────────────────────────
echo ""
log "${BOLD}All components launched successfully.${NC}"
echo ""
echo -e "${CYAN}Useful commands:${NC}"
echo "  Attach to simulation:    docker exec -it ${SIM_CONTAINER} tmux attach"
echo "  Attach to localization:  docker exec -it ${LOC_CONTAINER} bash"
echo "  Attach to exploration:   docker exec -it ${EXP_CONTAINER} bash"
echo "  Stop everything:         $0 --stop"
echo ""

# ─── Optional: open new terminal per container ─────────────────────────────────
if $ATTACH_AFTER; then
  section "Attaching terminals"
  # Uses gnome-terminal; swap for xterm/konsole/etc. if needed
  if command -v gnome-terminal &>/dev/null; then
    gnome-terminal -- bash -c "docker exec -it ${SIM_CONTAINER} tmux attach; exec bash" &
    if ! $SIM_ONLY; then
      gnome-terminal -- bash -c "docker exec -it ${LOC_CONTAINER} bash; exec bash" &
      gnome-terminal -- bash -c "docker exec -it ${EXP_CONTAINER} bash; exec bash" &
    fi
  else
    warn "--attach requires gnome-terminal. Open containers manually using the commands above."
  fi
fi
