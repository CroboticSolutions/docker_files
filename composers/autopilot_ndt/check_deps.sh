#!/bin/bash
# =============================================================================
# check_deps.sh
# Checks all dependencies required by launch_autopilot_ndt.sh
# Usage: ./check_deps.sh
# =============================================================================

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BOLD='\033[1m'
NC='\033[0m'

PASS=0
FAIL=0
WARN=0

ok()   { echo -e "  ${GREEN}✔${NC}  $1"; ((PASS++)); }
fail() { echo -e "  ${RED}✘${NC}  $1"; ((FAIL++)); }
warn() { echo -e "  ${YELLOW}⚠${NC}  $1"; ((WARN++)); }

echo -e "\n${BOLD}── System dependencies ──────────────────────────────${NC}"

# docker
if command -v docker &>/dev/null; then
  ok "docker          $(docker --version | awk '{print $3}' | tr -d ',')"
else
  fail "docker          not found  →  https://docs.docker.com/engine/install/"
fi

# docker daemon running
if docker info &>/dev/null 2>&1; then
  ok "docker daemon   running"
else
  fail "docker daemon   not running  →  sudo systemctl start docker"
fi

# terminator
if command -v terminator &>/dev/null; then
  ok "terminator      $(terminator --version 2>&1 | head -1)"
else
  fail "terminator      not found  →  sudo apt install terminator"
fi

# tmux (needed inside sim container, but useful to have on host too)
if command -v tmux &>/dev/null; then
  ok "tmux            $(tmux -V)"
else
  warn "tmux            not found on host (only needed inside container)"
fi

# xauth (for X11 forwarding / GUI apps)
if command -v xauth &>/dev/null; then
  ok "xauth           $(xauth --version 2>&1 | head -1)"
else
  warn "xauth           not found  →  sudo apt install xauth  (needed for GUI)"
fi

# DISPLAY set
if [ -n "${DISPLAY:-}" ]; then
  ok "DISPLAY         $DISPLAY"
else
  warn "DISPLAY         not set — GUI apps inside containers may not render"
fi

echo -e "\n${BOLD}── Docker containers ────────────────────────────────${NC}"

check_container() {
  local name="$1"
  if docker ps -a --format '{{.Names}}' | grep -q "^${name}$"; then
    local status
    status=$(docker inspect --format '{{.State.Status}}' "$name")
    if [ "$status" = "running" ]; then
      ok "${name}   (running)"
    else
      warn "${name}   exists but is ${status}"
    fi
  else
    fail "${name}   does not exist — run its docker run script first"
  fi
}

check_container "genom_gz_classic_react_cont"
check_container "ros_slam_noetic_cont"
check_container "frontier_exp"

echo -e "\n${BOLD}── Summary ───────────────────────────────────────────${NC}"
echo -e "  ${GREEN}✔ Passed:${NC}  $PASS"
[ $WARN -gt 0 ] && echo -e "  ${YELLOW}⚠ Warnings:${NC} $WARN"
[ $FAIL -gt 0 ] && echo -e "  ${RED}✘ Failed:${NC}  $FAIL"
echo ""

if [ $FAIL -gt 0 ]; then
  echo -e "${RED}Fix the failed checks before running launch_minithex.sh.${NC}"
  exit 1
elif [ $WARN -gt 0 ]; then
  echo -e "${YELLOW}Warnings present — launcher may work but some features could fail.${NC}"
  exit 0
else
  echo -e "${GREEN}All checks passed — ready to launch.${NC}"
  exit 0
fi
