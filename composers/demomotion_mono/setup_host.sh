#!/usr/bin/env bash
# Priprema računala za DemoMotion. Pokreće se jednom po računalu, na HOSTU (ne u kontejneru).
# Ne mijenja ništa bez potrebe; za udev pravila traži sudo.
set -uo pipefail
cd "$(dirname "$0")"

ok()   { printf '  \033[32mOK\033[0m   %s\n' "$*"; }
warn() { printf '  \033[33mPAZI\033[0m %s\n' "$*"; }
fail() { printf '  \033[31mNE\033[0m   %s\n' "$*"; FAILED=1; }
FAILED=0

echo "== Docker"
if docker version >/dev/null 2>&1; then ok "docker $(docker version -f '{{.Server.Version}}')"; else fail "docker ne radi ili korisnik nije u grupi docker"; fi
if docker compose version >/dev/null 2>&1; then ok "$(docker compose version)"; else fail "nema docker compose v2 plugina"; fi
if docker buildx version >/dev/null 2>&1; then ok "buildx"; else fail "nema docker buildx (treba za --ssh i cache mountove)"; fi

echo "== GPU"
if command -v nvidia-smi >/dev/null 2>&1; then
  ok "$(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader | head -n1)"
  if docker run --rm --gpus all ubuntu:24.04 nvidia-smi -L >/dev/null 2>&1; then
    ok "NVIDIA Container Toolkit (--gpus all radi)"
  else
    fail "--gpus all ne radi: instaliraj nvidia-container-toolkit i 'sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker'"
  fi
else
  fail "nema NVIDIA drivera (nvidia-smi)"
fi

echo "== SSH (privatni repoi u buildu)"
if ssh-add -l >/dev/null 2>&1; then ok "ključ u agentu"; else warn "ssh-add -l nema ključ; build ga treba (ssh-add ~/.ssh/id_ed25519)"; fi

echo "== Orbbec udev pravila"
if [ -f /etc/udev/rules.d/99-obsensor-libusb.rules ]; then
  ok "već instalirana"
else
  echo "  instaliram udev/99-obsensor-libusb.rules (sudo)"
  sudo cp udev/99-obsensor-libusb.rules /etc/udev/rules.d/ &&
    sudo udevadm control --reload-rules && sudo udevadm trigger && ok "instalirano" || fail "udev pravila"
fi

echo "== CAN (Piper)"
if ip link show can0 >/dev/null 2>&1; then
  ok "can0 postoji (GUI wizard ga sam digne na 1 Mbit/s)"
else
  warn "nema can0: spoji Piper USB-CAN adapter (ako je spojen: sudo modprobe gs_usb)"
fi

echo "== .env"
if [ ! -f .env ]; then
  cp .env.example .env
  warn "napravljen .env iz .env.example; provjeri DEMOMOTION_DATA"
else
  ok ".env postoji"
fi
set -a; source .env; set +a

echo "== Podaci ($DEMOMOTION_DATA)"
mkdir -p "$DEMOMOTION_DATA" 2>/dev/null || { sudo mkdir -p "$DEMOMOTION_DATA" && sudo chown "$USER:" "$DEMOMOTION_DATA"; }
mkdir -p "$DEMOMOTION_DATA"/{ros_home/calibration,bags,diagnostics,block_groups,logs,gui_config,megapose_cache,cache}
ok "mape postoje"
if [ -z "$(ls -A "$DEMOMOTION_DATA/ros_home/calibration" 2>/dev/null)" ]; then
  warn "nema kalibracije u $DEMOMOTION_DATA/ros_home/calibration (hand-eye, TCP). Vidi README, 'Kalibracija'."
fi

echo
if [ "$FAILED" = 0 ]; then
  echo "Spremno. Sljedeće: ./demomotion.sh build && ./demomotion.sh up"
else
  echo "Popravi stavke označene s NE pa ponovno pokreni ./setup_host.sh"
  exit 1
fi
