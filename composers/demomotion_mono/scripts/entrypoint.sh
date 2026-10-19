#!/usr/bin/env bash
# demomotion entrypoint
#   stack  (default) pokrene gui_stack (tmuxinator, detached) i čeka; docker stop ga uredno ugasi
#   shell            bash s ROS okruženjem, bez stacka
#   <naredba>        izvrši naredbu s ROS okruženjem
set -eo pipefail

# ROS setup skripte čitaju nepostavljene varijable
source /opt/ros/jazzy/setup.bash
. /root/arms_ws/setup_ros_domain.sh
source /root/arms_ws/install/setup.bash
source /root/py_global/bin/activate
source /opt/demomotion/calibration_env.sh

check_models() {
  local missing=0 f
  for f in \
    /root/arm_perception/models/sam/sam2.1_hiera_large.pt \
    /root/arms_ws/src/perception/WiLoR/pretrained_models/wilor_final.ckpt \
    /root/arms_ws/src/perception/WiLoR/pretrained_models/detector.pt \
    /root/arms_ws/src/perception/WiLoR/mano_data/mano/MANO_RIGHT.pkl; do
    if [ ! -s "$f" ]; then
      echo "[demomotion] NEDOSTAJE MODEL: $f -> $(readlink -f "$f")" >&2
      missing=1
    fi
  done
  if [ ! -d /root/arms_ws/src/megapose6d/local_data/megapose-models/ ]; then
    echo "[demomotion] upozorenje: nema MegaPose modela (/models/megapose), registracija modela neće raditi" >&2
  fi
  if [ ! -d /root/foundationpose/FoundationPose/weights/2024-01-11-20-02-45 ]; then
    echo "[demomotion] upozorenje: nema FoundationPose težina (/models/foundationpose), izbor FoundationPose neće raditi" >&2
  fi
  return $missing
}

# demomotion_calibration u Docker volumeu /root/calibration: prvi start klonira, kasnije
# pull (samo ako nema nespremljenih kalibracija). Bez mreže/SSH-a radi s onim što ima.
sync_calibration() {
  local dir=/root/calibration repo=git@github.com:CroboticSolutions/demomotion_calibration.git
  [ -n "${DEMOMOTION_CALIBRATION_SET:-}" ] || return 0
  if [ ! -d "$dir/.git" ]; then
    if git clone -q "$repo" /tmp/calibration_clone; then
      cp -a /tmp/calibration_clone/. "$dir/" && rm -rf /tmp/calibration_clone
      echo "[demomotion] kloniran demomotion_calibration"
    else
      echo "[demomotion] upozorenje: klon demomotion_calibration nije uspio (SSH agent?); bez kalibracija iz repoa" >&2
    fi
  elif [ -z "$(git -C "$dir" status --porcelain)" ]; then
    git -C "$dir" pull -q --ff-only ||
      echo "[demomotion] upozorenje: pull demomotion_calibration nije uspio, koristim lokalno stanje" >&2
  else
    echo "[demomotion] nespremljene kalibracije u volumeu: ./demomotion.sh save-calibration" >&2
  fi
  mkdir -p "$dir/$DEMOMOTION_CALIBRATION_SET"
}

case "${1:-stack}" in
  stack)
    sync_calibration
    check_models || { echo "[demomotion] Modeli su u imageu (stage models); image je nepotpun, ponovno ./demomotion.sh build." >&2; exit 1; }
    echo "[demomotion] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, verzije koda: /root/arms_ws/VERSIONS.yaml"
    echo "[demomotion] kalibracije: ${ARMS_CALIBRATION_DIR:-/root/.ros/calibration} ($(find "${ARMS_CALIBRATION_DIR:-/root/.ros/calibration}" -name current.yaml 2>/dev/null | wc -l) aktivnih; bez njih nominalne CAD vrijednosti)"

    stop_stack() {
      echo "[demomotion] gasim stack"
      bash /root/arms_ws/src/gui/arm_api2_gui/scripts/stop_gui_stack.sh || true
      tmux kill-server 2>/dev/null || true
      exit 0
    }
    trap stop_stack TERM INT

    tmuxinator start gui_stack --no-attach
    # 'wait' na sleep da TERM odmah prekine čekanje
    while tmux has-session -t gui_stack 2>/dev/null; do
      sleep 2 & wait $!
    done
    echo "[demomotion] tmux sesija gui_stack je završila" >&2
    exit 1
    ;;
  shell)
    exec bash
    ;;
  *)
    exec "$@"
    ;;
esac
