#!/usr/bin/env bash
# DemoMotion wrapper oko docker compose.
#   ./demomotion.sh build     povuče vrhove grana iz repos.yaml i builda image
#   ./demomotion.sh up        pokrene stack (GUI :8080, debug konzola :8091)
#   ./demomotion.sh down      ugasi stack (uredno, kroz stop_gui_stack.sh)
#   ./demomotion.sh attach    tmux sesija gui_stack (izlaz: Ctrl-b d)
#   ./demomotion.sh shell     bash u kontejneru koji radi
#   ./demomotion.sh logs      logovi kontejnera
#   ./demomotion.sh versions  točni commitovi u imageu
#   ./demomotion.sh save-calibration ["poruka"]   commit + push kalibracija u demomotion_calibration
set -euo pipefail
cd "$(dirname "$0")"

usage() { sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }

[ -f .env ] || { echo "Nema .env. Prvo pokreni ./setup_host.sh" >&2; exit 1; }

# Hash vrhova svih grana iz repos.yaml. Mijenja se samo kad se neka grana pomakne,
# pa Docker ponovno klonira i builda kod samo tada (apt/pip slojevi ostaju u cacheu).
src_revisions() {
  awk '$1=="url:"{u=$2} $1=="version:"{print u, $2}' repos.yaml |
    while read -r url ver; do
      sha=$(git ls-remote "$url" "refs/heads/$ver" "refs/tags/$ver" | head -n1 | cut -f1)
      if [ -z "$sha" ]; then
        echo "git ls-remote nije uspio: $url ($ver). Je li SSH ključ u agentu (ssh-add -l)?" >&2
        exit 1
      fi
      echo "$url $ver $sha"
    done | sha256sum | cut -c1-16
}

case "${1:-}" in
  build)
    shift
    ssh-add -l >/dev/null 2>&1 || { echo "SSH agent nema ključ (ssh-add -l). Privatni repoi se kloniraju preko SSH-a." >&2; exit 1; }
    SRC_REVISIONS=$(src_revisions)
    export SRC_REVISIONS
    echo "SRC_REVISIONS=${SRC_REVISIONS}"
    DOCKER_BUILDKIT=1 docker compose build "$@"
    ;;
  up)
    xhost +local:root >/dev/null 2>&1 || true   # RViz iz kontejnera, ako treba
    docker compose up -d
    echo "GUI:           http://localhost:8080"
    echo "Debug konzola: http://localhost:8091"
    echo "Stanje:        docker compose ps (health), ./demomotion.sh attach"
    ;;
  down)     docker compose down ;;
  attach)   docker compose exec demomotion tmux attach -t gui_stack ;;
  shell)    docker compose exec demomotion bash ;;
  logs)     docker compose logs -f ;;
  versions) docker compose run --rm --no-deps demomotion cat /root/arms_ws/VERSIONS.yaml ;;
  save-calibration)
    set -a; source .env; set +a
    [ -n "${DEMOMOTION_CALIBRATION_SET:-}" ] || { echo "DEMOMOTION_CALIBRATION_SET nije postavljen u .env" >&2; exit 1; }
    cal="$DEMOMOTION_DATA/calibration"
    git -C "$cal" add -A "$DEMOMOTION_CALIBRATION_SET"
    if git -C "$cal" diff --cached --quiet; then
      echo "Nema novih kalibracija u $DEMOMOTION_CALIBRATION_SET."
      exit 0
    fi
    git -C "$cal" status --short
    git -C "$cal" commit -q -m "${2:-$DEMOMOTION_CALIBRATION_SET: kalibracija $(date '+%F %H:%M')}"
    git -C "$cal" pull -q --rebase
    git -C "$cal" push -q
    echo "Spremljeno: $(git -C "$cal" log --oneline -1)"
    ;;
  *)        usage ;;
esac
