#!/usr/bin/env bash
# DemoMotion wrapper oko docker compose. Nema pripreme hosta: sve provjere su ovdje.
#   ./demomotion.sh build     povuče vrhove grana iz repos.yaml i builda image
#   ./demomotion.sh up        pokrene stack (builda prvo ako imagea nema); GUI :8080, debug :8091
#   ./demomotion.sh down      ugasi stack (uredno, kroz stop_gui_stack.sh)
#   ./demomotion.sh attach    tmux sesija gui_stack (izlaz: Ctrl-b d)
#   ./demomotion.sh shell     bash u kontejneru koji radi
#   ./demomotion.sh logs      logovi kontejnera
#   ./demomotion.sh versions  točni commitovi u imageu
#   ./demomotion.sh save-calibration ["poruka"]   commit + push kalibracija u demomotion_calibration
set -euo pipefail
cd "$(dirname "$0")"

usage() { sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }
die() { echo "GREŠKA: $*" >&2; exit 1; }

# .env je opcionalan (predložak .env.example); bez njega vrijede zadane vrijednosti
if [ -f .env ]; then set -a; source .env; set +a; fi
export DEMOMOTION_DATA="${DEMOMOTION_DATA:-$HOME/demomotion_data}"
export DEMOMOTION_IMAGE="${DEMOMOTION_IMAGE:-demomotion:latest}"

check_docker() {
  docker info >/dev/null 2>&1 || die "Docker ne radi ili korisnik nije u grupi docker (sudo usermod -aG docker \$USER, pa novi login)"
  docker compose version >/dev/null 2>&1 || die "nema docker compose v2 plugina (paket docker-compose-plugin)"
  docker buildx version >/dev/null 2>&1 || die "nema docker buildx (paket docker-buildx-plugin)"
}

check_ssh() {
  ssh-add -l >/dev/null 2>&1 || die "SSH agent nema ključ s pristupom CroboticSolutions repoima (ssh-add ~/.ssh/id_ed25519)"
}

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

build() {
  check_docker
  check_ssh
  SRC_REVISIONS=$(src_revisions)
  export SRC_REVISIONS
  echo "SRC_REVISIONS=${SRC_REVISIONS}"
  DOCKER_BUILDKIT=1 docker compose build "$@"
}

case "${1:-}" in
  build)
    shift
    build "$@"
    ;;
  up)
    check_docker
    docker info --format '{{json .Runtimes}}' | grep -q nvidia ||
      die "Docker ne vidi NVIDIA runtime: instaliraj nvidia-container-toolkit, pa 'sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker'"
    if command -v ss >/dev/null && ss -ltn | grep -qE ':(8080|8091|9094) '; then
      [ -n "$(docker compose ps -q demomotion 2>/dev/null)" ] ||
        die "port 8080/8091/9094 je zauzet. Radi li gui_stack u razvojnom kontejneru? (tmuxinator stop gui_stack)"
    fi
    if ! docker image inspect "$DEMOMOTION_IMAGE" >/dev/null 2>&1; then
      echo "Image $DEMOMOTION_IMAGE ne postoji, prvo build (prvi put 1-2 h)."
      build
    fi
    # mape za podatke kao korisnik (inače ih Docker napravi kao root)
    mkdir -p "$DEMOMOTION_DATA" 2>/dev/null || { sudo mkdir -p "$DEMOMOTION_DATA" && sudo chown "$USER:" "$DEMOMOTION_DATA"; }
    mkdir -p "$DEMOMOTION_DATA"/{ros_home,bags,diagnostics,block_groups,logs,gui_config,megapose_cache,cache}
    xhost +local:root >/dev/null 2>&1 || true   # RViz iz kontejnera, ako treba
    # SSH agent za kalibracije (klon/pull/push u kontejneru). Socket se mijenja po loginu,
    # pa stalni link koji compose montira (isti trik kao composers/demomotion).
    mkdir -p ~/.ssh
    ssh-add -l >/dev/null 2>&1 || echo "PAZI: SSH agent nema ključ; kontejner neće moći povući/spremiti kalibracije" >&2
    ln -sfn "${SSH_AUTH_SOCK:-/dev/null}" ~/.ssh/ssh_auth_sock
    # zadani postav kalibracija = ime ovog računala
    DEMOMOTION_HOSTNAME=$(hostname)
    export DEMOMOTION_HOSTNAME
    docker compose up -d
    echo "GUI:           http://localhost:8080"
    echo "Debug konzola: http://localhost:8091"
    echo "Podaci:        $DEMOMOTION_DATA"
    echo "Stanje:        docker compose ps (health), ./demomotion.sh logs, ./demomotion.sh attach"
    ;;
  down)     docker compose down ;;
  attach)   docker compose exec demomotion tmux attach -t gui_stack ;;
  shell)    docker compose exec demomotion bash ;;
  logs)     docker compose logs -f ;;
  versions) docker compose run --rm --no-deps demomotion cat /root/arms_ws/VERSIONS.yaml ;;
  save-calibration)
    # git radi u kontejneru (checkout je u njegovom volumeu); autor = tvoj git na hostu
    name=$(git config user.name || true); email=$(git config user.email || true)
    docker compose exec \
      -e GIT_AUTHOR_NAME="$name" -e GIT_AUTHOR_EMAIL="$email" \
      -e GIT_COMMITTER_NAME="$name" -e GIT_COMMITTER_EMAIL="$email" \
      demomotion /opt/demomotion/save_calibration.sh "${2:-}"
    ;;
  *)        usage ;;
esac
