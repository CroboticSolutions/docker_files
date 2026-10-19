#!/usr/bin/env bash
# Commit + push kalibracija postava iz /root/calibration (Docker volume) u demomotion_calibration.
# Poziva se s hosta: ./demomotion.sh save-calibration ["poruka"]
set -euo pipefail
set_name="${DEMOMOTION_CALIBRATION_SET:-}"
[ -n "$set_name" ] || { echo "DEMOMOTION_CALIBRATION_SET nije postavljen u .env" >&2; exit 1; }
cd /root/calibration
[ -d .git ] || { echo "/root/calibration nije git checkout (kontejner ga nije uspio klonirati, vidi logove)" >&2; exit 1; }
[ -n "${GIT_AUTHOR_NAME:-}" ] || export GIT_AUTHOR_NAME=demomotion GIT_COMMITTER_NAME=demomotion
[ -n "${GIT_AUTHOR_EMAIL:-}" ] || export GIT_AUTHOR_EMAIL=demomotion@localhost GIT_COMMITTER_EMAIL=demomotion@localhost

git add -A "$set_name"
if git diff --cached --quiet; then
  echo "Nema novih kalibracija u $set_name."
  exit 0
fi
git status --short
git commit -q -m "${1:-$set_name: kalibracija $(date '+%F %H:%M')}"
git pull -q --rebase
git push -q
echo "Spremljeno: $(git log --oneline -1)"
