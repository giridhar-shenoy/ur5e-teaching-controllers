#!/usr/bin/env bash
# Regenerate every number and every GIF in the README from scratch:
#   results/tracking.csv, results/handguide.csv -> docs/results.md, README results block, plots
#   docs/assets/mode_*.gif, docs/assets/hero.gif
# Run from a built workspace (scripts/build.sh). Prints the wall-clock time at the end.
set -euo pipefail
cd "$(dirname "$0")/.."

# On the shared dev laptop, hold the machine-wide Gazebo lock for the whole run so no other
# session starts a simulator between our phases; the per-phase sim launches then use a private
# lock (a second flock on the same file from a child would deadlock).
if [ -z "${UR5E_REGEN_LOCKED:-}" ] && [ -d "$HOME/work/.locks" ]; then
  echo "[regenerate_all] waiting for $HOME/work/.locks/gazebo.lock"
  exec env UR5E_REGEN_LOCKED=1 GAZEBO_LOCK=/tmp/ur5e_teaching_inner.lock \
    flock "$HOME/work/.locks/gazebo.lock" "$0" "$@"
fi

start=${SECONDS}
scripts/bench_tracking.sh
scripts/bench_handguide.sh
python3 scripts/make_results.py
scripts/record_gifs.sh
echo "[regenerate_all] done in $((SECONDS - start)) s wall"
