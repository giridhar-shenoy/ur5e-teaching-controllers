#!/usr/bin/env bash
# Regenerate every number and every GIF in the README from scratch:
#   results/dynamics_timing.csv, results/tracking.csv, results/handguide.csv
#     -> docs/results.md, README generated blocks, plots
#   docs/assets/hero.gif, docs/assets/tracking.gif, docs/assets/jogging.gif (scripts/record_gifs.sh)
# Run from a built workspace (scripts/build.sh). Prints the wall-clock time at the end.
# Not included: bench_jog.sh (its numbers are not published yet) and the five-run spread in
# results/runs_*/ (one bench_tracking.sh run per CSV, made separately).
#
# It refuses to start while another heavy process runs (scripts/lib/heavy_processes.sh): the
# dynamics timing is a wall-clock measurement, and published numbers should not depend on
# whatever else the machine is doing. UR5E_ALLOW_BUSY=1 skips that check; numbers made that way
# should not be published.
set -euo pipefail
cd "$(dirname "$0")/.."

# On the shared dev laptop, hold the machine-wide locks for the whole run, in the machine's order
# (heavy, build, gazebo), so no other job builds or starts a simulator between our phases.
# The per-phase sim launches then use a private lock (a second flock on the same file from a
# child would deadlock).
if [ -z "${UR5E_REGEN_LOCKED:-}" ] && [ -d "$HOME/work/.locks" ]; then
  echo "[regenerate_all] waiting for the heavy, build and gazebo locks in $HOME/work/.locks"
  exec env UR5E_REGEN_LOCKED=1 GAZEBO_LOCK=/tmp/ur5e_teaching_inner.lock \
    flock "$HOME/work/.locks/heavy.lock" flock "$HOME/work/.locks/build.lock" \
    flock "$HOME/work/.locks/gazebo.lock" "$0" "$@"
fi

if [ -z "${UR5E_ALLOW_BUSY:-}" ] && ! scripts/lib/heavy_processes.sh; then
  echo "[regenerate_all] refusing to start: other heavy processes (above) would skew the" \
    "tracking and timing numbers. Wait for them, or set UR5E_ALLOW_BUSY=1 for a draft run." >&2
  exit 3
fi

start=${SECONDS}
scripts/bench_dynamics_timing.sh  # first, while no simulator of ours is running either
scripts/bench_tracking.sh
scripts/bench_handguide.sh
python3 scripts/make_results.py
scripts/record_gifs.sh
echo "[regenerate_all] done in $((SECONDS - start)) s wall"
