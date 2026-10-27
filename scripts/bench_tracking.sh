#!/usr/bin/env bash
# Tracking benchmark: launches the sim headless, runs bench_tracking, writes results/tracking.csv.
#   scripts/bench_tracking.sh            # 3 reps, seed 1 (the numbers in docs/results.md)
#   REPS=1 scripts/bench_tracking.sh     # quick check
set -euo pipefail
cd "$(dirname "$0")/.."
REPS="${REPS:-3}"
SEED="${SEED:-1}"
OUT="${OUT:-results/tracking.csv}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-11}" GZ_PARTITION="${GZ_PARTITION:-r1}"
export UR5E_TEACHING_REPO="$PWD"
set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u

rm -f "${OUT}"
trap 'scripts/lib/sim_session.sh stop' EXIT
start=${SECONDS}
scripts/lib/sim_session.sh start payload_mass:=0.0
ros2 run ur5e_teaching_bench bench_tracking --out "${OUT}" --reps "${REPS}" --seed "${SEED}"
echo "[bench_tracking] done in $((SECONDS - start)) s wall -> ${OUT}"
