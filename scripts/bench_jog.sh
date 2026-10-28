#!/usr/bin/env bash
# Jog benchmark: launches the sim headless, runs bench_jog, writes results/jog.csv.
#   scripts/bench_jog.sh            # 3 reps x 3 axes, seed 1
#   REPS=1 OUT=/tmp/jog.csv scripts/bench_jog.sh   # quick check
#   DYNAMICS_BACKEND=kdl scripts/bench_jog.sh       # controllers on KDL
set -euo pipefail
cd "$(dirname "$0")/.."
REPS="${REPS:-3}"
SEED="${SEED:-1}"
OUT="${OUT:-results/jog.csv}"
DYNAMICS_BACKEND="${DYNAMICS_BACKEND:-pinocchio}"
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
scripts/lib/sim_session.sh start payload_mass:=0.0 "dynamics_backend:=${DYNAMICS_BACKEND}"
ros2 run ur5e_teaching_bench bench_jog --out "${OUT}" --reps "${REPS}" --seed "${SEED}" "$@"
echo "[bench_jog] done in $((SECONDS - start)) s wall -> ${OUT}"
