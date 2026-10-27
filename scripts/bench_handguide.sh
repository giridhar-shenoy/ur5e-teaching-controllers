#!/usr/bin/env bash
# Hand-guiding benchmark: model payload 1.0 kg vs actual Gazebo payload 0 / 1 / 2 kg. The actual
# payload is part of the spawned robot, so the sim is relaunched for each value.
#   scripts/bench_handguide.sh           # 3 reps, seed 1 (the numbers in docs/results.md)
set -euo pipefail
cd "$(dirname "$0")/.."
REPS="${REPS:-3}"
SEED="${SEED:-1}"
OUT="${OUT:-results/handguide.csv}"
MODEL_PAYLOAD="${MODEL_PAYLOAD:-1.0}"
ACTUAL_PAYLOADS="${ACTUAL_PAYLOADS:-0.0 1.0 2.0}"
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
for actual in ${ACTUAL_PAYLOADS}; do
  scripts/lib/sim_session.sh start "payload_mass:=${actual}"
  ros2 run ur5e_teaching_bench bench_handguide --out "${OUT}" --reps "${REPS}" --seed "${SEED}" \
    --actual-payload "${actual}" --model-payload "${MODEL_PAYLOAD}"
  scripts/lib/sim_session.sh stop
done
echo "[bench_handguide] done in $((SECONDS - start)) s wall -> ${OUT}"
