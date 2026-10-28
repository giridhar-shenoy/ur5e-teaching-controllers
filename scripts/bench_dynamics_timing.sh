#!/usr/bin/env bash
# Dynamics timing benchmark (no Gazebo): per-call wall time of gravity() and rnea() on the
# Pinocchio and KDL backends, 100 000 calls each, single-threaded -> results/dynamics_timing.csv.
#   scripts/bench_dynamics_timing.sh             # the numbers in docs/results.md
#   CALLS=1000 OUT=/tmp/t.csv scripts/bench_dynamics_timing.sh   # quick check
# The URDF is the one the simulation uses (ur5e_teaching.urdf.xacro through xacro). The script
# refuses to run next to a simulator, build, pip install or training run
# (scripts/lib/heavy_processes.sh; UR5E_ALLOW_BUSY=1 skips the check, for draft numbers only); on
# the shared laptop run it under all three locks:
#   flock ~/work/.locks/heavy.lock flock ~/work/.locks/build.lock \
#     flock ~/work/.locks/gazebo.lock scripts/bench_dynamics_timing.sh
set -euo pipefail
cd "$(dirname "$0")/.."
CALLS="${CALLS:-100000}"
SEED="${SEED:-1}"
OUT="${OUT:-results/dynamics_timing.csv}"
set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u

if [ -z "${UR5E_ALLOW_BUSY:-}" ] && ! scripts/lib/heavy_processes.sh; then
  echo "[bench_dynamics_timing] refusing to run: other heavy processes (above) would skew timings" >&2
  exit 3
fi

sha=$(git rev-parse --short=12 HEAD)
git diff --quiet HEAD -- src scripts || sha="${sha}-dirty"
build_type=$(sed -n 's/^CMAKE_BUILD_TYPE:STRING=//p' build/ur5e_teaching_controllers/CMakeCache.txt)
cpu=$(sed -n 's/^model name[[:space:]]*: //p' /proc/cpuinfo | head -1)
urdf=$(mktemp --suffix=.urdf)
trap 'rm -f "${urdf}"' EXIT
xacro "$(ros2 pkg prefix ur5e_teaching_bringup)/share/ur5e_teaching_bringup/urdf/ur5e_teaching.urdf.xacro" \
  ur_type:=ur5e -o "${urdf}"

echo "[bench_dynamics_timing] load average at start: $(cut -d' ' -f1-3 /proc/loadavg)"
echo "[bench_dynamics_timing] top CPU users at start:"
ps -eo pcpu,comm --sort=-pcpu | sed -n 2,6p
start=${SECONDS}
"$(ros2 pkg prefix ur5e_teaching_controllers)/lib/ur5e_teaching_controllers/bench_dynamics_timing" \
  --urdf "${urdf}" --calls "${CALLS}" --seed "${SEED}" --output "${OUT}" \
  --commit "${sha}" --build-type "${build_type:-unset}" --cpu "${cpu}"
echo "[bench_dynamics_timing] load average at end: $(cut -d' ' -f1-3 /proc/loadavg)"
echo "[bench_dynamics_timing] done in $((SECONDS - start)) s wall -> ${OUT}"
