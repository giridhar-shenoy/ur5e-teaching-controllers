#!/usr/bin/env bash
# colcon build with the flags used locally and in CI.
# Python3_EXECUTABLE is pinned because CMake's FindPython3 otherwise picks the first versioned
# python3.x on PATH, which on the dev laptop is a uv-managed interpreter without catkin_pkg.
set -euo pipefail
cd "$(dirname "$0")/.."
set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
set -u
export MAKEFLAGS="${MAKEFLAGS:--j4}"
exec colcon build --symlink-install --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPython3_EXECUTABLE=/usr/bin/python3 "$@"
