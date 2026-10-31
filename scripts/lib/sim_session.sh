#!/usr/bin/env bash
# Development helper: start/stop a headless sim in its own process group while holding the
# shared Gazebo lock (~/work/.locks/gazebo.lock on the dev machine), so benchmarks and manual
# checks never overlap with another Gazebo instance.
#   scripts/lib/sim_session.sh start [launch args...]   # returns once controllers are up
#   scripts/lib/sim_session.sh stop
set -euo pipefail
# The shared lock exists on the dev laptop (several sessions share one Gazebo budget); elsewhere a
# private lock file is enough.
if [ -d "$HOME/work/.locks" ]; then
  LOCK="${GAZEBO_LOCK:-$HOME/work/.locks/gazebo.lock}"
else
  LOCK="${GAZEBO_LOCK:-/tmp/ur5e_teaching_gazebo.lock}"
fi
STATE_DIR="${SIM_STATE_DIR:-/tmp/ur5e_teaching_sim}"
MAX_S="${SIM_MAX_S:-1800}"
mkdir -p "${STATE_DIR}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-11}" GZ_PARTITION="${GZ_PARTITION:-r1}"
set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
source "$(dirname "$0")/../../install/setup.bash"
set -u

case "${1:-}" in
  start)
    shift
    # setsid makes flock a session leader (sid = $!). GNU timeout then puts itself and the launch
    # into their own process group inside that session; stop signals that group and lets flock
    # exit last, so the lock is only released once Gazebo is gone.
    setsid flock "${LOCK}" timeout "${MAX_S}" \
      ros2 launch ur5e_teaching_bringup sim.launch.py "$@" >"${STATE_DIR}/launch.log" 2>&1 &
    echo $! >"${STATE_DIR}/sid"
    sid="$(cat "${STATE_DIR}/sid")"
    # Waiting for the lock (another session's Gazebo) has no deadline; the bringup itself gets
    # STARTUP_S once flock has started the launch.
    waited=0
    until pgrep -P "${sid}" timeout >/dev/null; do
      if ! kill -0 "${sid}" 2>/dev/null; then
        echo "[sim] flock exited before the launch started" >&2
        exit 1
      fi
      if [ $((waited % 60)) -eq 0 ]; then
        echo "[sim] waiting for ${LOCK} (${waited} s)"
      fi
      sleep 1
      waited=$((waited + 1))
    done
    deadline=$((SECONDS + ${STARTUP_S:-180}))
    while [ "${SECONDS}" -lt "${deadline}" ]; do
      # Bounded: without a controller manager the CLI waits forever, and a poll that outlives this
      # script keeps the lock descriptors it inherited (it held all three locks once).
      if timeout 10 ros2 control list_controllers 2>/dev/null \
          | grep -Eq "^joint_trajectory_controller .*active"; then
        echo "[sim] up (sid ${sid}, waited ${waited} s for the lock)"
        exit 0
      fi
      sleep 1
    done
    echo "[sim] did not come up; see ${STATE_DIR}/launch.log" >&2
    exit 1
    ;;
  stop)
    sid="$(cat "${STATE_DIR}/sid" 2>/dev/null || true)"
    if [ -n "${sid}" ]; then
      tpid="$(pgrep -P "${sid}" timeout || true)"
      if [ -n "${tpid}" ]; then
        kill -INT -- "-${tpid}" 2>/dev/null || true
        for _ in $(seq 1 40); do
          kill -0 -- "-${tpid}" 2>/dev/null || break
          sleep 0.5
        done
        kill -KILL -- "-${tpid}" 2>/dev/null || true
      fi
      sleep 0.5
      pkill -KILL -s "${sid}" 2>/dev/null || true
      rm -f "${STATE_DIR}/sid"
    fi
    if pgrep -af '[b]in/gz sim' | grep -q "/r1/install/ur5e_teaching_bringup"; then
      echo "[sim] WARNING: a gz sim from this repo is still running" >&2
      exit 1
    fi
    echo "[sim] stopped"
    ;;
  *)
    echo "usage: $0 start [launch args...] | stop" >&2
    exit 2
    ;;
esac
