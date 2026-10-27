#!/usr/bin/env bash
# Smoke test: boot the UR5e Gazebo sim headless, check the controllers, send one trajectory,
# and exit non-zero unless /joint_states shows the arm moved.
#
#   scripts/ci_smoke.sh                       # this repo's sim.launch.py (what CI runs)
#   SMOKE_TARGET=stock scripts/ci_smoke.sh    # stock ur_simulation_gz bringup, for comparison
#
# The stock bringup is known to be unreliable on Lyrical: its `ros_gz_sim create` can spawn the
# robot twice when Gazebo is slow to start (see sim.launch.py docstring and NOTES.md).
#
# Gazebo runs server-only (-s). Every process started here lives in one process group that is
# killed on exit, so no gz sim is left behind.
set -euo pipefail

TIMEOUT_S="${SMOKE_TIMEOUT:-180}"
TARGET="${SMOKE_TARGET:-teaching}"
LOG_DIR="${SMOKE_LOG_DIR:-/tmp/ur5e_teaching_smoke}"
JOINTS=(shoulder_pan_joint shoulder_lift_joint elbow_joint wrist_1_joint wrist_2_joint wrist_3_joint)
GOAL=(0.4 -1.2 0.6 -1.57 0.0 0.0)
MIN_MOTION_RAD=0.2

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-11}" GZ_PARTITION="${GZ_PARTITION:-r1}"
mkdir -p "${LOG_DIR}"
set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
if [ "${TARGET}" = "teaching" ]; then
  # shellcheck disable=SC1091
  source "$(dirname "$0")/../install/setup.bash"
fi
set -u

start=${SECONDS}
deadline=$((start + TIMEOUT_S))

if [ "${TARGET}" = "teaching" ]; then
  LAUNCH=(ros2 launch ur5e_teaching_bringup sim.launch.py gazebo_gui:=false launch_rviz:=false
          initial_joint_controller:=joint_trajectory_controller)
else
  # Stock bringup with the stock controller config; the only change is the params_file pointer
  # that ros2_control 6.10.1 needs (see scripts/lib/inject_params_file.py).
  python3 "$(dirname "$0")/lib/inject_params_file.py" \
    /opt/ros/lyrical/share/ur_simulation_gz/config/ur_controllers.yaml "${LOG_DIR}/ur_controllers.yaml"
  LAUNCH=(ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e gazebo_gui:=false
          launch_rviz:=false initial_joint_controller:=joint_trajectory_controller
          "controllers_file:=${LOG_DIR}/ur_controllers.yaml")
fi

echo "[smoke] launching: ${LAUNCH[*]}"
setsid "${LAUNCH[@]}" >"${LOG_DIR}/launch.log" 2>&1 &
LAUNCH_PGID=$!

cleanup() {
  local rc=$?
  echo "[smoke] stopping launch process group ${LAUNCH_PGID}"
  kill -INT -- "-${LAUNCH_PGID}" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 -- "-${LAUNCH_PGID}" 2>/dev/null || break
    sleep 0.5
  done
  kill -KILL -- "-${LAUNCH_PGID}" 2>/dev/null || true
  if [ "${rc}" -ne 0 ]; then
    echo "[smoke] FAILED (rc=${rc}); last 60 lines of the launch log:"
    tail -n 60 "${LOG_DIR}/launch.log" || true
  fi
  exit "${rc}"
}
trap cleanup EXIT INT TERM

remaining() { echo $((deadline - SECONDS)); }

echo "[smoke] waiting for joint_state_broadcaster and joint_trajectory_controller to be active"
until ros2 control list_controllers 2>/dev/null >"${LOG_DIR}/controllers.txt" \
    && grep -Eq "^joint_trajectory_controller .*active" "${LOG_DIR}/controllers.txt" \
    && grep -Eq "^joint_state_broadcaster .*active" "${LOG_DIR}/controllers.txt"; do
  if [ "$(remaining)" -le 0 ]; then
    echo "[smoke] controllers did not become active within ${TIMEOUT_S} s"
    cat "${LOG_DIR}/controllers.txt" || true
    exit 2
  fi
  sleep 2
done
cat "${LOG_DIR}/controllers.txt"
echo "[smoke] controllers active after $((SECONDS - start)) s"

read_positions() {
  timeout "$(remaining)" ros2 topic echo --once --field position /joint_states 2>/dev/null
}

before="$(read_positions)"
echo "[smoke] positions before: ${before//$'\n'/ }"

joint_list=$(IFS=,; echo "${JOINTS[*]}")
goal_list=$(IFS=,; echo "${GOAL[*]}")
echo "[smoke] sending FollowJointTrajectory goal ${goal_list}"
timeout "$(remaining)" ros2 action send_goal \
  /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [${joint_list}], points: [{positions: [${goal_list}], time_from_start: {sec: 4}}]}}" \
  >"${LOG_DIR}/send_goal.txt" 2>&1 || true
tail -n 5 "${LOG_DIR}/send_goal.txt"

after="$(read_positions)"
echo "[smoke] positions after:  ${after//$'\n'/ }"

python3 - "${before}" "${after}" "${MIN_MOTION_RAD}" <<'PY'
import re
import sys

def parse(text):
    return [float(x) for x in re.findall(r"-?\d+\.\d+(?:e[-+]?\d+)?", text)]

before, after, min_motion = parse(sys.argv[1]), parse(sys.argv[2]), float(sys.argv[3])
if len(before) != 6 or len(after) != 6:
    sys.exit(f"[smoke] could not parse joint positions: {before} / {after}")
moved = max(abs(a - b) for a, b in zip(after, before, strict=True))
print(f"[smoke] max joint displacement: {moved:.3f} rad (threshold {min_motion} rad)")
sys.exit(0 if moved >= min_motion else 3)
PY
echo "[smoke] PASS in $((SECONDS - start)) s"
