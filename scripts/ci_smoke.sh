#!/usr/bin/env bash
# Smoke test: boot the UR5e Gazebo sim headless, check the controllers, send one trajectory,
# and exit non-zero unless /joint_states shows the arm moved.
#
#   scripts/ci_smoke.sh                       # this repo's sim.launch.py (what CI runs)
#   SMOKE_BACKEND=kdl scripts/ci_smoke.sh     # same, both effort controllers on orocos KDL
#   SMOKE_TARGET=stock scripts/ci_smoke.sh    # stock ur_simulation_gz bringup, for comparison
#
# For this repo's bringup it then switches to mode track and has the effort tracking controller
# follow a trajectory with each control law (pid_ff, computed_torque), switches to mode handguide
# and checks that the arm floats still, switches to mode jog and checks that a streamed TCP twist
# moves the arm and that it stops after the stream ends, and checks that the effort controllers
# were configured with the requested dynamics backend.
# The stock bringup is known to be unreliable on Lyrical: its `ros_gz_sim create` can spawn the
# robot twice when Gazebo is slow to start (see the sim.launch.py docstring and
# docs/what_broke.md, F7).
#
# Gazebo runs server-only (-s). Every process started here lives in one process group that is
# killed on exit, so no gz sim is left behind.
set -euo pipefail

TIMEOUT_S="${SMOKE_TIMEOUT:-180}"
TARGET="${SMOKE_TARGET:-teaching}"
BACKEND="${SMOKE_BACKEND:-pinocchio}"
LOG_DIR="${SMOKE_LOG_DIR:-/tmp/ur5e_teaching_smoke}"
JOINTS=(shoulder_pan_joint shoulder_lift_joint elbow_joint wrist_1_joint wrist_2_joint wrist_3_joint)
GOAL=(0.4 -1.2 0.6 -1.57 0.0 0.0)
MIN_MOTION_RAD=0.2
# Effort-tracking check: back to home under pid_ff_trajectory_controller.
TRACK_GOAL=(0.0 -1.57 1.57 -1.57 -1.57 0.0)
TRACK_TOL_RAD=0.01
# Hand-guiding check: with the model payload equal to the actual one (0 kg) the arm must not move.
HOLD_TOL_RAD=0.005
# Jog check: 1 s of 5 cm/s along base z must move the joints, and the deadman must stop them.
JOG_MIN_MOTION_RAD=0.01
JOG_STOP_TOL_RAD=0.001

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
          initial_joint_controller:=joint_trajectory_controller "dynamics_backend:=${BACKEND}")
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

# One-shot publish to a controller topic. --keep-alive: a publisher that exits right after
# publishing can lose the message before reliable delivery completes; --max-wait-time-secs: give
# up on discovery after 30 s instead of eating the whole budget, and say what the graph shows.
publish_once() {
  local topic=$1 type=$2 msg=$3 log=$4
  if ! timeout "$(remaining)" ros2 topic pub --once -w 1 --max-wait-time-secs 30 --keep-alive 2 \
      "${topic}" "${type}" "${msg}" >"${log}" 2>&1; then
    echo "[smoke] publishing to ${topic} failed; publisher output and topic info:"
    cat "${log}"
    timeout 10 ros2 topic info -v "${topic}" || true
    return 1
  fi
}

echo "[smoke] waiting for joint_state_broadcaster and joint_trajectory_controller to be active"
until timeout 10 ros2 control list_controllers 2>/dev/null >"${LOG_DIR}/controllers.txt" \
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

# Six positions in JOINTS order (/joint_states is not in that order; map by name).
read_ordered() {
  timeout "$(remaining)" python3 "$(dirname "$0")/lib/joint_positions.py" 10
}

# max |a - b| over two space-separated lists of six positions
max_diff() {
  python3 - "$1" "$2" <<'PY'
import sys

a, b = [float(x) for x in sys.argv[1].split()], [float(x) for x in sys.argv[2].split()]
if len(a) != 6 or len(b) != 6:
    sys.exit(f"[smoke] could not parse joint positions: {a} / {b}")
print(f"{max(abs(x - y) for x, y in zip(a, b, strict=True)):.2e}")
PY
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

if [ "${TARGET}" = "teaching" ]; then
  # Both teaching modes must switch cleanly and leave exactly the expected controller active.
  for mode in track handguide jog; do
    case "${mode}" in
      track) expected=pid_ff_trajectory_controller ;;
      handguide) expected=gravity_comp_handguide_controller ;;
      jog) expected=tcp_jog_controller ;;
    esac
    timeout "$(remaining)" ros2 launch ur5e_teaching_bringup mode.launch.py "mode:=${mode}" \
      >"${LOG_DIR}/mode_${mode}.txt" 2>&1
    timeout 10 ros2 control list_controllers >"${LOG_DIR}/controllers_${mode}.txt"
    active=$(awk '$3 == "active" {print $1}' "${LOG_DIR}/controllers_${mode}.txt" | sort | tr '\n' ' ')
    echo "[smoke] mode ${mode}: active = ${active}"
    if [ "$(awk -v c="${expected}" '$1 == c {print $3}' "${LOG_DIR}/controllers_${mode}.txt")" != "active" ] \
        || [ "$(awk '$1 == "joint_trajectory_controller" {print $3}' "${LOG_DIR}/controllers_${mode}.txt")" = "active" ]; then
      echo "[smoke] mode ${mode} did not leave ${expected} as the only joint controller"
      exit 4
    fi
    case "${mode}" in
      track)
        track_list=$(IFS=,; echo "${TRACK_GOAL[*]}")
        echo "[smoke] effort tracking: trajectory to ${track_list} in 3 s"
        publish_once /pid_ff_trajectory_controller/joint_trajectory \
          trajectory_msgs/msg/JointTrajectory \
          "{joint_names: [${joint_list}], points: [{positions: [${track_list}], time_from_start: {sec: 3}}]}" \
          "${LOG_DIR}/track_pub.txt" || exit 5
        sleep 3
        err=$(max_diff "$(read_ordered)" "${TRACK_GOAL[*]}")
        echo "[smoke] effort tracking: max |q - goal| = ${err} rad (tolerance ${TRACK_TOL_RAD})"
        python3 -c "import sys; sys.exit(0 if ${err} <= ${TRACK_TOL_RAD} else 1)" \
          || { cat "${LOG_DIR}/track_pub.txt"; exit 5; }
        # Same controller, computed-torque law, back to the first goal.
        timeout "$(remaining)" ros2 param set /pid_ff_trajectory_controller control_law \
          computed_torque >"${LOG_DIR}/param_ct.txt" 2>&1 || { cat "${LOG_DIR}/param_ct.txt"; exit 5; }
        echo "[smoke] computed torque: trajectory to ${goal_list} in 3 s"
        publish_once /pid_ff_trajectory_controller/joint_trajectory \
          trajectory_msgs/msg/JointTrajectory \
          "{joint_names: [${joint_list}], points: [{positions: [${goal_list}], time_from_start: {sec: 3}}]}" \
          "${LOG_DIR}/track_ct_pub.txt" || exit 5
        sleep 3
        err=$(max_diff "$(read_ordered)" "${GOAL[*]}")
        echo "[smoke] computed torque: max |q - goal| = ${err} rad (tolerance ${TRACK_TOL_RAD})"
        python3 -c "import sys; sys.exit(0 if ${err} <= ${TRACK_TOL_RAD} else 1)" \
          || { cat "${LOG_DIR}/track_ct_pub.txt"; exit 5; }
        timeout "$(remaining)" ros2 param set /pid_ff_trajectory_controller control_law pid_ff \
          >>"${LOG_DIR}/param_ct.txt" 2>&1
        ;;
      handguide)
        sleep 0.5
        p0="$(read_ordered)"
        sleep 1
        drift=$(max_diff "$(read_ordered)" "${p0}")
        echo "[smoke] hand guiding: max joint drift over 1 s = ${drift} rad (tolerance ${HOLD_TOL_RAD})"
        python3 -c "import sys; sys.exit(0 if ${drift} <= ${HOLD_TOL_RAD} else 1)" || exit 6
        ;;
      jog)
        p0="$(read_ordered)"
        echo "[smoke] jog: streaming 5 cm/s along base z at 50 Hz for 1 s"
        timeout "$(remaining)" ros2 topic pub -w 1 -r 50 --times 50 /tcp_jog_controller/twist \
          geometry_msgs/msg/TwistStamped "{header: {frame_id: world}, twist: {linear: {z: 0.05}}}" \
          >"${LOG_DIR}/jog_pub.txt" 2>&1
        sleep 0.5  # deadman (0.1 s) and deceleration
        p1="$(read_ordered)"
        sleep 0.5
        p2="$(read_ordered)"
        moved=$(max_diff "${p1}" "${p0}")
        after=$(max_diff "${p2}" "${p1}")
        echo "[smoke] jog: moved ${moved} rad (min ${JOG_MIN_MOTION_RAD}), then ${after} rad in 0.5 s (max ${JOG_STOP_TOL_RAD})"
        python3 -c "import sys; sys.exit(0 if ${moved} >= ${JOG_MIN_MOTION_RAD} and ${after} <= ${JOG_STOP_TOL_RAD} else 1)" || exit 8
        ;;
    esac
  done
  configured=$(grep -c "dynamics backend: ${BACKEND}," "${LOG_DIR}/launch.log" || true)
  echo "[smoke] effort controllers configured with the ${BACKEND} backend: ${configured}"
  if [ "${configured}" -lt 3 ]; then
    echo "[smoke] expected all three effort controllers to report 'dynamics backend: ${BACKEND}'"
    exit 7
  fi
fi
echo "[smoke] PASS in $((SECONDS - start)) s"
