# NOTES — ur5e-teaching-controllers

Session log: decisions, unverified items, time spent, commands that worked.
A fresh session should read this file, CHECKPOINTS.md, MORNING.md and `git log -20` first.

## Session 1 — 2026-10-28, overnight (started 03:25 IST)

Environment for every shell: `export ROS_DOMAIN_ID=11 GZ_PARTITION=r1 MAKEFLAGS=-j4`,
then `source /opt/ros/lyrical/setup.bash` (and `install/setup.bash` after a build).

### Plan (numbered; scope = SPRINT OVERRIDE MUST list for v0.1)

1. Plumbing: git init, LICENSE, README stub + CI badge, NOTES/CHANGELOG/THIRD_PARTY,
   Dockerfile (FROM `ros:lyrical-ros-base`, verified in SHARED.md), devcontainer,
   `.github/workflows/ci.yml` (docker build in the job, then build/lint/test inside it),
   pre-commit (ruff, ruff-format, clang-format ROS 2 style, yamllint, markdownlint-cli2),
   docs skeletons, `scripts/regenerate_all.sh`. Create the PRIVATE GitHub repo, push, get CI green.
2. Stock sim headless (`ur_sim_control.launch.py gazebo_gui:=false launch_rviz:=false`),
   verify controllers, send one trajectory, write `scripts/ci_smoke.sh` (180 s timeout, non-zero
   if the robot does not move). Run it in CI under `xvfb-run -a` + `LIBGL_ALWAYS_SOFTWARE=1`.
   → CHECKPOINT 1.
3. `ur5e_teaching_bringup`: `ur5e_teaching.urdf.xacro` (ur_description macro, gz_ros2_control,
   effort/velocity/position on all joints, F/T sensor on wrist_3_joint as a ros2_control
   `<sensor>`, payload link with xacro `payload_mass`), `worlds/bench.sdf`
   (ApplyLinkWrench, Sensors/ogre2, world camera), `sim.launch.py` that *includes*
   `ur_sim_control.launch.py` with our `description_file`/`controllers_file`/`world_file`.
4. `ur5e_teaching_controllers` (C++17, ament_cmake, generate_parameter_library, pluginlib):
   - `dynamics/pinocchio_model.{hpp,cpp}`: URDF → Pinocchio, gravity, RNEA, frame Jacobian,
     payload inertia update.
   - `profiles/trapezoidal.hpp` header-only, unit-tested.
   - (a) `PidFfTrajectoryController`: effort; topic `~/joint_trajectory`; trapezoidal
     re-profiling; PID + anti-windup + torque limits + RNEA feedforward (on/off param).
   - (b) `GravityCompHandGuidingController`: effort; g(q) + optional Coriolis, viscous
     damping, friction hook, runtime `payload_mass`/`payload_com`, joint-limit repulsion,
     speed cap → deactivate with error.
   - gtests ≥ 12 (profiles, limits, model wrapper).
5. `ur5e_teaching_bench` (Python): `mode_manager` node (strict switch_controller), `mode.launch.py`
   (track|handguide), benchmark drivers. → CHECKPOINT 2.
6. Benchmarks: `bench_tracking` (3 speeds × FF on/off × 3 reps), `bench_handguide`
   (model 1.0 kg vs actual 0/1/2 kg, drift + effort-to-move, 3 reps). CSVs stamped with commit,
   step size, update rate, seed. → CHECKPOINT 3.
7. `scripts/record_gifs.sh` (headless rendering per SHARED.md, world camera → frames → ffmpeg),
   two mode GIFs + hero GIF ≤ 3 MB.
8. README (required shape), docs/results.md (generated from CSVs by script), docs/architecture.md
   Mermaid, docs/what_broke.md DRAFT (400–600 words, no invented numbers), CHANGELOG, NOTES.
   → CHECKPOINT 4. MORNING.md STATUS.
9. SHOULD (only if MUST is green): TcpJogController + bench_jog.

### Decisions

- **CI container**: GitHub's `container:` key needs a registry image, and the prompt forbids
  depending on a registry. The job therefore runs on `ubuntu-24.04`, builds the image with
  `docker/build-push-action` (`load: true`, GHA layer cache = the apt cache), and runs every
  build/test step via `docker run` inside that image. Lint (pre-commit) runs on the runner.
- **clang-format style**: `ament_clang_format` is not installed on this machine, so its
  `.clang-format` (Google-based ROS 2 style) is reproduced in the repo root; see THIRD_PARTY.md.
- **Git author**: the global git config on this machine is `Giridhar J Shenoy
  <the old work e-mail>`; I did not change it (flagged in MORNING.md).

### Verified facts (command → answer)

- `python3 --version` → Python 3.14.4 (system). ROS_DISTRO=lyrical.
- `dpkg-query -W ros-lyrical-*`: ur-simulation-gz 2.5.0, ur-description 4.3.1,
  gz-ros2-control 3.0.9, ros2-controllers 6.9.0, controller-manager 6.10.1, ros-gz-bridge 3.0.10,
  ros-gz-sim 3.0.10, pinocchio 4.1.0, generate-parameter-library 1.3.0, realtime-tools 5.4.0,
  kinematics-interface(-kdl) 2.4.2, ros2-control-test-assets 6.10.1, xacro 2.1.1.
- `ros-lyrical-hardware-interface-testing` is **not installed** (not needed: mock hardware lives
  in `hardware_interface` as `mock_components/GenericSystem`).
- ros2_control 6.10.1 API: `LoanedCommandInterface::set_value()` is `[[nodiscard]] bool`;
  reads use `get_optional<double>()` (`get_value()` is deprecated).
- `ur_sim_control.launch.py` default `initial_joint_controller` is
  `scaled_joint_trajectory_controller` (not `joint_trajectory_controller` as the prompt says);
  stock controllers yaml sets `update_rate: 500`.
- `ur_description/urdf/inc/ur_joint_control.xacro` already declares position, velocity and
  effort command interfaces on every joint.
- UR5e effort limits (ur_description `joint_limits.yaml`): 150 N·m (base three), 28 N·m (wrist).
- Tools: `pre-commit`, `ruff`, `yamllint`, `markdownlint-cli2` are not on PATH → installed into
  `~/work/r1/.venv` (pip) / run via pre-commit's own node env.

### Unverified items

(none yet)

### Time log

- 03:25 start; read MACHINE/SHARED/ISOLATION; package checks.
