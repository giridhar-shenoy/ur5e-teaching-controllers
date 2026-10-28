# Changelog

All notable changes to this project are documented in this file.
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project
uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Repository skeleton: MIT license, Lyrical Dockerfile and devcontainer, GitHub Actions CI that
  builds the image in the job and runs build, lint, tests, a results-consistency check and a
  headless Gazebo smoke test; pre-commit with ruff, clang-format (ROS 2 style), yamllint and
  markdownlint.
- `ur5e_teaching_bringup`: UR5e xacro overlay (position/velocity/effort command interfaces,
  wrist force/torque sensor as a ros2_control sensor, Gazebo-only payload link with xacro
  `payload_mass`), `bench.sdf` (DART, 1 ms, ForceTorque, ApplyLinkWrench), `bench_camera.sdf`,
  `sim.launch.py` (headless by default, `mode:=track|handguide`), `mode.launch.py`, controller
  configuration at 500 Hz.
- `ur5e_teaching_controllers` (C++17 ros2_control plugins, Pinocchio dynamics from the URDF):
  - `PidFfTrajectoryController`: effort; trapezoidal re-profiling under per-joint velocity and
    acceleration limits; PID with conditional-integration anti-windup; URDF torque limits;
    RNEA feedforward on the desired state, switchable at runtime.
  - `GravityCompHandGuidingController`: effort; g(q) + optional Coriolis, viscous damping,
    friction hook, runtime payload mass/CoM, joint-limit repulsion, speed cap that deactivates
    the controller with an error.
  - `PinocchioModel` wrapper and 34 gtest cases (profiles, limits, PID, rigid-body identities on
    the UR5e URDF, controller_manager integration on mock hardware).
- `ur5e_teaching_bench` (Python): strict conflict-aware `mode_manager`, `bench_tracking`,
  `bench_handguide`, `record_mode`.
- Scripts: `ci_smoke.sh`, `bench_tracking.sh`, `bench_handguide.sh`, `record_gifs.sh`,
  `make_results.py`, `regenerate_all.sh`, `build.sh`.
- Docs: README, `docs/results.md` (generated), `docs/architecture.md`, `docs/controllers.md`,
  `docs/what_broke.md` (draft).

### Fixed

- Effort controllers no longer write zero torque on deactivation; doing so dropped the arm under
  gravity for one cycle at every hand-over between controllers.
- Runtime parameter changes are adopted without blocking the realtime loop and only as a whole
  set with correctly sized per-joint arrays; trajectories with NaN/Inf, out-of-limit targets or
  a stale start are rejected; trajectories are never freed in the realtime loop.
- Simulated gravity set to 9.81 m/s² to match Pinocchio (SDF defaults to 9.8).
- Hand-guiding damping raised so a payload-model error cannot carry the arm to the floor within
  two seconds; the hand-guiding controller falls back to the tracking controller on a speed-cap
  trip.

- Work around two failures of the stock `ur_sim_control.launch.py` on Lyrical (no controller
  parameters reach the controllers; duplicate robot spawn when Gazebo boots slowly) in
  `sim.launch.py`.
