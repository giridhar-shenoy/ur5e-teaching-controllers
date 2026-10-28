# Changelog

All notable changes to this project are documented in this file.
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project
uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Second dynamics backend on orocos KDL (`KdlModel`: kdl_parser chain `base_link → tool0`,
  `ChainIdSolver_RNE`, `ChainDynParam`, `ChainJntToJacSolver`) behind a common `DynamicsModel`
  interface that `PinocchioModel` now implements; read-only `dynamics_backend`
  (`pinocchio` | `kdl`, default `pinocchio`) and `gravity_z` (default −9.81) parameters on both
  controllers; `sim.launch.py dynamics_backend:=kdl`.
- `test_dynamics_equivalence`: both backends on 200 random states at 0 kg and 1 kg payload
  (gravity and RNEA within 1e-6 N·m, M within 1e-6, J within 1e-9); KDL controller tests on
  mock hardware; a test that the worlds' gravity equals `gravity_z`.
- `bench_dynamics_timing` (C++) and `scripts/bench_dynamics_timing.sh`: per-call median / p95
  of gravity() and rnea() on both backends and their share of the 2 ms cycle,
  `results/dynamics_timing.csv`, a table in `docs/results.md` and a row in the README; CI runs
  it as a 1 000-call smoke test.
- `scripts/lib/heavy_processes.sh`: `regenerate_all.sh` and the timing benchmark refuse to
  start next to a simulator, build, pip install or training run.
- The smoke test now runs both effort controllers in Gazebo (tracking reaches its goal,
  hand guiding holds still) and CI runs it on both backends.
- `docs/what_broke.md` written (draft); README "Two dynamics backends" and a CPU-load item
  under "Where it fails".
- Computed-torque law on the tracking controller (`control_law: pid_ff | computed_torque`,
  `ct_omega`, `ct_zeta`; runtime-switchable, either backend), `test_computed_torque`
  (closed loop against an exact-model plant), and a computed-torque leg in the smoke test.
- `bench_tracking` runs PID, PID + RNEA FF and computed torque (`--variants`) and stamps the
  dynamics backend; `make_results.py` renders whichever variants the CSV holds.
- `TcpJogController` (mode `jog`): TCP twist on `~/twist` → damped least squares with
  manipulability-scheduled damping, joint speed / acceleration / position limits as uniform
  scalings, 0.1 s deadman, PID + RNEA joint loop on the integrated reference, `~/jog_state`
  with sim-time stamps; `test_dls`, mock-hardware tests, `bench_jog` and a jog leg in the smoke
  test.

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

### Changed

- A runtime payload change no longer modifies the model inside `update()`: a model with the new
  payload is built in the parameter callback and swapped into the realtime loop through a
  `RealtimeThreadSafeBox` (needed for KDL, whose solvers are rebuilt; same path for Pinocchio).
- `regenerate_all.sh` takes the machine's heavy, build and gazebo locks, in that order, and
  regenerates the timing table first.

### Fixed

- CI: `ur5e_teaching_bench` declares pytest through `extras_require`; the container's
  setuptools ignores `tests_require`, so colcon found no tests and CI failed.
- `bench_camera.sdf` is well-formed XML (a `--` inside a comment).
- The controller polls in `sim_session.sh` and `ci_smoke.sh` are bounded by `timeout 10`: an
  orphaned poll once outlived its script and kept the machine-wide locks it had inherited.
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
