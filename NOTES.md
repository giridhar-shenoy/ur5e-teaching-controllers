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
  Resolved 28 Oct: history rewritten to `giridharjshenoy@gmail.com` and force-pushed.

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
- 03:30 skeleton committed and pushed (private repo). CI blocked by billing (MORNING.md #1).
- 03:33–03:40 stock bringup: two failures (no controller params on Lyrical; double spawn race).
- 03:47 own sim.launch.py; ci_smoke.sh PASS in 17 s locally. CHECKPOINT 1 at 03:48.

### Stock ur_sim_control.launch.py on Lyrical — findings (verified 2026-10-28)

1. ros2_control 6.10.1 controller nodes only receive parameters through the controller manager
   parameter `<controller>.params_file` (set by `spawner --param-file` or in the CM YAML). The
   stock UR launch sets neither → `joint_trajectory_controller` fails with
   "Length of parameter 'joints' is '0'". `strings libcontroller_manager.so | grep params_file`
   → `{}.params_file`. Fix used: inject `params_file: <yaml itself>` per controller
   (`scripts/lib/inject_params_file.py`, same logic inside `sim.launch.py`).
2. The stock spawner for the inactive branch still passes `--stopped`; the 6.10.1 spawner only
   knows `--inactive` (from `ros2 run controller_manager spawner -h`).
3. Duplicate spawn: `ros_gz_sim create` WARN "Waiting for service [/world/empty/create]",
   then Gazebo logged `Created entity [10] named [ur]` and `Created entity [39] named [ur_0]`.
   Two robots overlap (ODE trimesh-trimesh overflow, -150 N·m stall) and two
   `/controller_manager` nodes exist. Gazebo needed ~20 s to boot on the loaded laptop.
   Fix: wait for `/world/bench/create` in `gz service -l`, then spawn once with
   `-allow_renaming false`.
4. `gz_ros_control` warns "Desired controller update period (0.002 s) is slower than the gazebo
   simulation period (0.001 s)" → update_rate 500 Hz on a 1 ms step is honoured as a 2-step
   decimation (this is the intended setup, not an error).

### Commands that worked

- Build: `flock ~/work/.locks/build.lock scripts/build.sh`
- Smoke: `flock ~/work/.locks/gazebo.lock timeout 240 scripts/ci_smoke.sh`
- No Gazebo left: `pgrep -af '[b]in/gz sim'` (the bracket stops pgrep matching its own shell).
- 04:10 INCIDENT: the first `sim_session.sh stop` killed only the setsid/flock group; GNU
  `timeout` had moved the launch into its own process group, so Gazebo kept running while the
  lock was released. R2's queued benchmark then started its own Gazebo; both ran together for
  ~1 min until I killed my session (`pkill -s <sid>`). Fixed: stop now signals timeout's group,
  waits for it, and only then lets flock exit. Checked with `ps -eo pid,pgid,sid,args`.
- 04:25 FINDING (gz_ros2_control 3.0.9): switching a joint from the effort interface back to the
  position interface (pid_ff_trajectory_controller -> joint_trajectory_controller) leaves it
  uncommanded: /joint_states effort drops to 0 and the arm falls onto the ground within 1 s
  (`mode_manager mode:=position`, then /joint_states: q [0.381 -0.535 2.628 -3.66 -3.126 -0.235],
  effort all 0). position -> effort works. Root cause not verified (no source in the binary
  package). Workaround: never go back to position; homing uses the effort tracking controller;
  `position` mode removed from mode_manager. Listed under "Where it fails".
- 04:20 bench_tracking first validation: FF-on run after an FF-off run gave 1.1 rad RMSE —
  traced (raw dump) to the effort->position homing above, not to the controller. FF-on alone:
  max RMSE 0.13 mrad, max |e| 1.9 mrad at 2x speed.
- 04:30–04:45 R2 holds the Gazebo lock (bench_planners.sh, timeout 3600 s); wrote docs, results
  tooling and GIF pipeline meanwhile. sim_session.sh now waits for the lock without a deadline.
- 04:45–04:53 first full `scripts/regenerate_all.sh`: exit 0 in **489 s wall** (tracking 284 s,
  hand-guiding 99 s, GIFs ~100 s). Numbers (commit a1a3c35, not kept): tracking worst-joint RMSE
  PID 5.3 / 8.8 / 10.7 mrad and PID+FF 0.05–0.27 mrad at 0.5 / 1 / 2x; hand-guiding hold force
  Fz -10.57 / -0.55 / +8.99 N for payload error -1 / 0 / +1 kg (expected -9.81 / 0 / +9.81).
  => a constant ~ -0.6 N offset (arm accelerates upward a little even with a perfect payload
  model; 4 mm/s drift at 2 s). Candidate 1: SDF default gravity is 9.8, Pinocchio uses 9.81
  (my estimate says this explains only ~0.1 N; fixed anyway by setting <gravity> in the worlds).
  GIF review: payload cube floated 2 cm under the flange (CoM 5 cm, cube 6 cm) -> payload CoM
  default 0.03 m everywhere; 12 N demo pushes threw the arm to 0.5 m/s -> 5 N pushes.
- 04:55–05:10 independent read-only review of the controller code (sub-agent). Fixed:
  non-blocking `try_update_params` + whole-set size validation of runtime params (a 1-element
  `kp` was read out of bounds), NaN/limit validation of trajectories, stale-start check on
  adoption (`max_start_deviation`), trajectories kept alive by the callback so update() never
  frees them, positive effort limits required. Documented, not fixed: pre-emption at speed steps
  qd_des to 0 (rest-to-rest profiles). Added `fallback_controllers: [pid_ff_trajectory_controller]`
  for the hand-guiding controller (ros2_control 6.10.1 supports it: `strings libcontroller_manager.so
  | grep fallback`). New tests -> 41 gtest cases; colcon test 45/45.
- 05:10 fallback verified in sim: cap lowered to 0.3 rad/s at runtime, 15 N push ->
  `speed cap: |qd| of wrist_1_joint is 0.385 rad/s > 0.300 rad/s; deactivating` ->
  `Activating fallback controllers : [ pid_ff_trajectory_controller ]`, arm held at
  q [0, -1.565, 1.578, -1.568, -1.57, 0]. A 25 N push with the default cap did not trip it.
- Lock contention: sim starts waited 162 s and 461 s for the shared Gazebo lock (R2 benchmarks).
- 05:20 second full regen (commit e73a586, 438 s wall). Gravity 9.81 cut zero-error drift from
  4 mm/s to 0.3 mm/s. But hand-guiding with damping [8, 8, 5, 2, 1, 0.05]: TCP moved
  0.99 m (Δm = -1 kg) and 0.50 m (Δm = +1 kg) in 2 s; the +1 kg arm ended on the floor, so its
  "speed at 2 s" (4.4 mm/s) measured a collision, not drift. Hold force was fine
  (-10.51 / -0.49 / +9.15 N vs -9.81 / 0 / +9.81). Raised damping to [25, 25, 15, 3, 1.5, 0.05]
  (D*dt/M < 1 kept) and re-ran hand guiding + GIFs only; tracking CSV stays at e73a586.
- 05:30 hand-guiding re-run with damping [25, 25, 15, 3, 1.5, 0.05] (commit c5c3498): drift at 2 s
  0.28 / 0.0000 / 0.12 m/s and displacement 0.60 / 0.003 / 0.36 m for Δm -1 / 0 / +1 kg; the
  release-acceleration hold force degraded to -9.6 / -0.96 / +7.9 N because with this damping the
  wrist joints reach terminal speed inside the 40 ms fit window. Replaced by a static measurement
  (converged PID hold torque minus hand-guiding g_model, through J^-T) and drift at 1 s
  (commit 93b62f7). First attempt failed on my bug (set the tracker's params before loading it,
  fixed in the next commit — which also carries the 10 N GIF pushes, not mentioned in its message).
- Lesson for the README: joint damping alone does not make a payload-model error safe; even
  3x damping lets 1 kg of error carry the TCP tens of centimetres in 2 s.
- 05:35 the static hold force showed a constant -2.4 N offset while the Δm = 0 arm did not drift
  at all — contradictory. Raw dump (`/tmp/hg_debug.py`): at the first hand-guiding sample the
  elbow already moved at 0.052 rad/s although the arm was still under the tracking controller.
  Cause: `on_deactivate()` wrote zero effort and the hardware applied it for one 2 ms cycle
  before the next controller's first update (25 N·m / 0.88 kg m² × 2 ms ≈ 0.057 rad/s). Fixed in
  f2bcf5f: deactivation leaves the last torque; the speed-cap trip writes gravity + damping.
  After the fix: hg[0] torque == hold torque, qd = 0; hold force -9.81 / 0.00 / +9.81 N.
  Also fixed: both controller_state subscriptions fed one recording, so the outgoing
  controller's samples leaked into the next recording (gave exactly 0 N).
- 05:42 final hand-guiding CSV (commit f2bcf5f): drift at 1 s 374 / 0 / 202 mm/s, displacement
  296 / 0 / 201 mm for Δm -1 / 0 / +1 kg. Tracking CSV unchanged (commit e73a586).

### Time log (continued)

- 05:53 final run at HEAD c048632 (clean): ci_smoke PASS 23 s (incl. track/handguide mode
  check), `regenerate_all.sh` **436 s wall** (tracking 273 s, hand guiding 108 s, GIFs). All CSV
  rows stamped c048632. Run-to-run: tracking FF worst-joint RMSE at 1x was 0.10 ± 0.07 mrad
  (commit e73a586 run) vs 0.05 ± 0.02 (final); PID-only 8.82 vs 8.83 mrad. Hand guiding identical
  to the f2bcf5f run to the printed precision.
- 05:55 acceptance: `ros2 launch ur5e_teaching_bringup sim.launch.py gazebo_gui:=false
  launch_rviz:=false mode:=track` -> pid_ff_trajectory_controller active after 11 s, trajectory
  followed, 22 s total (< 120 s).
- 06:05 CHECKPOINT 4 written; MORNING.md STATUS written; pushed. No Gazebo or launch process
  of this session left running (`ps -eo pid,args | grep r1/install` → none).

### Acceptance status (feature prompt, v0.1 scope) — verified tonight

| Item | Status | Evidence |
|---|---|---|
| CI green on GitHub | BLOCKED (billing) | MORNING.md #1; local equivalents below |
| colcon build | done | `scripts/build.sh` (all 3 packages) |
| colcon test, ≥ 12 gtest | done: 41 gtest cases, 45/45 colcon tests | `colcon test && colcon test-result` |
| ci_smoke.sh | done locally (17 s; + mode switching section) | CHECKPOINTS.md |
| rosdep for CI | `rosdep check --from-paths src --ignore-src` → all satisfied | |
| Docker image builds | UNVERIFIED (no Docker here; CI blocked) | |
| sim.launch.py mode:=track | see CHECKPOINT 4 | |
| mode.launch.py track / handguide | done | CHECKPOINT 2, ci_smoke mode section |
| results/tracking.csv, handguide.csv | done, 3 reps, commit + seed stamped | CHECKPOINT 3 |
| GIFs ≤ 3 MB | done: 0.73 / 0.42 / 1.07 MB | record_gifs.sh output |
| regenerate_all.sh < 60 min | done: 489 s and 438 s wall (two full runs) | NOTES above |
| No hard-coded UR5e dynamics | done: all from robot_description via Pinocchio | `grep` of src: no masses/lengths |
| THIRD_PARTY.md | only `.clang-format`; no code copied (sim.launch.py re-implements the stock launch's structure, see below) | |

Note on THIRD_PARTY: `sim.launch.py` follows the structure of ur_simulation_gz's BSD-3
`ur_sim_control.launch.py` (same nodes in the same roles) but is written fresh; no lines were
copied, so it has no THIRD_PARTY entry. If you prefer to credit it anyway, add a row.

## 2026-10-28 midday — results re-stamped after the history rewrite

The history was rewritten to change the commit e-mail, so `scripts/regenerate_all.sh` was re-run
to stamp the results with a commit that exists. Three full runs of the tracking benchmark, RNEA-FF
worst-joint RMSE at 1× (mean ± std over 3 reps):

| Run | Machine load | FF RMSE (mrad) | FF max error (mrad) | PID-only RMSE (mrad) |
|---|---|---|---|---|
| overnight final | alone | 0.05 ± 0.02 | 0.87 ± 0.16 | 8.83 ± 0.01 |
| midday run 1 | R3 benchmark running at the same time | 0.13 ± 0.04 | 1.80 ± 1.38 | 8.82 ± 0.01 |
| midday run 2 (committed) | alone, 437 s wall | 0.04 ± 0.03 | 1.22 ± 0.47 | 8.83 ± 0.01 |

Hand-guiding rows were unchanged within their std in all three. The feedforward result degrades
about 3× under CPU contention (likely late controller cycles against wall-clock-timed inputs);
worth a line in the README's "Where it fails" and a check of the controller's cycle timing.

## Session 2 — 2026-10-29 (v0.2 sprint, started 02:18 IST)

Environment: `export ROS_DOMAIN_ID=11 GZ_PARTITION=r1 MAKEFLAGS=-j4`; builds under
`flock ~/work/.locks/build.lock`, Gazebo under `flock ~/work/.locks/gazebo.lock timeout <s>`.
The repo is public (`gh repo view` → `"visibility":"PUBLIC"`), so Actions minutes are free and
the first real CI run (40766406049) started at 20:47 UTC on 28 Oct.

### Plan (SPRINT OVERRIDE v0.2)

0. CI green: watch run 40766406049, read the logs, fix Dockerfile / ci.yml until green, record
   every fix here. → CHECKPOINT 1.
1. Abstract `dynamics::DynamicsModel` interface (gravity, rnea, nonlinear_effects, mass_matrix,
   tip_jacobian, tip pose, URDF limits, payload) that `PinocchioModel` implements unchanged in
   maths; factory `make_dynamics_model(backend, urdf, joints, tip, gravity_z)`.
2. `KdlModel`: kdl_parser tree → chain `base_link → tool0`; ChainIdSolver_RNE (RNEA),
   ChainDynParam (gravity, mass, Coriolis), ChainJntToJacSolver (Jacobian), FK for the tip pose;
   gravity and Jacobian expressed in the URDF root frame like Pinocchio's; payload = last
   segment inertia + cube payload, solvers rebuilt.
3. Controllers (a) and (b): read-only params `dynamics_backend: pinocchio | kdl` (default
   pinocchio) and `gravity_z` (default -9.81). Payload changes no longer touch the model inside
   update(): the parameter listener's user callback (executor thread) builds a new model with the
   new payload and hands it over through a `realtime_tools::RealtimeThreadSafeBox`; update() only
   swaps a shared_ptr; old models are freed outside the loop.
4. `test_dynamics_equivalence` (gtest): 200 random (q, qd, qdd) within limits × payload {0, 1} kg;
   gravity and RNEA ≤ 1e-6 N m, M ≤ 1e-6, J ≤ 1e-9. Plus the existing Pinocchio identity tests
   re-run on KDL, and the mock-hardware controller test with `dynamics_backend: kdl`.
5. `bench_dynamics_timing` (C++ executable, no ROS graph): 100 000 gravity() and rnea() calls per
   backend on pre-generated random states, per-call steady_clock timing, median / p95 µs and the
   fraction of the 2 ms cycle; CSV `results/dynamics_timing.csv`; `scripts/bench_dynamics_timing.sh`
   refuses to run while gz sim / colcon / training is running. CI: 1 000-call smoke only.
6. Launch: `dynamics_backend:=kdl` on sim.launch.py / mode.launch.py; gravity_z in controllers.yaml
   with a test that it equals the worlds' `<gravity>`. → CHECKPOINT 2 (ask before Gazebo).
7. Docs: README "Two dynamics backends" + timing row + CPU-load "Where it fails" item + "v0.1
   numbers were produced with the Pinocchio backend"; docs/results.md timing table;
   docs/what_broke.md rewrite (DRAFT, 400–700 words); regenerate_all.sh refuses to start when
   another heavy process runs; architecture.md / controllers.md backend notes; CHANGELOG.
8. SHOULD: computed-torque law on controller (a) (`control_law: pid_ff | computed_torque`),
   sim check → CHECKPOINT 3; bench_tracking 3 × 3 × 3 rerun (ask first, ~15 min) → CHECKPOINT 4.
9. TcpJogController only if 8 is done by Friday evening.

### Step 0 — CI on GitHub (first runs that actually execute)

| Run | Result | Cause → fix |
|---|---|---|
| [40766406049](https://github.com/giridhar-shenoy/ur5e-teaching-controllers/actions/runs/40766406049) | lint ✓, image build ✓ (6 min 46 s, cold cache), rosdep ✓, colcon build ✓ (2 min 13 s), **colcon test ✗** | `ur5e_teaching_bench`: "Ran 0 tests … NO TESTS RAN", exit 5. The container's setuptools warns `Unknown distribution option: 'tests_require'` and colcon there no longer sees pytest as a test dependency, so it falls back to unittest, which finds none of the pytest-style tests. Locally (colcon-core 0.21.3) the same setup.py worked. Fix: declare `extras_require={"test": ["pytest"]}`, which colcon-core maps to test dependencies (`colcon_core/package_augmentation/python.py`, `extras_mapping`); locally `colcon test --packages-select ur5e_teaching_bench` → 3 tests, 0 failures. |
| [40768154969](https://github.com/giridhar-shenoy/ur5e-teaching-controllers/actions/runs/40768154969) | **success** (6 min 9 s; image 2 min 34 s from the layer cache, colcon build 2 min 4 s, `Summary: 48 tests, 0 errors, 0 failures, 0 skipped`, `[smoke] PASS in 23 s`) | — |

Step 0 took 22 min wall (02:18–02:40 IST); one fix. CI is green at `ca71285`.

## CHANGELOG entry for this session

See CHANGELOG.md `[Unreleased]` (Added: skeleton, bringup, two controllers, Pinocchio wrapper,
41 gtests, mode manager, two benchmarks, GIF pipeline, docs; Fixed: stock-launch workarounds,
deactivation torque drop, RT-safe parameters and trajectory hand-off, gravity mismatch,
hand-guiding damping and fallback).

## Open questions for Giri

1. CI: fix the Actions billing/spending limit, or make the repo public when ready (MORNING #1)?
2. Commit author e-mail: resolved 28 Oct (history rewritten to `giridharjshenoy@gmail.com`).
3. README "What transfers to a real UR": is "the stock driver gives position/velocity, not
   torque" the claim you want, given newer PolyScope torque features?
4. Hand-guiding damping [25, 25, 15, 3, 1.5, 0.05] is my choice (drift vs push effort).
5. SHOULD (TcpJogController + bench_jog) was not started: the MUST's "CI green" item is not
   green (billing), and the sprint rule says SHOULD only after MUST is green. Start it once CI
   runs?
6. Do you want the stock `ur_sim_control.launch.py` failures (no params on Lyrical, double
   spawn) reported upstream to Universal_Robots_ROS2_GZ_Simulation? I did not open any issue.
7. gz_ros2_control 3.0.9 effort -> position switch leaves joints uncommanded (arm falls): worth
   an upstream issue once you have checked it against the current source?
