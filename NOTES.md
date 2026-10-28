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

### KDL backend (MUST 1) — decisions and verified facts

- Versions: `liborocos-kdl-dev` 1.5.2-1, `ros-lyrical-kdl-parser` 3.0.1, `ros-lyrical-urdf` 2.13.2
  (`apt-cache policy`, all installed). CMake: `find_package(orocos_kdl_vendor)` provides the
  imported target `orocos-kdl`; `kdl_parser::kdl_parser`, `urdf::urdf`.
- The brief says "the existing dynamics wrapper interface (the one pinocchio_model implements)";
  v0.1 had a concrete `PinocchioModel` class and no interface, so I extracted
  `dynamics::DynamicsModel` from its public API (same method names and semantics) and made
  `PinocchioModel` implement it. Pinocchio maths unchanged; its gravity is now set from
  `gravity_z` (default -9.81, the same double as Pinocchio's built-in `gravity981`).
- KDL frame conventions, checked by the equivalence test rather than assumed:
  - chain `base_link -> tool0` (`tree.getChain`); `base_link` hangs off the URDF root `world`
    through a fixed joint, so the wrapper composes root→base_link and rotates gravity into
    base_link and the Jacobian and tip pose out of it. With the shipped URDF that rotation is
    the identity; the code handles a rotated mount anyway.
  - `ChainJntToJacSolver`: reference point at the chain tip, base orientation = Pinocchio's
    `LOCAL_WORLD_ALIGNED`; rows [linear; angular] in both.
  - `RigidBodyInertia(m, c, Ic)` takes the rotational inertia **about the CoM**; a segment's
    inertia is expressed in its tip frame, so the payload is added to the last segment (tool0)
    with its CoM in tool0 coordinates. Pinocchio adds it to the wrist_3 joint body through the
    frame placement. Same physics.
  - KDL solvers keep a `const Chain&`: copying a `KdlModel` has to rebuild its solvers on the new
    chain (custom copy constructor), and `set_payload()` rebuilds them after `setInertia`.
  - kdl_parser does not carry joint limits; they come from `urdf::Model` (same URDF fields
    Pinocchio reads); the test asserts the limit vectors are identical.
- `test_dynamics_equivalence` result (first build, nothing to fix; 200 states, seed 20261029,
  q in the URDF limits, qd in the URDF velocity limits, |qdd| ≤ 10 rad/s²):

  | Case | gravity (N m) | RNEA (N m) | C·qd+g (N m) | M | J | tip p (m) |
  |---|---|---|---|---|---|---|
  | payload 0 kg | 2.1e-14 | 2.1e-14 | 2.1e-14 | 3.6e-15 | 3.3e-16 | 2.2e-16 |
  | payload 1 kg, CoM (0, 0, 0.03) | 2.1e-14 | 4.3e-14 | 2.8e-14 | 4.4e-15 | 3.3e-16 | 2.2e-16 |
  | 1 kg off-axis CoM on the rotated ft_frame | 1.4e-14 | 2.8e-14 | 2.8e-14 | 3.6e-15 | 3.3e-16 | 2.2e-16 |
  | gravity_z = -9.8 | 1.4e-14 | 2.8e-14 | 2.8e-14 | 3.6e-15 | 3.3e-16 | 2.2e-16 |

  (max |Pinocchio − KDL| over all samples and joints; tolerances 1e-6 / 1e-6 / 1e-9 as briefed.)
  Both are exact O(n) recursions in double precision on the same inertias, so agreement at
  rounding level is what to expect; a convention bug would show up at 1e-2 to 1e+1.

### bench_dynamics_timing (MUST 3) — runs

- 02:54 published run (`results/dynamics_timing.csv`, commit `96365efa478c`, RelWithDebInfo,
  i5-1135G7): taken under heavy → build → gazebo locks; `scripts/lib/heavy_processes.sh` found
  nothing; load average at start 5.69 (other sessions' claude agents, VS Code's cpptools at 38 %
  of one thread, a browser at ~26 %). 1 s wall for 4 × 100 000 timed calls.
- Two extra runs right after, same commit, same locks, to see the spread (not published,
  `/tmp/r1_timing_rerun{2,3}.csv`). Median / p95 in µs:

  | Call | published run | rerun 2 | rerun 3 |
  |---|---|---|---|
  | clock (two reads) | 0.016 / 0.017 | 0.016 / 0.018 | 0.015 / 0.016 |
  | Pinocchio g(q) | 0.538 / 0.740 | 0.508 / 0.580 | 0.511 / 0.684 |
  | Pinocchio RNEA | 0.887 / 1.123 | 0.794 / 1.004 | 0.798 / 1.165 |
  | KDL g(q) | 1.678 / 1.748 | 1.479 / 2.085 | 1.524 / 1.627 |
  | KDL RNEA | 1.702 / 2.155 | 1.483 / 1.725 | 1.551 / 2.112 |

  Medians move by up to ~12 % between runs and p95 more; the ordering (Pinocchio g < Pinocchio
  RNEA < KDL g ≈ KDL RNEA) and "worst p95 ≤ 0.11 % of 2 ms" hold in all three. The max per row
  (5–142 µs) is scheduler/interrupt noise on a non-RT kernel, not the libraries.
- Why Pinocchio is faster (reasoning, not measured separately): its `computeGeneralizedGravity`
  is a dedicated pass and RNEA is header-only template code compiled into our wrapper at -O2
  with fixed-size spatial algebra; KDL's `JntToGravity` runs the full RNE with zero q̇/q̈ through
  its precompiled library, and the wrapper copies into `KDL::JntArray`s.
- The functional check before the bench existed (2 000 calls on the test URDF at 02:50, while
  R2 compiled) gave ~1.5 µs for every call and 52 ns for the clock: CPU contention and
  frequency state dominate at that size, which is why the guard exists.

### Time log (session 2)

- 02:18 start; 02:40 CHECKPOINT 1 (CI green at `ca71285`).
- 02:42 overnight mode (Giri asleep): branch protection was not approved → MORNING.md.
- 02:44 KDL backend + controller wiring committed (`af0002b`, `adfd712`); 67/67 tests.
- 02:54 timing run (1 s wall); 03:00–03:06 smoke on both backends in Gazebo (gazebo lock,
  41–89 s each; a slow 63 s bringup once while R2 compiled).
- 03:08 CHECKPOINT 2.

### Computed torque (SHOULD 1) — implementation notes and what the sim showed

- Law: `tau = RNEA(q, qd, qdd_des + Kp e + Kd e_dot)`, Kp = w², Kd = 2ζw, per joint, URDF
  torque limits, no integral; shipped w = [20, 20, 20, 30, 30, 30] rad/s, ζ = 1 (the PID's
  design bandwidth from the controllers.yaml comment). `control_law` is runtime-switchable so
  one sim session can run all three benchmark variants; switching resets the PID integrators.
- Unit test bug I made and fixed before committing: the sinusoid test compared q_des(t) with the
  plant state after the step (t + 2 ms), which alone is 2.4 mrad at 1.2 rad/s. Measured at the
  sampling instant (as the controller and bench_tracking do) it is 9.5e-5 rad, the same as a
  Python replica (`/tmp/r1_ct_diag.py`).
- 03:13 short sim check (1 rep, 1×, Pinocchio, `/tmp/r1_ct_check.csv`, not published):
  worst-joint RMSE / max |e| pid_ff 0.175 / 2.88 mrad, computed_torque 0.386 / 5.46 mrad.
  CT is ~2.2× worse on every joint. Checked, in order:
  1. Integral term? PID + FF with ki = 0 (`/tmp/r1_noki.csv`): 0.132 / 0.95 mrad, i.e. better
     still. Not the integral.
  2. Implementation? Offline replica of the same path and gains on an exact Pinocchio plant with
     a 2 ms hold and 1 ms substeps (`/tmp/r1_ct_path.py`): PD+FF 0.007–0.020, CT 0.009–0.028 mrad
     RMSE per joint. Both laws behave as expected and CT is only 1.2–2× PD+FF there.
  3. Delay sensitivity? Same replica with 1 or 2 ms extra actuation delay: CT degrades faster
     than PD+FF (ratio grows to 2–3×), because CT feeds the measured state back through the
     whole inverse dynamics. Plausible part of the story, but the replica's errors stay 3–10×
     below Gazebo's for both laws, so the main error source in Gazebo is something the replica
     does not model (how DART and gz_ros2_control apply effort and report state). **Not
     identified**; stopped there (time box).

### Tracking rerun (SHOULD 2) — log

- 03:26 queued under heavy → build → gazebo; R4's ACT training held all three locks until
  ~03:32. 03:32 attempt 1 got the locks (guard found no heavy process; load average 25 at that
  moment, the lagging average of R4's training).
- 03:33 **aborted attempt 1 before it recorded anything**: I had started editing `src/` for
  TcpJogController while the run was queued, so its commit stamp would have read `-dirty` and
  the symlinked launch file it used carried uncommitted (unused) lines. Stopped it with
  `sim_session.sh stop` (no gz left), restored `results/tracking.csv` from git (the bench
  script deletes it at start), stashed the TcpJog work (`git stash`, "tcp-jog WIP"), and
  restarted on the clean tree at `d5c2a2b`. The runner now refuses to start on a dirty tree.
  Lesson: no edits under src/ or scripts/ while a stamped benchmark is queued or running.

- **03:34 INCIDENT (mine): two orphaned processes from the aborted attempt hold all three
  machine locks.** When I stopped attempt 1 (`kill -TERM` on `sim_session.sh start` and
  `bench_tracking.sh`, then `sim_session.sh stop`), the start script's controller poll
  `ros2 control list_controllers | grep -Eq ...` survived as orphans (PIDs 148857, 148858;
  PGID 136328 / SID 136324 = my first runner; cwd ~/work/r1; GZ_PARTITION=r1,
  ROS_DOMAIN_ID=11; started 03:33:03). They inherited the heavy, build and gazebo lock file
  descriptors from the flock chain, and with the CLI's default `service_timeout=0`
  (`controller_manager_services.service_caller`: `while not cli.service_is_ready(): wait 10 s`)
  they wait for a controller manager that no longer exists, forever. My `kill` and `kill -INT`
  of them were refused by the session's permission classifier ("interfere with workloads"), and
  TaskStop no longer knows the finished task, so I stopped and asked (MORNING.md B).
  Effect: every session's heavy/build/Gazebo work is blocked, including my clean rerun (queued:
  it starts by itself once the locks are free, and refuses a dirty tree).
  Fix to make after the rerun: poll with `timeout 10 ros2 control list_controllers` in
  `sim_session.sh` (and `ci_smoke.sh`), so no poll can outlive its parent by more than 10 s.

- ~03:39 the orphans were gone (not by me; I did not get permission to end them) and the queued
  clean rerun ran at once: guard quiet, load average 1.21, 393 s wall, exit 0,
  `results/tracking.csv` 162 rows, stamp `6bf35f463513` (a docs-only commit on top of
  `d5c2a2b`; same src/scripts), backend pinocchio. Backup: `/tmp/r1_tracking_v02_rerun.csv`.
- Rerun numbers, worst-joint RMSE at 1× per rep: PID 8.81 / 8.82 / 8.81 mrad (v0.1: 8.83),
  PID + FF 0.233 / 0.180 / 0.215 mrad (v0.1: 0.04 ± 0.03), computed torque 0.236 / 0.165 /
  0.235 mrad. Two consequences:
  1. On a quiet machine computed torque ≈ PID + FF. The 2.2× gap at CHECKPOINT 3 came from a
     check made while other sessions were compiling: the CPU-load sensitivity again, and
     stronger for CT. The README / controllers.md text that says "CT tracks worse" is wrong
     and must be corrected from the table.
  2. PID + FF is 4–7× worse than v0.1 on every joint at every speed, proportional to each
     joint's path amplitude (timing-like); PID-only is unchanged except wrist_3 (2×). Every
     PID + FF run tonight is ≥ 0.13 mrad. Code diff of the pid_ff path shows nothing that
     should change it. A/B in progress: v0.1 (`c35e70c`) built in a worktree at
     `/tmp/r1_v01`, to be run with its own bench_tracking.sh under the same locks.
- Not committed yet: the rerun CSV and its tables (waiting for the A/B), TcpJogController and
  bench_jog (written, not yet built: R4's training holds the build lock 03:46–~04:31), the
  `timeout 10` poll fix in sim_session.sh / ci_smoke.sh.

- Verified (headers + strings of the 3.0.9 binary): gz_ros2_control's plugin implements
  `ISystemPreUpdate` / `ISystemPostUpdate`, i.e. the controller manager's read/update/write run
  synchronously inside Gazebo's step; ROS messaging is on a `MultiThreadedExecutor` thread. So
  CPU load cannot change the control loop's timing per sim step; whatever makes the feedforward
  rows load- and run-dependent enters through the ROS side (when trajectories / parameters are
  adopted, what is published and recorded) or through the physics itself. Not resolved.

- **04:20 A/B: the v0.1 code reproduces tonight's level, not its own published numbers.**
  `git worktree add --detach /tmp/r1_v01 c35e70c`, built there, its own
  `scripts/bench_tracking.sh` run under heavy → build → gazebo (guard quiet, load 3.80, 287 s
  wall, `/tmp/r1_v01_tracking.csv`). Worst-joint RMSE in mrad, mean ± std over 3 reps:

  | Run | PID 0.5× | PID 1× | PID 2× | PID+FF 0.5× | PID+FF 1× | PID+FF 2× | wrist_3 PID 1× |
  |---|---|---|---|---|---|---|---|
  | v0.1 published (`c35e70c`, 28 Oct midday) | 5.346 ± 0.006 | 8.828 ± 0.007 | 10.700 ± 0.036 | 0.027 ± 0.010 | 0.045 ± 0.028 | 0.141 ± 0.047 | 0.106 |
  | v0.1 code re-run 29 Oct 04:20 | 5.338 ± 0.005 | 8.809 ± 0.007 | 10.680 ± 0.042 | 0.094 ± 0.018 | 0.242 ± 0.008 | 0.335 ± 0.046 | 0.228 |
  | v0.2 rerun 29 Oct 03:39 (`6bf35f4`, published now) | 5.336 ± 0.002 | 8.812 ± 0.006 | 10.680 ± 0.044 | 0.085 ± 0.005 | 0.209 ± 0.027 | 0.368 ± 0.087 | 0.216 |

  So no v0.2 regression: same code, same packages (`/var/log/dpkg.log`: the only install after
  the v0.1 runs is an unrelated desktop app, `t3code`, at 13:58 on 28 Oct), several times
  worse feedforward rows a day later. What differs is the machine's state (tonight: up to five
  other agent sessions, their ACT training runs between ours, VS Code's cpptools indexer, a
  browser; CPU governor `powersave`, platform profile `balanced`). The guard catches named
  heavy processes, not this. Published: the v0.2 three-column table (approved), with the
  README and results.md saying the two tables are not comparable row by row. The worktree is
  left in place (`git worktree list`); `git worktree remove /tmp/r1_v01` when no longer needed.

- 04:45 build with TcpJogController: 87 tests, 0 failures. 04:47 CHECKPOINT 4. Gazebo checks
  of the jog controller queued behind R2's chain (holds the gazebo lock since 04:45).

- 04:53 CI [40786279797](https://github.com/giridhar-shenoy/ur5e-teaching-controllers/actions/runs/40786279797)
  (checkpoint 4) **failed** in "Smoke test, KDL dynamics backend": the tracking controller never
  logged "trajectory accepted" and the arm stayed at the previous goal (|q − goal| 1.57 rad);
  the Pinocchio smoke in the same job passed, as did the KDL smoke locally three times. Cause
  (most likely): `ros2 topic pub --once` exits 0.1 s after publishing and the reliable message
  is lost on a loaded runner. Fix `fix(smoke)`: `--keep-alive 2`, the failed check prints the
  publisher's output; checks and tolerances unchanged. Re-run: 40787316271.

- 05:05 CI 40787316271 failed again, now in the Pinocchio smoke: `ros2 topic pub -w 1` never
  matched the controller's subscription in 150 s (rc 124). Together with the previous failure
  (matched, message lost) this looks like DDS transport in the container, not the controllers.
  Fix: CI container with `--network host --ipc host` like the devcontainer (64 MB /dev/shm and
  the bridge network were the differences); smoke publishes bounded and diagnosed. Hypothesis,
  not proven.
- 05:04–05:07 local Gazebo checks of the jog work (gazebo lock): smoke pinocchio PASS 68 s, kdl
  PASS 66 s (jog moved 0.31 rad, then ≤ 2e-6 rad after the deadman); bench_jog REPS=1 to
  `/tmp/r1_jog_check.csv` (not published): latency to 50 % / 90 % of 5 cm/s 17–18 / 27–31 ms,
  tracking RMS 1.8–5.3 % of the command, stop 184–229 ms after the last command (0.1 s of it
  is the deadman), min manipulability 0.059 (no damping active).

- 05:18 CI [40788849695](https://github.com/giridhar-shenoy/ur5e-teaching-controllers/actions/runs/40788849695)
  green at `e72fa6d` with host network/IPC: 87 tests, smoke PASS pinocchio 64 s, kdl 55 s,
  jog leg included. 05:20 closing docs (MORNING STATUS, these notes, README jog example).
  Session total: 3 h 02 min wall (02:18–05:20), about 1 h of it waiting for locks.

## CHANGELOG entry for this session

Session 1 (28 Oct, v0.1): see CHANGELOG.md `[Unreleased]` (Added: skeleton, bringup, two
controllers, Pinocchio wrapper, 41 gtests, mode manager, two benchmarks, GIF pipeline, docs;
Fixed: stock-launch workarounds, deactivation torque drop, RT-safe parameters and trajectory
hand-off, gravity mismatch, hand-guiding damping and fallback).

Session 2 (29 Oct, v0.2), also in CHANGELOG.md `[Unreleased]`:

- Added: KDL dynamics backend behind `DynamicsModel`, `dynamics_backend` / `gravity_z`
  parameters, `test_dynamics_equivalence`, `bench_dynamics_timing` + results, computed-torque
  law + `test_computed_torque`, three-variant `bench_tracking` + rerun, `TcpJogController` +
  `test_dls` + `bench_jog`, smoke test driving every effort controller on both backends in CI,
  `heavy_processes.sh` guard, what_broke.md (draft).
- Changed: payload changes build a new model outside the realtime loop and swap it in;
  `regenerate_all.sh` takes heavy → build → gazebo locks and runs the timing first.
- Fixed: pytest declaration (CI), `bench_camera.sdf` XML comment, unbounded controller polls,
  one-shot smoke publishes, CI container network/IPC.

## Open questions for Giri

Session 2 (new):

1. Branch protection on `main` now that CI is green (MORNING A)?
2. Publish `bench_jog` (MORNING C)? It is ready; only tonight's approval list kept it out.
3. The README's "Two dynamics backends" paragraph says KDL's RNE solver is what you used on the
   production arms (your brief's wording). Keep it as written?
4. The feedforward tracking rows are machine-state dependent (A/B: same code 0.04 → 0.24 mrad
   a day apart). Publish them as a level (as now), or invest in finding the mechanism first?
5. `/tmp/r1_v01` is a git worktree of `c35e70c` I created for the A/B; remove it with
   `git worktree remove /tmp/r1_v01` when you no longer need it (I did not delete anything).

Session 1 (as written on 28 Oct; status on 29 Oct in brackets):

1. CI: fix the Actions billing/spending limit, or make the repo public when ready (MORNING #1)?
   [resolved: public, CI runs and is green]
2. Commit author e-mail: resolved 28 Oct (history rewritten to `giridharjshenoy@gmail.com`).
3. README "What transfers to a real UR": is "the stock driver gives position/velocity, not
   torque" the claim you want, given newer PolyScope torque features? [open]
4. Hand-guiding damping [25, 25, 15, 3, 1.5, 0.05] is my choice (drift vs push effort). [open]
5. SHOULD (TcpJogController + bench_jog) was not started: the MUST's "CI green" item is not
   green (billing), and the sprint rule says SHOULD only after MUST is green. Start it once CI
   runs? [done 29 Oct, `c66be22` `d2509d4`]
6. Do you want the stock `ur_sim_control.launch.py` failures (no params on Lyrical, double
   spawn) reported upstream to Universal_Robots_ROS2_GZ_Simulation? I did not open any issue.
   [open]
7. gz_ros2_control 3.0.9 effort -> position switch leaves joints uncommanded (arm falls): worth
   an upstream issue once you have checked it against the current source? [open]
