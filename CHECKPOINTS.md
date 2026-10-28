# CHECKPOINTS — ur5e-teaching-controllers

## CHECKPOINT 1 — plumbing + sim boots headless (2026-10-28 03:48 IST)

Wall-clock spent so far: 0 h 23 min (session start 03:25 IST).

### git status

```text
$ git status --short
D  src/.gitkeep
?? MORNING.md
```

(both are committed with this checkpoint.)

### Plumbing acceptance checklist

- [ ] **docker build . succeeds locally** — replaced by the MACHINE ADDENDUM with "the GitHub
  Actions job builds the image from the Dockerfile and is green". Docker is not installed here.
- [ ] **CI run on GitHub is green** — **BLOCKED (billing)**. Every run ends in ~3 s with
  *"The job was not started because recent account payments have failed or your spending limit
  needs to be increased."* See MORNING.md → WAITING FOR GIRI #1. Run:
  <https://github.com/giridhar-shenoy/ur5e-teaching-controllers/actions/runs/40591497563>
- [x] **pre-commit run --all-files passes**

  ```text
  $ .venv/bin/pre-commit run --all-files
  check for added large files..............................................Passed
  check for merge conflicts................................................Passed
  fix end of files.........................................................Passed
  trim trailing whitespace.................................................Passed
  check xml................................................................Passed
  ruff.....................................................................Passed
  ruff-format..............................................................Passed
  clang-format.........................................(no files to check)Skipped
  yamllint.................................................................Passed
  markdownlint-cli2........................................................Passed
  ```

- [x] **LICENSE contains my name; required files exist**

  ```text
  $ grep "Copyright" LICENSE
  Copyright (c) 2026 Giridhar J Shenoy
  $ ls THIRD_PARTY.md NOTES.md CHANGELOG.md docs/architecture.md docs/results.md scripts/regenerate_all.sh
  CHANGELOG.md
  NOTES.md
  THIRD_PARTY.md
  docs/architecture.md
  docs/results.md
  scripts/regenerate_all.sh
  ```

- [x] **Conventional commits, none dated before today**

  ```text
  $ git log --format="%h %ad %s" --date=iso
  cd641e0 2026-10-28 03:47:05 +0530 test: add headless sim smoke test and run it in CI
  afa1c67 2026-10-28 03:47:05 +0530 feat(bringup): add UR5e teaching xacro, bench worlds and sim launch
  316b247 2026-10-28 03:30:54 +0530 ci: build the Docker image in the job and run build/lint/test in it
  4daa9c9 2026-10-28 03:30:54 +0530 chore: add pre-commit with ruff, clang-format, yamllint, markdownlint
  df00db9 2026-10-28 03:30:54 +0530 build: add Lyrical Dockerfile and devcontainer
  caa7416 2026-10-28 03:30:54 +0530 chore: add repository skeleton
  ```

- [x] **GitHub repo**: `gh repo create ur5e-teaching-controllers --private --source . --push`
  → <https://github.com/giridhar-shenoy/ur5e-teaching-controllers> (PRIVATE), description set,
  topics: robotics, ros2, ros2-control, gazebo, manipulation, control-systems. No branch
  protection (not approved tonight).

### Feature step 2 — sim boots headless, robot moves (local; CI blocked)

Stock bringup (`ur_sim_control.launch.py`) on Lyrical failed twice; both are documented in
`sim.launch.py` and NOTES.md:

1. `joint_trajectory_controller` → `Invalid value set during initialization for parameter
   'joints': Length of parameter 'joints' is '0'` (no params file reaches the controller node).
2. With (1) worked around: the arm stalled at -150 N·m on shoulder_lift with ODE trimesh contact
   overflows. Gazebo log: `Created entity [10] named [ur]` and `Created entity [39] named [ur_0]`,
   i.e. the robot was spawned twice by `ros_gz_sim create` (first request timed out while
   Gazebo booted, then both were delivered).

`scripts/ci_smoke.sh` therefore targets this repo's `sim.launch.py` (same building blocks, both
fixed). Output:

```text
$ flock ~/work/.locks/gazebo.lock timeout 240 env SMOKE_TARGET=teaching scripts/ci_smoke.sh
[smoke] launching: ros2 launch ur5e_teaching_bringup sim.launch.py gazebo_gui:=false launch_rviz:=false initial_joint_controller:=joint_trajectory_controller
[smoke] waiting for joint_state_broadcaster and joint_trajectory_controller to be active
joint_trajectory_controller     joint_trajectory_controller/JointTrajectoryController         active
joint_state_broadcaster         joint_state_broadcaster/JointStateBroadcaster                 active
[smoke] controllers active after 10 s
[smoke] positions before: ... array('d', [1.5700000000011773, -1.5699999999997418, -1.2328038878802412e-20, -1.5699999999970788, -1.570000000000004, -7.74712548144043e-20]) ---
[smoke] sending FollowJointTrajectory goal 0.4,-1.2,0.6,-1.57,0.0,0.0
    error_code: 0
error_string: Goal successfully reached!
[smoke] positions after:  ... array('d', [0.6000000000019173, -1.1999999999994226, 0.3999999999999995, -1.569999999995985, -2.5209465252817068e-12, -4.2040260202560043e-19]) ---
[smoke] max joint displacement: 1.570 rad (threshold 0.2 rad)
[smoke] PASS in 17 s
[smoke] stopping launch process group 129568
real    0m21.980s
$ pgrep -af '[b]in/gz sim'; echo $?
1          # no Gazebo left running
```

### MUST list remaining

xacro/world runtime verification (F/T state interfaces, ApplyLinkWrench), Pinocchio wrapper,
controllers (a) and (b), gtests, mode manager + mode.launch.py, bench_tracking, bench_handguide,
GIFs, README/docs, CI green (blocked on billing).

### Does 1 Nov hold?

Yes for everything except "CI green", which depends on the billing fix (WAITING FOR GIRI #1).

## CHECKPOINT 2 — controllers (a) + (b) load, switch and move the arm headless (2026-10-28 04:50 IST)

Wall-clock spent so far: 1 h 25 min. Evidence below is from runs at 04:00–04:30 in this session
(commits `f0fa401` controllers, `a54aa0d` mode manager / drivers).

### git status

```text
$ git status --short
 M NOTES.md
?? results/            (tracking benchmark writing results/tracking.csv right now)
```

### Unit and integration tests (colcon test)

```text
$ colcon test && colcon test-result
Summary: 38 tests, 0 errors, 0 failures, 0 skipped
test_trapezoidal: tests="10" failures="0"
test_control_laws: tests="9" failures="0"
test_pinocchio_model: tests="11" failures="0"
test_load_controllers: tests="4" failures="0"     # 25/25 repeated runs pass
```

34 gtest cases (≥ 12 required): profiles (10), limits + PID anti-windup (9), Pinocchio wrapper
(11), controller_manager + mock hardware (4: load/configure both plugins, hand-guiding output ==
Pinocchio g(q) on mock_components/GenericSystem, tracking controller activates and holds, the two
effort controllers conflict and a strict hand-over succeeds). The other 4 of the 38 are
ament/pytest wrappers.

Bug found and fixed on the way: `test_load_controllers` segfaulted in ~50 % of runs. gdb showed
the crash in the test's own subscription lambda (captured locals used after the test body
returned while the executor thread still delivered messages) — a harness bug, not controller
code. Fixed by stopping the executor before reading/destroying the captures.

### F/T sensor reaches ros2_control (no ros_gz_bridge fallback needed)

```text
$ ros2 control list_hardware_interfaces | grep tcp_fts
        tcp_fts_sensor/force.x
        tcp_fts_sensor/force.y
        ... (force.z, torque.x/y/z)
$ ros2 topic echo --once /force_torque_sensor_broadcaster/wrench
header: {stamp: {sec: 13, nanosec: 327000000}, frame_id: wrist_3_link}
wrench: {force: {x: 0.00285, y: -0.00285, ...}}
```

### Mode switching (mode.launch.py, strict)

```text
$ ros2 launch ur5e_teaching_bringup mode.launch.py mode:=track
[mode_manager]: mode 'track': activated pid_ff_trajectory_controller, deactivated ['joint_trajectory_controller']; active now: ['force_torque_sensor_broadcaster', 'joint_state_broadcaster', 'pid_ff_trajectory_controller']
$ ros2 run ur5e_teaching_bench mode_manager --ros-args -p mode:=handguide
[mode_manager]: mode 'handguide': activated gravity_comp_handguide_controller, deactivated ['pid_ff_trajectory_controller']; active now: ['force_torque_sensor_broadcaster', 'gravity_comp_handguide_controller', 'joint_state_broadcaster']
```

### (a) tracking controller moves the arm on the effort interface

```text
# FF switched off at runtime -> the arm sags until the integrator carries gravity (proves the
# torques are really applied):
$ ros2 param set /pid_ff_trajectory_controller feedforward false; probe 3 s
1501 msgs over sim 3.00s (rate 500 Hz)
max|err| [0.     0.0045 0.0111 0.0157 0.     0.    ]
# FF on, 2-waypoint trajectory (0.5 rad moves, 2 s per segment):
2442 msgs over sim 4.89s (rate 499 Hz)
max|err| [0.0004 0.0003 0.0004 0.0008 0.0003 0.0004]
max|qd|  [0.3755 0.2782 0.2779 0.2792 0.2797 0.3755]
```

### (b) hand guiding floats the arm and the speed cap works

```text
# model payload = actual (0 kg), no push, 3 s:
max|qd| [0.0002 0.0026 0.0041 0.0037 0.0001 0.    ]
# 20 N persistent push on wrist_3_link (ApplyLinkWrench via ros_gz_bridge):
[gravity_comp_handguide_controller]: speed cap: |qd| of wrist_1_joint is 2.042 rad/s > 2.000 rad/s; deactivating
[controller_manager]: Deactivating controllers : [ gravity_comp_handguide_controller ] as their update resulted in an error!
```

(Wrist damping was raised afterwards, `[.., 2.0, 1.0, 0.05]`, so pushes move the arm rather
than spin the wrist.)

### Finding: effort -> position switch drops the arm (gz_ros2_control 3.0.9)

```text
JTC start    q [-0.   -1.57  1.57 -1.57 -1.57 -0.  ] eff [  0.   -20.28 -20.24  -1.82   0.     0.  ]
mode 'track': activated pid_ff_trajectory_controller
after track  q [ 0.   -1.57  1.57 -1.57 -1.57 -0.  ] eff [  0.   -20.28 -20.24  -1.82   0.    -0.  ]
mode 'position': activated joint_trajectory_controller
JTC +1s      q [ 0.381 -0.535  2.628 -3.66  -3.126 -0.235] eff [ 0.  0. -0. -0. -0. -0.]
```

Workaround: no switch back to position; homing uses the effort tracking controller.

### MUST list remaining

Benchmarks running (tracking, then hand-guiding), results tables and figures, GIFs, final
README numbers, CHANGELOG, checkpoint 3/4, CI green (blocked on billing).

### Does 1 Nov hold?

Yes, except "CI green", which needs the billing fix.

## CHECKPOINT 3 — the two CSVs exist and the numbers are plausible (2026-10-28 05:50 IST)

Wall-clock spent so far: 2 h 25 min.

### git status

```text
$ git status --short
 M CHECKPOINTS.md
(only this report is uncommitted; HEAD f29a1c1)
```

### CSVs

```text
$ wc -l results/*.csv
   10 results/handguide.csv
  109 results/tracking.csv
  119 total
$ cut -d, -f2-5 results/tracking.csv | sort -u
e73a5866a475,0.001,500,1
git_commit,sim_step_s,update_rate_hz,seed
$ cut -d, -f2-5 results/handguide.csv | sort -u
f2bcf5f5e9d5,0.001,500,1
git_commit,sim_step_s,update_rate_hz,seed
```

### Summary tables (generated by `python3 scripts/make_results.py`; `--check` passes)

**Joint-space tracking** (PidFfTrajectoryController, 3-segment 6-joint path; worst joint per repetition, then mean ± std over repetitions):

| Speed (× nominal) | PID: worst-joint RMSE (mrad) | PID + RNEA FF: worst-joint RMSE (mrad) | PID: worst max \|e\| (mrad) | PID + RNEA FF: worst max \|e\| (mrad) | Regenerate with |
|---|---|---|---|---|---|
| 0.5 | 5.34 ± 0.01 | 0.07 ± 0.00 | 11.94 ± 0.03 | 0.49 ± 0.00 | `scripts/bench_tracking.sh` |
| 1 | 8.82 ± 0.01 | 0.10 ± 0.07 | 20.68 ± 0.16 | 2.37 ± 2.44 | `scripts/bench_tracking.sh` |
| 2 | 10.70 ± 0.03 | 0.18 ± 0.06 | 26.08 ± 0.14 | 2.49 ± 1.02 | `scripts/bench_tracking.sh` |

**Hand guiding with a wrong payload model** (GravityCompHandGuidingController, model payload fixed, actual Gazebo payload varied; mean ± std over repetitions):

| Model / actual payload (kg) | Payload error (kg) | TCP drift speed at 1 s (mm/s) | TCP drift in 1 s (mm) | Effort to hold, F_z (N) | Expected (Δm·g, N) | Regenerate with |
|---|---|---|---|---|---|---|
| 1 / 0 | -1 | 374.40 ± 1.96 | 295.94 ± 4.14 | -9.81 ± 0.00 | -9.81 | `scripts/bench_handguide.sh` |
| 1 / 1 | +0 | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.00 | `scripts/bench_handguide.sh` |
| 1 / 2 | +1 | 201.94 ± 2.38 | 201.04 ± 3.53 | 9.81 ± 0.00 | 9.81 | `scripts/bench_handguide.sh` |

### Plausibility

- Tracking: PID-only worst-joint RMSE grows with speed (5.3 → 8.8 → 10.7 mrad); with RNEA
  feedforward it is 50–100× smaller (0.07–0.18 mrad) — expected, since Gazebo's dynamics are
  the same URDF the feedforward uses. The 1× and 2× FF rows have a large max |e| spread
  (2.4 ± 2.4 mrad): one repetition per speed has a short transient; the RMSE is unaffected.
- Hand guiding: the hold force equals Δm·g (−9.81 / 0 / +9.81 N) — Gazebo and Pinocchio share
  URDF and gravity, so this confirms the payload model is in the right place and the hand-over
  is seamless. Drift with a 1 kg error is large (0.20–0.37 m/s after 1 s) even with the raised
  damping: joint damping alone does not make a payload-model error safe.

### How these numbers changed tonight (all in NOTES.md)

1. Gravity: SDF default 9.8 vs Pinocchio 9.81 → set 9.81 in the worlds (drift at Δm = 0 went
   from 4 mm/s to ~0).
2. Damping [8, 8, 5, 2, 1, 0.05] let a 1 kg error carry the arm to the floor in < 2 s → raised to
   [25, 25, 15, 3, 1.5, 0.05]; drift window 2 s → 1 s.
3. Hold-force method: release-acceleration fit broke down with realistic damping → static
   (converged PID torque vs g_model).
4. Two real bugs found by the benchmark: zero torque on deactivation (one-cycle drop at every
   hand-over) and controller_state samples leaking between recordings. Both fixed.

### MUST list remaining

Final full regeneration at HEAD (running), checkpoint 4, MORNING STATUS. CI green is blocked on
billing (MORNING #1).

### Does 1 Nov hold?

Yes, apart from CI (needs the billing fix).

## CHECKPOINT 4 — final (2026-10-28 06:00 IST)

Wall-clock spent: 2 h 35 min (03:25–06:00 IST). HEAD before this report: `3f3d9bc`.

### git status

```text
$ git status --short
 M CHECKPOINTS.md      (this report)
$ git log --format="%ad" --date=short | sort -u
2026-10-28
$ git log --oneline | wc -l
28
```

### MUST list (SPRINT OVERRIDE) — status with evidence

- [x] **Plumbing; stock sim boots headless; ci_smoke.sh** — skeleton, Dockerfile, devcontainer,
  CI, pre-commit. Stock `ur_sim_control.launch.py` fails on Lyrical (documented); our
  `sim.launch.py` boots headless and `ci_smoke.sh` passes locally:

  ```text
  [smoke] controllers active after 10 s
  [smoke] max joint displacement: 1.570 rad (threshold 0.2 rad)
  [smoke] mode track: active = force_torque_sensor_broadcaster joint_state_broadcaster pid_ff_trajectory_controller
  [smoke] mode handguide: active = force_torque_sensor_broadcaster gravity_comp_handguide_controller joint_state_broadcaster
  [smoke] PASS in 23 s
  ```

- [x] **xacro overlay + bench.sdf** — effort/velocity/position interfaces, wrist F/T as a
  ros2_control `<sensor>` (no bridge fallback needed: six `tcp_fts_sensor/*` state interfaces
  listed), Gazebo-only payload link (`payload_mass`), ApplyLinkWrench bridged from ROS.
- [x] **Controller (a) PidFfTrajectoryController** — PID + anti-windup + Pinocchio RNEA
  feedforward, trapezoidal profiles, topic input only.
- [x] **Controller (b) GravityCompHandGuidingController** — runtime `payload_mass`/`payload_com`,
  viscous damping, joint-limit repulsion, speed cap (+ fallback to tracking, verified).
- [x] **Mode manager + mode.launch.py (track, handguide)**, and `sim.launch.py mode:=track`:

  ```text
  $ ros2 launch ur5e_teaching_bringup sim.launch.py gazebo_gui:=false launch_rviz:=false mode:=track
  [check] pid_ff_trajectory_controller active after 11 s
  [check] after:  array('d', [1.1999999972467708, -1.1999999984036531, 0.500000003757626, -1.1999999954038774, -1.1999999940462274, 0.5000000037083117])
  [check] total 22 s
  ```

- [x] **bench_tracking** — 3 speeds × FF on/off × 3 reps; per-joint RMSE and max |e|, mean ± std.
- [x] **bench_handguide** — model 1.0 kg vs actual 0 / 1 / 2 kg × 3 reps; drift at zero push and
  effort to hold vs payload error.
- [x] **GIFs via scripts/record_gifs.sh (≤ 3 MB)** — headless rendering worked (SHARED.md), no
  RViz/xvfb fallback needed:

  ```text
  $ ls -l docs/assets/*.gif
  1055472 docs/assets/hero.gif
  410597 docs/assets/mode_handguide.gif
  719647 docs/assets/mode_track.gif
  ```

- [x] **README, docs/results.md, docs/architecture.md (Mermaid), docs/what_broke.md (DRAFT,
  547 words, no invented numbers)** — `docs/controllers.md` too.
- [x] **colcon build + colcon test (≥ 12 gtest cases)** — locally:

  ```text
  $ colcon test && colcon test-result
  Summary: 45 tests, 0 errors, 0 failures, 0 skipped
  test_trapezoidal: tests="11" failures="0"
  test_control_laws: tests="11" failures="0"
  test_pinocchio_model: tests="12" failures="0"
  test_load_controllers: tests="7" failures="0"      # 41 gtest cases
  $ python3 scripts/make_results.py --check
  results up to date
  $ .venv/bin/pre-commit run --all-files            # all hooks Passed
  ```

- [ ] **CI green on GitHub** — BLOCKED: the account's Actions billing/spending limit stops every
  job before it starts (MORNING.md #1). The Docker image build is therefore also unverified
  (no Docker on this laptop).

### Other acceptance lines

- `scripts/regenerate_all.sh` end to end: 489 s, 438 s and 436 s wall in three full runs
  (< 60 min). Final run at clean commit c048632; every CSV row carries that hash and seed 1.
- No hard-coded UR5e dynamics: `grep -rnE "8\.058|3\.761|0\.425|0\.392|2\.846" src
  --include=*.cpp --include=*.hpp | wc -l` → 0; all from robot_description via Pinocchio.
- THIRD_PARTY.md: `.clang-format` only; no third-party code was copied.

### What is left on the MUST list

Only "CI green", which needs Giri (billing). SHOULD (TcpJogController) not started — see
NOTES.md open question 6.

### Does 1 Nov hold?

Yes. Everything on the MUST list is done and verified locally; CI turns green as soon as the
billing issue is fixed, unless the image build or a container-only difference fails, which
nobody can see before then.

## v0.2 CHECKPOINT 1 — CI green on main (2026-10-29 02:40 IST)

Wall-clock spent: 0 h 22 min (session start 02:18 IST). The repo is public
(`gh repo view --json visibility` → `"visibility":"PUBLIC"`), so Actions runs for the first time.

### CI runs

| Run | Commit | Result |
|---|---|---|
| [40766406049](https://github.com/giridhar-shenoy/ur5e-teaching-controllers/actions/runs/40766406049) | `933fae5` | failure in `colcon test`: `ur5e_teaching_bench` "NO TESTS RAN" (exit 5) |
| [40768154969](https://github.com/giridhar-shenoy/ur5e-teaching-controllers/actions/runs/40768154969) | `ca71285` fix(bench): declare pytest as an extras_require test dependency | **success**, 6 min 9 s |

```text
$ gh run view 40768154969 --json conclusion,jobs --jq '.conclusion, (.jobs[0].steps[] | "\(.name) \(.conclusion)")'
success
Set up job success
Checkout success
Set up Python (pre-commit) success
Lint (pre-commit) success
Set up Docker Buildx success
Build image (apt layers cached in the GHA cache) success
Start container success
rosdep install success
colcon build success
colcon test success
Results tables match the CSVs success
Smoke test (Gazebo server-only, physics only) success
...
$ grep "Summary: 48\|smoke\] PASS" <run log>
colcon test   Summary: 48 tests, 0 errors, 0 failures, 0 skipped
Smoke test    [smoke] PASS in 23 s
```

- The Docker image (`FROM ros:lyrical-ros-base`) builds on the runner: 6 min 46 s cold,
  2 min 34 s with the GHA layer cache. This was the v0.1 item that could not be checked without Docker.
- No test was skipped or weakened; the only change was how the Python package declares pytest.
- Branch protection: not enabled (waiting for Giri, see the checkpoint message).

## v0.2 CHECKPOINT 2 — KDL backend builds, both controllers run on KDL in the sim, equivalence passes (2026-10-29 03:08 IST)

Wall-clock spent: 0 h 50 min (session start 02:18 IST). Overnight mode: not waiting here.

### git status (before this checkpoint's commit)

```text
$ git status --short
 M CHANGELOG.md
 M NOTES.md
$ git log --oneline 98fee7e..HEAD
58b205f docs: write what_broke.md from this repository's findings (draft)
ce9360e docs: document the dynamics interface, both backends and the payload swap
81d0045 data: publish the dynamics timing and the two-backend README section
844830e test(smoke): exercise both effort controllers in the sim, on both backends
96365ef feat(bench): time gravity() and rnea() on both backends; refuse busy machines
adfd712 feat(controllers): choose the dynamics backend per controller, swap payloads outside the loop
8e6917c fix(bringup): make bench_camera.sdf well-formed XML
af0002b feat(dynamics): add an orocos KDL backend behind a common dynamics interface
```

### Checklist

- [x] **KDL backend builds** (`liborocos-kdl-dev` 1.5.2, `ros-lyrical-kdl-parser` 3.0.1,
  `ros-lyrical-urdf` 2.13.2, all already installed; no new dependency)

  ```text
  $ flock ~/work/.locks/build.lock scripts/build.sh
  Finished <<< ur5e_teaching_bench [20.1s]
  Finished <<< ur5e_teaching_controllers [4min 4s]
  Finished <<< ur5e_teaching_bringup [5.31s]
  ```

- [x] **Equivalence test passes, tolerances as briefed (1e-6 N·m, 1e-6, 1e-9), nothing loosened**

  ```text
  $ ./build/ur5e_teaching_controllers/test_dynamics_equivalence
  [equivalence] payload 0.0 kg: max |pinocchio - kdl| gravity 2.13163e-14 N m, rnea 2.13163e-14 N m, nle 2.13163e-14 N m, M 3.55271e-15, J 3.33067e-16, p 2.22045e-16 m, R 4.44089e-16
  [equivalence] payload 1.0 kg: max |pinocchio - kdl| gravity 2.13163e-14 N m, rnea 4.26326e-14 N m, nle 2.84217e-14 N m, M 4.44089e-15, J 3.33067e-16, p 2.22045e-16 m, R 4.44089e-16
  [equivalence] payload 1.0 kg off-axis on ft_frame: max |pinocchio - kdl| gravity 1.42109e-14 N m, rnea 2.84217e-14 N m, nle 2.84217e-14 N m, M 3.55271e-15, J 3.33067e-16, p 2.22045e-16 m, R 4.44089e-16
  [equivalence] gravity_z -9.8: max |pinocchio - kdl| gravity 1.42109e-14 N m, rnea 2.84217e-14 N m, nle 2.84217e-14 N m, M 3.55271e-15, J 3.33067e-16, p 2.22045e-16 m, R 4.44089e-16
  [  PASSED  ] 9 tests.
  ```

  The backends agreed on the first build; there was no frame/axis/inertia bug to fix. The
  conventions that had to line up are listed in NOTES.md ("KDL backend").

- [x] **Both controllers run with `dynamics_backend:=kdl` in the sim** (short Gazebo check under
  the gazebo lock; the smoke test now drives both effort controllers)

  ```text
  $ flock ~/work/.locks/gazebo.lock timeout 300 env SMOKE_BACKEND=kdl scripts/ci_smoke.sh
  [smoke] controllers active after 28 s
  [smoke] max joint displacement: 1.570 rad (threshold 0.2 rad)
  [smoke] mode track: active = force_torque_sensor_broadcaster joint_state_broadcaster pid_ff_trajectory_controller
  [smoke] effort tracking: trajectory to 0.0,-1.57,1.57,-1.57,-1.57,0.0 in 3 s
  [smoke] effort tracking: max |q - goal| = 6.40e-08 rad (tolerance 0.01)
  [smoke] mode handguide: active = force_torque_sensor_broadcaster gravity_comp_handguide_controller joint_state_broadcaster
  [smoke] hand guiding: max joint drift over 1 s = 0.00e+00 rad (tolerance 0.005)
  [smoke] effort controllers configured with the kdl backend: 2
  [smoke] PASS in 54 s
  launch.log: [pid_ff_trajectory_controller]: dynamics backend: kdl, gravity_z -9.8100 m/s^2
  launch.log: [gravity_comp_handguide_controller]: dynamics backend: kdl, gravity_z -9.8100 m/s^2
  $ pgrep -af '[b]in/gz sim' | grep r1 || echo "no r1 gz left"
  no r1 gz left
  ```

  Same script with the default backend: PASS in 41 s. CI now runs both.

- [x] **Full test suite**

  ```text
  $ colcon test && colcon test-result
  Summary: 67 tests, 0 errors, 0 failures, 0 skipped
  ```

- [x] **Timing CSV** (MUST 3, done early): `results/dynamics_timing.csv` at `96365efa478c`; see
  docs/results.md. Run under heavy → build → gazebo locks, guard found no heavy process.

### What is left

- Push and confirm CI green with the new KDL smoke step.
- SHOULD: computed-torque control law on controller (a), sim check (CHECKPOINT 3), the
  3 speeds × {pid, pid_ff, computed_torque} × 3 reps rerun (CHECKPOINT 4).
- TcpJogController only after that.

### Does Sunday 1 Nov hold?

Yes. The MUST list is done except CI confirming the new commits; the SHOULD is next tonight.

## v0.2 CHECKPOINT 3 — timing CSV exists; computed torque tracks a trajectory in the sim (2026-10-29 03:36 IST)

Wall-clock spent: 1 h 18 min (session start 02:18 IST).

### git status (before this checkpoint's commit)

```text
$ git status --short
 M NOTES.md
 M docs/controllers.md
$ git log --oneline 8949478..HEAD
f91e559 test(smoke): drive the tracking controller with computed torque as well
781e63f feat(bench): run the tracking benchmark as pid / pid_ff / computed_torque
6c17e77 feat(controllers): add a computed-torque law to the tracking controller
```

CI for checkpoint 2 (`8949478`):
[40773777144](https://github.com/giridhar-shenoy/ur5e-teaching-controllers/actions/runs/40773777144)
— success, including the new step "Smoke test, KDL dynamics backend".

### Checklist

- [x] **Timing CSV exists**

  ```text
  $ head -3 results/dynamics_timing.csv | cut -c1-140
  bench,git_commit,build_type,cpu,seed,calls,backend,function,median_us,p95_us,mean_us,max_us,median_cycle_fraction,p95_cycle_fraction
  dynamics_timing,96365efa478c,RelWithDebInfo,"11th Gen Intel(R) Core(TM) i5-1135G7 @ 2.40GHz",1,100000,none,clock,0.0160,0.0170,...
  ```

  Pinocchio g / RNEA 0.538 / 0.887 µs median, KDL 1.678 / 1.702 µs; worst p95 0.1077 % of 2 ms.

- [x] **Computed torque runs on both backends (unit level)**

  ```text
  $ ./build/ur5e_teaching_controllers/test_computed_torque
  [computed_torque] pinocchio: worst deviation from the continuous-time response 0.0147057 x |e0|
  [computed_torque] kdl: worst deviation from the continuous-time response 0.0147057 x |e0|
  [computed_torque] pinocchio: max |e| on the sinusoid 9.49433e-05 rad
  [computed_torque] kdl: max |e| on the sinusoid 9.49433e-05 rad
  [  PASSED  ] 9 tests.
  $ colcon test --packages-select ur5e_teaching_controllers && colcon test-result
  Summary: 72 tests, 0 errors, 0 failures, 0 skipped
  ```

- [x] **Computed torque tracks a trajectory in the sim** (short Gazebo checks under the gazebo
  lock; not published)

  ```text
  $ REPS=1 OUT=/tmp/r1_ct_check.csv scripts/bench_tracking.sh --speeds 1.0 --variants pid_ff computed_torque
  [bench_tracking]: rep 0 speed 1.0 pid_ff: max RMSE 1.75e-04 rad, max |e| 2.88e-03 rad
  [bench_tracking]: rep 0 speed 1.0 computed_torque: max RMSE 3.86e-04 rad, max |e| 5.46e-03 rad
  [bench_tracking] done in 52 s wall -> /tmp/r1_ct_check.csv
  $ SMOKE_BACKEND=pinocchio scripts/ci_smoke.sh   (and SMOKE_BACKEND=kdl)
  [smoke] computed torque: max |q - goal| = 0.00e+00 rad (tolerance 0.01)     (both backends)
  [smoke] PASS in 66 s / PASS in 45 s
  $ pgrep -af '[b]in/gz sim' | grep r1 || echo "no r1 gz left"
  no r1 gz left
  ```

  Finding: on the benchmark path computed torque is ~2.2× worse than PID + feedforward on every
  joint at the same nominal bandwidth, with exact models. The integral term is not the reason
  (PID + FF with ki = 0: 0.132 / 0.95 mrad). An offline exact-plant replica shows both laws
  ~10× better than Gazebo and CT more delay-sensitive. The remaining cause is not identified;
  NOTES.md "Computed torque" has the runs. Not tuned away: the published comparison uses the
  shipped gains.

### What is left

- The 3 × 3 × 3 tracking rerun (approved, ~7–15 min): queued behind R4's training run, which
  holds the heavy/build/gazebo locks (started 03:31).
- README / results.md three-column tables (generated after the rerun), CHECKPOINT 4.
- TcpJogController (allowed: computed torque is done before Friday evening). Running bench_jog
  for published numbers is not on tonight's approved list; it will go to WAITING FOR GIRI.

### Does Sunday 1 Nov hold?

Yes.
