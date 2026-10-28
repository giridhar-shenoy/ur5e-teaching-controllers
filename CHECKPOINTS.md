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
  400fa48 2026-10-28 03:47:05 +0530 test: add headless sim smoke test and run it in CI
  2d6df0c 2026-10-28 03:47:05 +0530 feat(bringup): add UR5e teaching xacro, bench worlds and sim launch
  9e269d6 2026-10-28 03:30:54 +0530 ci: build the Docker image in the job and run build/lint/test in it
  0ca05be 2026-10-28 03:30:54 +0530 chore: add pre-commit with ruff, clang-format, yamllint, markdownlint
  53484a9 2026-10-28 03:30:54 +0530 build: add Lyrical Dockerfile and devcontainer
  616bc76 2026-10-28 03:30:54 +0530 chore: add repository skeleton
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
(commits `1aa8ff0` controllers, `975ac5b` mode manager / drivers).

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
(only this report is uncommitted; HEAD 524b35b)
```

### CSVs

```text
$ wc -l results/*.csv
   10 results/handguide.csv
  109 results/tracking.csv
  119 total
$ cut -d, -f2-5 results/tracking.csv | sort -u
7c938a6a6344,0.001,500,1
git_commit,sim_step_s,update_rate_hz,seed
$ cut -d, -f2-5 results/handguide.csv | sort -u
bd001f60c69b,0.001,500,1
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
