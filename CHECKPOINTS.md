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
