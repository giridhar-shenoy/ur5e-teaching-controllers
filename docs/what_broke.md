# What broke (DRAFT)

> DRAFT — written from this repository's NOTES.md and CHECKPOINTS.md (28–29 Oct 2026); to be
> edited before it is final.

Everything here happened in this repository while bringing up two effort controllers on a
simulated UR5e under ROS 2 Lyrical and Gazebo Jetty. Most of it was not in the controllers.

**The stock bringup did not work as shipped.** I meant to wrap `ur_sim_control.launch.py` from
ur_simulation_gz 2.5.0 and only swap in my URDF and controller YAML. It failed in two ways. Its
spawners pass no parameter file, and ros2_control 6.10.1 only hands parameters to a controller
through `<controller>.params_file`, so the stock trajectory controller died with "Length of
parameter 'joints' is '0'". (The inactive spawner also still passes `--stopped`, which the 6.10.1
spawner no longer knows.) Then, on a loaded laptop, `ros_gz_sim create` asked Gazebo for
`/world/<w>/create` before Gazebo served it: the first request timed out but was delivered, the
retry succeeded too, and with renaming hard-coded on I had two overlapping robots and two
controller managers. My `sim.launch.py` keeps the same building blocks, gives every controller a
`params_file`, and spawns once, after the create service exists.

**Going back from effort to position dropped the arm.** With gz_ros2_control 3.0.9, switching a
joint from my effort tracking controller back to the stock position controller left it
uncommanded: `/joint_states` effort went to zero and the arm fell onto the ground within a
second. Position to effort works. The binary package ships no source, so I have not found the
cause. I removed the way back: homing is done by the effort tracking controller itself.

**Gazebo and the model disagreed about gravity.** SDF's default gravity is 9.8 m/s²; Pinocchio
uses 9.81. In my first hand-guiding run the arm drifted at 4 mm/s even with a perfect payload
model. Setting `<gravity>0 0 -9.81</gravity>` in the worlds cut that to 0.3 mm/s (the hold-force
offset I also saw turned out to be the next bug). It is now one `gravity_z` parameter shared by both dynamics backends, with a test
that keeps it equal to the worlds.

**My controllers dropped the arm at every hand-over.** The static hold-force measurement showed
a −2.4 N offset while the arm with a correct payload model did not drift at all, which cannot
both be true. A raw dump explained it: at the first hand-guiding sample the elbow was already
moving at 0.052 rad/s. The outgoing controller wrote zero torque in `on_deactivate()`, and the
hardware applied that for one 2 ms cycle before the next controller's first update (25 N·m on
0.88 kg·m² for 2 ms is about 0.057 rad/s). Leaving the last torque on the interface fixed it.
The hold force then matched Δm·g exactly (−9.81 / 0.00 / +9.81 N).

**Joint damping did not make a payload error safe.** With my first damping values, 1 kg of
payload-model error carried the TCP 0.5 to 1.0 m in two seconds; when the real payload was the
heavier one the arm ended on the floor, so its "drift speed" was measuring a collision. Roughly
tripling the damping on the big joints helped, but the published table still shows the TCP
moving 201 to 296 mm in the first second after letting go. Damping trades push effort for
drift; it does not remove the error.

**The feedforward numbers depend on CPU load.** When I re-ran the tracking benchmark next to another benchmark on the same laptop, the worst-joint RMSE with RNEA feedforward at
1× went from 0.04 ± 0.03 to 0.13 ± 0.04 mrad, and the max error from 1.22 to 1.80 mrad. The
PID-only and hand-guiding rows did not move. I have not found the mechanism; the controller runs
inside the physics step, so I suspect the benchmark driver's wall-clock timing rather than the
control law. The regeneration script now refuses to start while another simulator, build or
training run is active.

**Smaller ones.** CI could not run at all until the repository was public (billing). Its first
real run failed because setuptools in the image ignores `tests_require`, so colcon ran the
Python tests under unittest and found none. And when I added a "reach the goal" check to the
smoke test, the first version compared positions in `/joint_states` order, which on this sim
starts with the elbow.

## Appendix: run logs

Copied from the build log (`NOTES.md`, not tracked since 30 Oct 2026), numbers unchanged.

**Tracking rerun on a quiet machine, 29 Oct 03:39** (stamp `6bf35f463513`, backend pinocchio,
load average 1.21, 393 s wall). Worst-joint RMSE at 1× per rep: PID 8.81 / 8.82 / 8.81 mrad
(v0.1: 8.83), PID + FF 0.233 / 0.180 / 0.215 mrad (v0.1: 0.04 ± 0.03), computed torque
0.236 / 0.165 / 0.235 mrad. On a quiet machine computed torque ≈ PID + FF; the 2.2× gap seen
earlier came from a check made while a build was running.

**A/B, 29 Oct 04:20: the v0.1 code reproduces the new level, not its own published numbers.**
The v0.1 commit (`c35e70c`) was built in a separate worktree and run with its own
`scripts/bench_tracking.sh` under the same locks (load 3.80, 287 s wall). Worst-joint RMSE in
mrad, mean ± std over 3 reps:

| Run | PID 0.5× | PID 1× | PID 2× | PID+FF 0.5× | PID+FF 1× | PID+FF 2× | wrist_3 PID 1× |
|---|---|---|---|---|---|---|---|
| v0.1 published (`c35e70c`, 28 Oct midday) | 5.346 ± 0.006 | 8.828 ± 0.007 | 10.700 ± 0.036 | 0.027 ± 0.010 | 0.045 ± 0.028 | 0.141 ± 0.047 | 0.106 |
| v0.1 code re-run 29 Oct 04:20 | 5.338 ± 0.005 | 8.809 ± 0.007 | 10.680 ± 0.042 | 0.094 ± 0.018 | 0.242 ± 0.008 | 0.335 ± 0.046 | 0.228 |
| v0.2 rerun 29 Oct 03:39 (`6bf35f4`, published now) | 5.336 ± 0.002 | 8.812 ± 0.006 | 10.680 ± 0.044 | 0.085 ± 0.005 | 0.209 ± 0.027 | 0.368 ± 0.087 | 0.216 |

**Dynamics timing spread, 29 Oct.** The published run (`results/dynamics_timing.csv`, commit
`96365efa478c`, RelWithDebInfo, i5-1135G7) and two extra runs right after it, same commit, same
locks (not published). Median / p95 in µs:

| Call | published run | rerun 2 | rerun 3 |
|---|---|---|---|
| clock (two reads) | 0.016 / 0.017 | 0.016 / 0.018 | 0.015 / 0.016 |
| Pinocchio g(q) | 0.538 / 0.740 | 0.508 / 0.580 | 0.511 / 0.684 |
| Pinocchio RNEA | 0.887 / 1.123 | 0.794 / 1.004 | 0.798 / 1.165 |
| KDL g(q) | 1.678 / 1.748 | 1.479 / 2.085 | 1.524 / 1.627 |
| KDL RNEA | 1.702 / 2.155 | 1.483 / 1.725 | 1.551 / 2.112 |

Medians move by up to ~12 % between runs and p95 more; the ordering (Pinocchio g < Pinocchio
RNEA < KDL g ≈ KDL RNEA) and "worst p95 ≤ 0.11 % of 2 ms" hold in all three.
