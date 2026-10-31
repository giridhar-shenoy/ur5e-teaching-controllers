# What broke (DRAFT)

> DRAFT — written from this repository's history (28–31 Oct 2026) and the review of 31 Oct; to
> be edited by the author before it is final. Numbers that no script in the repository
> regenerates are collected, with their date and source, in the
> [appendix](#appendix-hand-recorded-runs-not-regenerated-by-any-script).

Everything here happened in this repository while bringing up three effort controllers on a
simulated UR5e under ROS 2 Lyrical and Gazebo Jetty. Most of it was not in the controllers. The
first section is the finding that changes how the tracking numbers should be read; then the
thirteen failure items the README lists, in its groups and numbering; then the bugs that were
fixed along the way; then the appendix.

## Timing offset between reference and state

> Draft for the author to edit (first person).

For two days I explained the run-to-run changes in the feedforward tracking numbers with CPU
load. The runs I recorded do not support that, and a different explanation fits all of them.

Take one run of `bench_tracking` and divide each joint's RMSE by the RMS of that joint's own
reference velocity $\dot q_{des}$ over the same window. (The CSV does not store $\dot q_{des}$,
but it follows from the 1/3-rule trapezoid and the path in `bench_tracking.py`: a segment of
length $D$ and duration $T$ contributes $\int \dot q_{des}^2\,dt = \tfrac{5}{4} D^2/T$, and the
window is the three segments plus 0.5 s at rest. No segment was stretched: `profiled_s` equals
`requested_s` in every row.) The ratio has units of time. For PID + feedforward and for computed
torque it is the same on all six joints within a run, to a few per cent, and it changes from run
to run: see table A1 in the appendix for the v0.1 and 29 Oct runs, and
[results.md](results.md#run-to-run-spread) for the five runs of 31 Oct.

The six joints differ by about 10 000× in home-pose inertia (the diagonal of $M(q)$ goes from
2.59 kg m² on shoulder_lift to $2.6 \cdot 10^{-4}$ kg m² on wrist_3) and by 2400× in $K_p$
(1200 to 0.5 N·m/rad). A control error would scale with those. This one does not: it is
proportional to each joint's reference speed, with one constant shared by every joint, and the
constant is the same for two laws with different feedback gains. (All joints share the segment
times and the profile shape, so any error that is a fixed linear response to the reference
would scale with each joint's path; what points to a time offset is that the constant does not
depend on the feedback, whose bandwidth differs by a factor of two between the base and wrist
joints and between the two laws.) That is what a sub-millisecond offset in when $q_{des}$ and
$q$ are paired looks like: if $q$ is effectively sampled $\delta$ later than the time at which
$q_{des}$ is evaluated, then $e(t) = q_{des}(t) - q(t) \approx \dot q_{des}(t)\,\delta$ even for
perfect tracking. So the feedforward and computed-torque numbers mainly measure an offset in the
bench, not the control law, and the true feedforward error in this simulation is below the
published number.

For PID alone the same ratio differs by orders of magnitude between joints (appendix A1): PID's
error is a real control error, except on wrist_3, where it is small enough to sit on the same
floor ([F8](#f8-the-feedforward-error-is-set-by-a-timing-offset-between-reference-and-measured-state-not-by-the-control-law-and-not-by-cpu-load)).

Why I no longer believe the CPU-load story (appendix A2):

- The three v0.1 runs on 28 Oct looked like a load effect: two runs made alone gave a low
  feedforward number, one made next to another benchmark gave a higher one.
- The 29 Oct 03:39 rerun was made on a quiet machine and came out well above the quiet v0.1
  runs, with no change to the tracking law.
- In the 29 Oct 04:20 A/B the v0.1 code itself, at a clearly higher load average than the 03:39
  run, reproduced the 29 Oct level, not its own published number, and came out close to the
  quiet 03:39 run.
- The 31 Oct 07:00 regeneration of the same code as the 03:39 run started at a higher load
  average than that run and came out well below it; it is now the published single-run table in
  [results.md](results.md#tracking-full-tables).
- So load average does not order the runs, and the spread among quiet runs is larger than the
  load effect I claimed. The five runs of 31 Oct, each with its load average logged every 5 s,
  are in [results.md](results.md#run-to-run-spread).

Where the offset comes from I have not found. gz_ros2_control runs the controller manager's
read, update and write synchronously inside the Gazebo step (verified on the 3.0.9 binary), and
the controller computes $e = q_{des} - q$ itself, from the state it read and the reference it
sampled at the `time` it was given in the same `update()`. So the offset is inside one 2 ms
cycle, and the candidates are which 1 ms physics step the state comes from relative to that
`time`, where the trajectory's start time is latched (`t_start_` is set on the cycle that adopts
the trajectory), and how the zero-order hold over two physics steps lines up with both. The
measurement I would make first: log $q_{des}$ and $q$ with physics-step indices, cross-correlate
them to measure the lag, fix the stamping, and then publish the feedforward and computed-torque
numbers as a spread over at least five runs.

`scripts/regenerate_all.sh` still refuses to start next to a simulator, a build, a pip install
or a training run: the dynamics timing is a wall-clock measurement, and published numbers should
not depend on whatever else the machine is doing. That guard is hygiene, not the explanation.

## Model errors show up as drift

### F1. A wrong payload model makes the arm drift, and the operator pays for it

The hand-guiding table in [results.md](results.md#hand-guiding) shows the arm rising or sinking
when the model payload is off, and the vertical force needed to hold it still equals
$\Delta m\,g$. That force is equal to $\Delta m\,g$ by construction: Gazebo and the model share
the URDF and $g$, and the benchmark computes it as $J^{-T}(\tau_{hold} - \tau_{model})$, so it
confirms that the payload enters the model in the right place and measures nothing that could
fail. A test that could fail would need a model that differs from the plant: wrong inertia or
centre of mass, friction, a noisy F/T sensor, or the force felt during motion rather than at rest.

The drift after release is the measured part, and it depends on the damping. With my first
damping values a 1 kg payload-model error carried the TCP to the floor within two seconds, so
its "drift speed" was measuring a collision (appendix A12). Roughly tripling the damping on the
big joints helped, but the published table still shows the TCP moving tens of centimetres in the
first second after letting go. Damping trades push effort for drift; it does not remove the
error. The law and the payload handling are in
[controllers.md](controllers.md#hand-guiding).

### F2. Without feedforward, PID sags first and tracks late

The integrator has to build up the gravity torque before it can hold the arm (the benchmark
therefore starts PID-only runs from that steady state, after 3 s), and during motion the
inertial and Coriolis torques show up as tracking error. Compare the PID-only and PID + RNEA
columns in [results.md](results.md#tracking-full-tables). PID's worst joint is wrist_2 in every
run, and its error does not have the timing-offset signature: RMSE divided by RMS reference
velocity differs by orders of magnitude between joints (appendix A1).

### F3. Computed torque buys nothing over PID + feedforward here, and is more fragile

With exact models the two track about equally well; which one comes out ahead changes from run
to run, because both sit on the timing floor ([F8](#f8-the-feedforward-error-is-set-by-a-timing-offset-between-reference-and-measured-state-not-by-the-control-law-and-not-by-cpu-load)),
so this benchmark cannot rank them. They are also not compared at equal stiffness on the wrist
joints, where the PID's PD loops are faster than computed torque's 30 rad/s
([controllers.md](controllers.md#gains-and-bandwidths)).

"More fragile" rests on an argument and two observations no script reproduces. The argument:
computed torque feeds the measured state back through the whole inverse dynamics, so lag or
noise in that state reaches the torque directly, while PID + feedforward evaluates its model on
the reference. The observations: a one-repetition check made while builds were running, where
computed torque fell clearly behind PID + feedforward on every joint (appendix A7), and an
offline replica on an exact rigid-body plant where an added actuation delay hurt computed torque
more (appendix A8). On a real arm, where the joint velocities are estimated and
the model is wrong, that trade would need measuring again.

## Simulation is kinder than hardware

### F4. Gazebo's effort interface is kinder than a real arm's

No joint friction (the URDF joints have zero damping and friction), no gearbox compliance, no
motor current limits or torque ripple, perfect torque tracking, an exactly known model, exact
joint velocities (a real UR estimates them), no sensor noise, delay or latency model anywhere in
the bringup, and an ideal F/T sensor that no controller reads. The feedforward numbers here are
an upper bound on what a real UR5e would give.

What changes on a real UR. The UR driver exposes an effort command interface on PolyScope
≥ 5.23 / 10.10, so these controllers could run there in principle, but the robot's
`direct_torque` has to be called every control step or the robot reverts to position control.
That breaks this repository's hand-over design, which leaves the last torque on the interface
between controllers and relies on gz_ros2_control holding it
([controllers.md](controllers.md#hand-guiding)). Friction compensation (the hand-guiding friction
hook is zero here) and latency become the first things to measure. `ur_controllers` already ships
a freedrive controller and a twist controller, which are the stock baselines for hand guiding and
TCP jogging, and a friction-model controller for direct torque. The robot's own safety system
(joint speed and torque limits, protective stop) sits above every safety layer in this
repository and would trip on, for example, the drift that follows a 1 kg payload-model error.

Where direct torque is not available or not wanted, hand guiding becomes velocity servoing on a
force estimate: read the wrist F/T sensor (exposed here too, as `tcp_fts_sensor`, though no
controller reads it), subtract the payload's gravity wrench using the same payload model, and
command a TCP velocity proportional to the remaining external force through the Jacobian (an
admittance law), with the same speed cap and limit handling. The tracking controller's
feedforward would then move into the trajectory generator (torque-feasibility checks, time
scaling) instead of the torque loop. What transfers either way is the structure: dynamics from
the URDF through Pinocchio or KDL, the runtime payload, the safety layers (under the robot's
own), and the benchmarks that say what a model error costs.

### F5. No controller handles a stale or NaN joint state — the first thing that would bite on hardware

- *Stale state.* `EffortControllerBase::read_state()` (`effort_controller_base.cpp:147–161`)
  returns false and keeps the previous sample when a state interface has no value, but all three
  `update()` functions ignore the return value: `pid_ff_trajectory_controller.cpp:241`,
  `gravity_comp_handguide_controller.cpp:115`, `tcp_jog_controller.cpp:180`. Only `on_activate()`
  checks it. A controller that loses its state keeps computing torque from the last sample,
  indefinitely. `write_effort()`'s return value is ignored too
  (`pid_ff_trajectory_controller.cpp:315`, `gravity_comp_handguide_controller.cpp:148` and `:152`,
  `tcp_jog_controller.cpp:238`).
- *NaN.* `Pid::saturate` (`control/control_laws.hpp:50–53`) clamps with `std::clamp`, which
  returns NaN for a NaN input, so a NaN in $q$ or $\dot q$ reaches the effort interface. It also
  poisons the integrator for good: both saturation tests compare false, so the NaN is stored in
  `integral` (`control_laws.hpp:40–44`). Hand guiding saturates its torque with the same function
  (`gravity_comp_handguide_controller.cpp:131`), and its speed cap does not trip on a NaN velocity
  either, because `first_speed_violation` (`control_laws.hpp:74–84`) tests $|\dot q| > \text{cap}$,
  which is false for NaN.
- *Unbounded jog reference.* The jog controller integrates $q_{des}$ open loop
  (`tcp_jog_controller.cpp:215–219`) with no bound on $|q_{des} - q|$. If the arm is blocked, the
  reference keeps going and the torque rises to the effort limit. None of the three controllers
  has a tracking-error abort.

On hardware this is the first thing that would bite. The fix is a sample-age watchdog and a NaN
guard in front of every control law, a bound on $|q_{des} - q|$, and a controlled stop when any
of them trips.

### F6. Switching a joint from effort back to position drops the arm (gz_ros2_control 3.0.9)

With gz_ros2_control 3.0.9, switching a joint from my effort tracking controller back to the
stock position `joint_trajectory_controller` left it uncommanded: `/joint_states` effort went to
zero and the arm fell onto the ground within a second. Position to effort works. The binary
package ships no source, so I have not found the cause, and no test or benchmark reproduces it
(observed on 28 Oct). I removed the way back: there is no mode that returns to position control,
and homing is done by the effort tracking controller itself (`common.py` `home()`,
`controller_switching.py`).

### F7. The stock `ur_sim_control.launch.py` does not work on Lyrical as shipped

I meant to wrap `ur_sim_control.launch.py` from ur_simulation_gz 2.5.0 and only swap in my URDF
and controller YAML. It failed in two ways. Its spawners pass no parameter file, and ros2_control
6.10.1 only hands parameters to a controller through `<controller>.params_file`, so the stock
trajectory controller died with "Length of parameter 'joints' is '0'". (The inactive spawner also
still passes `--stopped`, which the 6.10.1 spawner no longer knows.) Then, on a loaded laptop,
`ros_gz_sim create` asked Gazebo for `/world/<w>/create` before Gazebo served it: the first
request timed out but was delivered, the retry succeeded too, and with renaming hard-coded on I
had two overlapping robots and two controller managers. My `sim.launch.py` keeps the same
building blocks, gives every controller a `params_file`, and spawns once, after the create
service exists; its docstring has the details. `SMOKE_TARGET=stock scripts/ci_smoke.sh` runs the
stock bringup for comparison; CI does not run it.

## The benchmark has a floor I have not removed

### F8. The feedforward error is set by a timing offset between reference and measured state, not by the control law (and not by CPU load)

This replaces the earlier item "the feedforward tracking numbers depend on what else the machine
is doing". The finding, the evidence and the measurement I would make are in
[Timing offset between reference and state](#timing-offset-between-reference-and-state). In
short: within a run, RMSE divided by RMS reference velocity is the same on all six joints for
both model-based laws, which is the signature of a sub-millisecond offset in when $q_{des}$ and
$q$ are paired, and that offset changes from run to run.

The earlier item also said that in the loaded run "the PID-only and hand-guiding rows did not
move". The hand-guiding rows did not (that benchmark is close to deterministic), but the PID
claim was an artefact of the worst-joint metric: PID's worst joint is wrist_2, whose large
control error does not move, while wrist_3's PID-only RMSE, which sits on the timing floor, moves
with every run (appendix A4 and A5). The same item said re-running the v0.1 code on 29 Oct
happened "with no package changed in between"; no package list was recorded, so that cannot be
checked.

### F9. wrist_3 is the worst joint because of that timing floor, not because of a stability limit

The earlier item said the wrist_3 loop "is at the edge of what a 2 ms explicit loop allows",
because its tiny inertia means $K_d \Delta t / M$ must stay below one. With the shipped gains and
the model's inertia that ratio is about 0.16 for wrist_3, six times inside the bound
([controllers.md](controllers.md#gains-and-bandwidths); 0.13 with the YAML comment's rounded
inertia). What the earlier item got right: wrist_3's inertia is tiny (the last link plus
whatever is bolted to the flange), its gains are tiny too, and a payload changes its dynamics
more than any other joint's.

wrist_3 is the worst joint for PID + feedforward and computed torque because it has the largest
path on the benchmark, hence the
largest RMS reference velocity, hence the largest error under a timing offset that is the same
for every joint. Its waypoints lie 0.8 rad either side of the start (segments of 0.8, 1.6 and
0.8 rad), against 0.6 rad or less for every other joint (`PATH_OFFSETS` in `bench_tracking.py`).

### F10. The max-error statistic is unstable between runs, because the trapezoid steps the desired acceleration at segment corners

For the model-based laws the worst-joint max error is several times the worst-joint RMSE, and
the ratio itself changes a lot between runs; between the 29 Oct run and the 31 Oct regeneration
the max moved the opposite way to the RMSE (appendix A6). A pure timing offset would put the max
near the peak-to-RMS ratio of the reference velocity, about 2 for wrist_3 on this path, so short
transients dominate the max. The 1/3-rule trapezoid steps $\ddot q_{des}$ at the four corners of
every segment (infinite jerk), the RNEA feedforward turns each step into a torque step, and any
timing offset turns a torque step into an error spike; an RMSE over a window that is mostly
cruise and rest dilutes them. I have not localised the max in time, so the corners are the
candidate, not a verified cause. The fix is a jerk-limited (S-curve or quintic) profile, and
reporting where the max occurs, or a high percentile instead of the max.

## Rough edges in the controllers

### F11. The speed cap is a hard stop, and its fallback brakes hard (see F12)

If any joint exceeds `max_joint_velocity`, hand guiding writes its gravity-compensation-plus-
damping torque once more and returns an error; the controller manager then activates the
configured fallback, the tracking controller. Its `on_activate()` sets $q_{des} = q$ and
$\dot q_{des} = 0$ (`pid_ff_trajectory_controller.cpp:201–202`), so it does stop the arm where the
trip happened, but it gets there by braking at full torque (F12), not with a gentle position
hold, as the earlier item said. Without a fallback nothing would actively stop the arm. No test
covers this path; I checked it by hand by lowering the cap at runtime, and I also found by hand
that a sustained push of a few tens of newtons does not trip the default cap with the shipped
damping (appendix A9).

### F12. The speed-cap fallback is a full-torque brake, not a gentle hold

When the fallback activates, $e = 0$ and $\dot e = -\dot q$, so its damping term alone asks for
$K_d \dot q$. Hand guiding trips at 1.5 rad/s on the base joints; on shoulder_lift
$K_d$ = 100 N·m·s/rad, so the request is at least 150 N·m, which is the joint's full URDF effort
limit (`effort_limit_scale` 1.0) and saturates the command. Shoulder_pan ($K_d$ = 50) and elbow
($K_d$ = 40) ask for 75 and 60 N·m at the same speed. The joint that carries most of the arm is
stopped with everything it has. On hardware that is a protective stop at best. The fix: start
the fallback with $\dot q_{des} = \dot q$ and decelerate the reference to rest within the joint
acceleration limits, or a dedicated stop controller.

### F13. Trajectories stop at every waypoint, and pre-emption is abrupt

The trapezoidal re-profiling is rest-to-rest per segment (no blending), which is simple and
exactly testable but slower than necessary. A new trajectory pre-empts the running one from the
current desired position at zero velocity, so pre-empting at cruise speed steps the desired
velocity to zero and the $K_d$ term produces a torque spike. The jog controller has a smaller
relative of this: its position-limit stop is applied after the acceleration limit, so at the
margin the joint velocity can drop to zero within one cycle and the resulting $\ddot q_{des}$
step goes into the feedforward (`tcp_jog_controller.cpp:211–216`).

## Fixed along the way

**Gazebo and the model disagreed about gravity.** SDF's default gravity is 9.8 m/s²; Pinocchio
uses 9.81. In my first hand-guiding run the arm drifted even with a perfect payload model.
Setting `<gravity>0 0 -9.81</gravity>` in the worlds cut most of that drift (appendix A10; the hold-force offset I also saw turned out to be the next bug). It is now one
`gravity_z` parameter shared by both dynamics backends, with a test that keeps it equal to the
worlds.

**My controllers dropped the arm at every hand-over.** The static hold-force measurement showed
an offset while the arm with a correct payload model did not drift at all, which cannot both be
true. A raw dump explained it: at the first hand-guiding sample the elbow was already moving. The
outgoing controller wrote zero torque in `on_deactivate()`, and the hardware applied that for one
2 ms cycle before the next controller's first update; the elbow's gravity torque over its inertia
for 2 ms gives about the velocity I saw (appendix A11). Leaving the last torque on the interface
fixed it, and the hold force then matched $\Delta m\,g$ exactly.

**Joint damping did not make a payload error safe.** See F1 and appendix A12.

**Smaller ones.** CI could not run at all until the repository was public (billing). Its first
real run failed because setuptools in the image ignores `tests_require`, so colcon ran the
Python tests under unittest and found none. And when I added a "reach the goal" check to the
smoke test, the first version compared positions in `/joint_states` order, which on this sim
starts with the elbow.

## Appendix: hand-recorded runs (not regenerated by any script)

Every number below was produced once, by hand or by a one-off script that is not in the
repository, and no script here regenerates it. Each entry says when and how. Entries copied from
the build log (which is not part of the repository) are unchanged. Where an entry is computed
from a CSV that is in git history, the `git show` command gives the input.

**A1. RMSE divided by RMS reference velocity, per run** (computed on 31 Oct from the committed
CSVs, as described in [Timing offset between reference and state](#timing-offset-between-reference-and-state);
mean over 3 repetitions at 1×, range over the six joints). The review of 31 Oct computed the same
ranges independently.

| Run | CSV | PID + FF (ms) | Computed torque (ms) | PID (ms) |
|---|---|---|---|---|
| v0.1, 28 Oct overnight | `git show 3f3d9bc:results/tracking.csv` | 0.107–0.116 | not run | 0.28 (wrist_3) – 24.0 (wrist_2) |
| v0.1 published, 28 Oct midday | `git show 933fae5:results/tracking.csv` | 0.091–0.133 | not run | 0.22 (wrist_3) – 24.0 (wrist_2) |
| v0.2, 29 Oct 03:39 (stamp `6bf35f4`) | `git show 28074ac:results/tracking.csv` | 0.415–0.440 | 0.432–0.437 | 0.44 (wrist_3) – 24.0 (wrist_2) |

The 31 Oct runs are generated in [results.md](results.md#run-to-run-spread).

**A2. PID + feedforward worst-joint RMSE at 1× against machine load** (mrad, mean ± std over 3
repetitions; load as recorded at the time).

| Run | Code | Load as recorded | PID + FF RMSE | PID + FF max error | Source |
|---|---|---|---|---|---|
| 28 Oct overnight final | v0.1 | alone | 0.05 ± 0.02 | 0.87 ± 0.16 | build log; `git show 3f3d9bc:results/tracking.csv` |
| 28 Oct midday run 1 | v0.1 | another benchmark running on the same laptop | 0.13 ± 0.04 | 1.80 ± 1.38 | build log only, no CSV |
| 28 Oct midday run 2 (published v0.1) | v0.1 | alone, 437 s wall | 0.04 ± 0.03 (0.045 ± 0.028) | 1.22 ± 0.47 | build log; `git show 933fae5:results/tracking.csv` |
| 29 Oct 03:39 rerun (published until 31 Oct) | `6bf35f4` | load average 1.21 | 0.209 ± 0.027 | — | `git show 28074ac:results/tracking.csv` |
| 29 Oct 04:20 A/B | v0.1 code (`c35e70c`) | load average 3.80 | 0.242 ± 0.008 | — | build log only, no CSV |
| 31 Oct 07:00 regeneration | `28074ac` | load average 1.76 at start; a few seconds of ffmpeg and an HTML validator during part of the run | see [results.md](results.md#tracking-full-tables) (now the published table) | — | `regenerate.log`, outside the repository |

**A3. Tracking rerun on a quiet machine, 29 Oct 03:39** (build log; stamp `6bf35f463513`,
backend pinocchio, load average 1.21, 393 s wall). Worst-joint RMSE at 1× per rep: PID 8.81 /
8.82 / 8.81 mrad (v0.1: 8.83), PID + FF 0.233 / 0.180 / 0.215 mrad (v0.1: 0.04 ± 0.03), computed
torque 0.236 / 0.165 / 0.235 mrad. On this run computed torque ≈ PID + FF; the 2.2× gap seen
earlier came from a check made while a build was running (A7).

**A4. A/B, 29 Oct 04:20: the v0.1 code reproduces the new level, not its own published
numbers** (build log). The v0.1 commit (`c35e70c`) was built in a separate worktree and run with
its own `scripts/bench_tracking.sh` under the same locks (load average 3.80, 287 s wall).
Worst-joint RMSE in mrad, mean ± std over 3 reps:

| Run | PID 0.5× | PID 1× | PID 2× | PID+FF 0.5× | PID+FF 1× | PID+FF 2× | wrist_3 PID 1× |
|---|---|---|---|---|---|---|---|
| v0.1 published (`c35e70c`, 28 Oct midday) | 5.346 ± 0.006 | 8.828 ± 0.007 | 10.700 ± 0.036 | 0.027 ± 0.010 | 0.045 ± 0.028 | 0.141 ± 0.047 | 0.106 |
| v0.1 code re-run 29 Oct 04:20 | 5.338 ± 0.005 | 8.809 ± 0.007 | 10.680 ± 0.042 | 0.094 ± 0.018 | 0.242 ± 0.008 | 0.335 ± 0.046 | 0.228 |
| v0.2 rerun 29 Oct 03:39 (`6bf35f4`) | 5.336 ± 0.002 | 8.812 ± 0.006 | 10.680 ± 0.044 | 0.085 ± 0.005 | 0.209 ± 0.027 | 0.368 ± 0.087 | 0.216 |

**A5. wrist_3's PID-only RMSE moves between runs** (computed on 31 Oct from the CSVs; rows
`pid, 1.0, wrist_3_joint`, per rep, mrad). 29 Oct 03:39 run (`git show
28074ac:results/tracking.csv`): 0.240 / 0.210 / 0.198, mean 0.22. 31 Oct 07:00 regeneration
(now `results/tracking.csv`): 0.133 / 0.112 / 0.148, mean 0.13. PID's worst joint, wrist_2, did
not move beyond its within-run spread, so the worst-joint metric hid the change.

**A6. Max error against RMSE for the model-based laws** (computed on 31 Oct from the same two
CSVs; worst-joint max divided by worst-joint RMSE, per rep, at 1×). 29 Oct 03:39: PID + FF 3.7 /
5.4 / 4.6, computed torque 3.5 / 5.6 / 3.3. 31 Oct 07:00: PID + FF 17.2 / 11.1 / 12.0, computed
torque 12.8 / 10.6 / 18.3. The PID + FF worst-joint max at 0.5× went from 0.50 ± 0.00 to
1.00 ± 0.37 mrad between the two runs while its RMSE fell.

**A7. Computed torque under contention, 29 Oct 03:13** (build log; 1 rep, 1×, Pinocchio, while
other builds were compiling; CSV in `/tmp`, not kept). Worst-joint RMSE / max |e|: PID + FF 0.175
/ 2.88 mrad, computed torque 0.386 / 5.46 mrad, i.e. computed torque about 2.2× worse on every
joint. PID + FF with $K_i = 0$: 0.132 / 0.95 mrad, so the integral was not the cause. The earlier
README item 6 and controllers.md said "about 2× behind" from this check.

**A8. Offline replica of the tracking loop, 29 Oct about 03:15** (build log; a Python script in
`/tmp`, not kept). Same path and gains on an exact Pinocchio plant with a 2 ms hold and 1 ms
substeps: RMSE per joint PID + FF 0.007–0.020 mrad, computed torque 0.009–0.028 mrad. With 1 or
2 ms of extra actuation delay computed torque degraded faster (ratio to PID + FF growing to
2–3×), but the replica's errors stayed 3–10× below Gazebo's for both laws, so the main error
source in Gazebo is something the replica does not model. A separate replica of the closed-loop
unit test's sinusoid (`test_computed_torque.cpp`, the "~0.1 mrad" in its comment) gave
9.5·10⁻⁵ rad at the sampling instant.

**A9. Speed-cap fallback, checked by hand, 28 Oct 05:10** (build log). Cap lowered to 0.3 rad/s
at runtime, 15 N push at the wrist: `speed cap: |qd| of wrist_1_joint is 0.385 rad/s > 0.300
rad/s; deactivating`, then `Activating fallback controllers : [ pid_ff_trajectory_controller ]`,
arm held at q = [0, −1.565, 1.578, −1.568, −1.57, 0]. A 25 N push with the default cap did not
trip it (the earlier README item 4 said "a few tens of newtons").

**A10. Gravity mismatch, 28 Oct 04:45–05:20** (build log). First full regeneration (commit
`a1a3c35`, not kept): hold force −10.57 / −0.55 / +8.99 N for payload error −1 / 0 / +1 kg
(expected −9.81 / 0 / +9.81), a constant offset of about −0.6 N, and the arm with a correct
payload model drifting at 4 mm/s at 2 s. The estimate at the time was that the 9.8 / 9.81
mismatch explains only about 0.1 N of the offset; the rest was the hand-over bug (A11). Second
regeneration with `<gravity>0 0 -9.81</gravity>` (commit `e73a586`): the zero-error drift went
from 4 mm/s to 0.3 mm/s.

**A11. Hand-over drop, 28 Oct 05:35** (build log). The static hold force showed a constant
−2.4 N offset while the arm with a correct payload model did not drift. At the first hand-guiding
sample the elbow was moving at 0.052 rad/s; 25 N·m on 0.88 kg m² for 2 ms is about 0.057 rad/s.
After the fix (commit `f2bcf5f`) the hold force was −9.81 / 0.00 / +9.81 N. controllers.md
called this "a 0.05 rad/s jolt at the elbow".

**A12. First hand-guiding damping, 28 Oct 05:20** (build log; commit `e73a586`). With damping
[8, 8, 5, 2, 1, 0.05] a 1 kg payload-model error moved the TCP 0.99 m (Δm = −1 kg) and 0.50 m
(Δm = +1 kg) in 2 s; the +1 kg arm ended on the floor, so its "speed at 2 s" (4.4 mm/s) measured
a collision. The damping was raised to the shipped [25, 25, 15, 3, 1.5, 0.05].

**A13. Dynamics timing spread, 29 Oct** (build log). The run published until 31 Oct
(`results/dynamics_timing.csv` at 28074ac, commit `96365efa478c`, RelWithDebInfo, i5-1135G7) and
two extra runs right after it, same commit, same locks (not published). Median / p95 in µs:

| Call | published run | rerun 2 | rerun 3 |
|---|---|---|---|
| clock (two reads) | 0.016 / 0.017 | 0.016 / 0.018 | 0.015 / 0.016 |
| Pinocchio g(q) | 0.538 / 0.740 | 0.508 / 0.580 | 0.511 / 0.684 |
| Pinocchio RNEA | 0.887 / 1.123 | 0.794 / 1.004 | 0.798 / 1.165 |
| KDL g(q) | 1.678 / 1.748 | 1.479 / 2.085 | 1.524 / 1.627 |
| KDL RNEA | 1.702 / 2.155 | 1.483 / 1.725 | 1.551 / 2.112 |

Medians move by up to ~12 % between these three runs and p95 more; the ordering (Pinocchio g <
Pinocchio RNEA < KDL g ≈ KDL RNEA) and "worst p95 ≤ 0.11 % of 2 ms" hold in all three. The
31 Oct regeneration (now the published table in [results.md](results.md#dynamics-timing)) moved
further than 12 % on the KDL medians.
