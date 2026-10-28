# Results

Every number on this page is computed by `scripts/make_results.py` from the CSVs in
`results/`, which the benchmark scripts write. Regenerate everything with
`scripts/regenerate_all.sh`. Simulation only (Gazebo Jetty, DART); see
[the README](../README.md#where-it-fails) for what does not transfer to a real UR5e.

## Summary

**Joint-space tracking** (PidFfTrajectoryController, 3-segment 6-joint path; worst joint per repetition, then mean ± std over repetitions):

| Speed (× nominal) | PID: worst-joint RMSE (mrad) | PID + RNEA FF: worst-joint RMSE (mrad) | Computed torque: worst-joint RMSE (mrad) | PID: worst max \|e\| (mrad) | PID + RNEA FF: worst max \|e\| (mrad) | Computed torque: worst max \|e\| (mrad) | Regenerate with |
|---|---|---|---|---|---|---|---|
| 0.5 | 5.34 ± 0.00 | 0.08 ± 0.00 | 0.09 ± 0.01 | 12.03 ± 0.14 | 0.50 ± 0.00 | 0.60 ± 0.23 | `scripts/bench_tracking.sh` |
| 1 | 8.81 ± 0.01 | 0.21 ± 0.03 | 0.21 ± 0.04 | 20.67 ± 0.04 | 0.94 ± 0.07 | 0.84 ± 0.07 | `scripts/bench_tracking.sh` |
| 2 | 10.68 ± 0.04 | 0.37 ± 0.09 | 0.38 ± 0.09 | 26.45 ± 0.36 | 2.83 ± 1.00 | 2.52 ± 1.47 | `scripts/bench_tracking.sh` |

**Hand guiding with a wrong payload model** (GravityCompHandGuidingController, model payload fixed, actual Gazebo payload varied; mean ± std over repetitions):

| Model / actual payload (kg) | Payload error (kg) | TCP drift speed at 1 s (mm/s) | TCP drift in 1 s (mm) | Effort to hold, F_z (N) | Expected (Δm·g, N) | Regenerate with |
|---|---|---|---|---|---|---|
| 1 / 0 | -1 | 374.42 ± 1.98 | 296.18 ± 4.44 | -9.81 ± 0.00 | -9.81 | `scripts/bench_handguide.sh` |
| 1 / 1 | +0 | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.00 | `scripts/bench_handguide.sh` |
| 1 / 2 | +1 | 201.97 ± 2.39 | 201.04 ± 3.53 | 9.81 ± 0.00 | 9.81 | `scripts/bench_handguide.sh` |

**Cost of one dynamics call** (gravity g(q) and RNEA on each backend, single thread; median / p95 over 100 000 timed calls):

| Per call, median / p95 (µs) | Pinocchio g(q) | Pinocchio RNEA | KDL g(q) | KDL RNEA | Worst p95, share of the 2 ms cycle | Regenerate with |
|---|---|---|---|---|---|---|
| UR5e, 6 DoF, one thread | 0.538 / 0.740 | 0.887 / 1.123 | 1.678 / 1.748 | 1.702 / 2.155 | 0.1077 % | `scripts/bench_dynamics_timing.sh` |

## Tracking — details

`results/tracking.csv`: commit `6bf35f463513`, seed 1, physics step 0.001 s, controller update rate 500 Hz, dynamics backend pinocchio; 3 repetitions per condition.

Method: the arm is homed by the tracking controller itself (feedforward on) to a
seeded random offset (≤ 0.05 rad per joint) from the home pose; for the PID-only
variant feedforward is then switched off and the integrator given 3 s to carry
gravity; the computed-torque variant switches the same controller to
`control_law: computed_torque` (ω, ζ per joint from `controllers.yaml`), with no
integral term. A 3-waypoint path (2.5 s per segment at 1×) is sent on
`~/joint_trajectory`. The error is the controller's own `q_des − q` at 500 Hz,
from trajectory start to 0.5 s after the last
reference motion. "Worst joint" = the joint with the largest RMSE in that run.
These rows replace v0.1's two-column table. Its PID + RNEA FF rows were several
times lower; re-running the v0.1 code itself on 29 Oct reproduced this table's
level, not its own, with no package changed in between (NOTES.md has the A/B), so
the two tables should not be compared row by row.
Whole-benchmark reruns reproduce the PID-only rows closely; the feedforward rows sit
near the noise floor and vary more between reruns than the within-run std shows
(the run log is in NOTES.md).

![tracking](assets/tracking_rmse_light.png)

Per joint, RMSE / max |e| in mrad, mean ± std over repetitions:

| Speed | Variant | shoulder_pan_joint RMSE / max (mrad) | shoulder_lift_joint RMSE / max (mrad) | elbow_joint RMSE / max (mrad) | wrist_1_joint RMSE / max (mrad) | wrist_2_joint RMSE / max (mrad) | wrist_3_joint RMSE / max (mrad) |
|---|---|---|---|---|---|---|---|
| 0.5 | PID | 0.18 ± 0.02 / 0.96 ± 0.07 | 1.00 ± 0.01 / 2.07 ± 0.03 | 0.09 ± 0.01 / 0.27 ± 0.01 | 1.23 ± 0.12 / 3.13 ± 0.38 | 5.34 ± 0.00 / 12.03 ± 0.14 | 0.10 ± 0.03 / 0.53 ± 0.04 |
| 0.5 | PID + RNEA FF | 0.06 ± 0.00 / 0.36 ± 0.01 | 0.02 ± 0.00 / 0.13 ± 0.01 | 0.03 ± 0.00 / 0.16 ± 0.02 | 0.05 ± 0.00 / 0.28 ± 0.00 | 0.07 ± 0.00 / 0.39 ± 0.01 | 0.08 ± 0.00 / 0.50 ± 0.00 |
| 0.5 | Computed torque | 0.07 ± 0.00 / 0.45 ± 0.18 | 0.02 ± 0.00 / 0.12 ± 0.00 | 0.03 ± 0.00 / 0.15 ± 0.00 | 0.05 ± 0.00 / 0.34 ± 0.13 | 0.07 ± 0.00 / 0.43 ± 0.17 | 0.09 ± 0.01 / 0.60 ± 0.23 |
| 1 | PID | 0.90 ± 0.04 / 3.13 ± 0.06 | 1.92 ± 0.01 / 4.16 ± 0.04 | 0.16 ± 0.01 / 0.51 ± 0.02 | 2.02 ± 0.22 / 4.92 ± 0.59 | 8.81 ± 0.01 / 20.67 ± 0.04 | 0.22 ± 0.02 / 1.56 ± 0.38 |
| 1 | PID + RNEA FF | 0.16 ± 0.02 / 0.64 ± 0.03 | 0.05 ± 0.01 / 0.23 ± 0.01 | 0.06 ± 0.01 / 0.29 ± 0.01 | 0.12 ± 0.01 / 0.51 ± 0.03 | 0.16 ± 0.02 / 0.72 ± 0.08 | 0.21 ± 0.03 / 0.94 ± 0.07 |
| 1 | Computed torque | 0.16 ± 0.03 / 0.62 ± 0.05 | 0.05 ± 0.01 / 0.27 ± 0.11 | 0.07 ± 0.01 / 0.34 ± 0.14 | 0.12 ± 0.02 / 0.48 ± 0.02 | 0.16 ± 0.03 / 0.59 ± 0.04 | 0.21 ± 0.04 / 0.84 ± 0.07 |
| 2 | PID | 4.63 ± 0.21 / 11.62 ± 0.47 | 3.67 ± 0.01 / 7.81 ± 0.07 | 0.43 ± 0.01 / 1.07 ± 0.07 | 2.84 ± 0.31 / 6.68 ± 1.13 | 10.68 ± 0.04 / 26.45 ± 0.36 | 0.63 ± 0.05 / 3.57 ± 0.10 |
| 2 | PID + RNEA FF | 0.28 ± 0.07 / 2.08 ± 0.83 | 0.09 ± 0.03 / 0.45 ± 0.05 | 0.11 ± 0.03 / 0.57 ± 0.06 | 0.21 ± 0.05 / 1.56 ± 0.58 | 0.29 ± 0.08 / 2.22 ± 0.47 | 0.37 ± 0.09 / 2.83 ± 1.00 |
| 2 | Computed torque | 0.29 ± 0.07 / 1.90 ± 1.09 | 0.10 ± 0.02 / 0.63 ± 0.25 | 0.12 ± 0.02 / 0.80 ± 0.31 | 0.21 ± 0.05 / 1.43 ± 0.83 | 0.29 ± 0.07 / 1.80 ± 1.13 | 0.38 ± 0.09 / 2.52 ± 1.47 |

## Hand guiding — details

`results/handguide.csv`: commit `c35e70c82023`, seed 1, physics step 0.001 s, controller update rate 500 Hz, dynamics backend pinocchio; 3 repetitions per condition.

Method: per repetition the tracking controller holds the arm at a seeded random
offset (≤ 0.05 rad per joint) from home until its integrator has converged (3 s);
its torque is then what the simulated arm needs to stay put. The hand-guiding
controller then takes over with the model payload and nobody touches the arm.
*Effort to hold* is the tool force an operator must apply to keep the arm still,
F = J(q)⁻ᵀ (τ_hold − τ_handguide) at the release pose; its z component should
equal (m_actual − m_model)·g. *Drift* is the TCP linear speed |J(q) q̇| over
[0.9, 1.0] s after release and the TCP displacement at 1 s (by 2 s a 1 kg error has
carried the arm tens of centimetres, towards the floor when it sinks).

The hold force matches Δm·g to the printed precision because Gazebo (DART) and
Pinocchio are given the same URDF and the same gravity (9.81 m/s², set explicitly in
the world): the benchmark confirms that the payload model enters the dynamics at the
right place and that a hand-over between controllers is seamless. On a real arm this
is exactly where friction and model error would appear. Two earlier versions of this
benchmark showed offsets of −0.5 N and −2.4 N; both were artefacts (SDF's default
gravity of 9.8, and a controller that zeroed its torque on deactivation) and are
described in NOTES.md.

![handguide](assets/handguide_light.png)

## Dynamics timing — details

`results/dynamics_timing.csv`: commit `96365efa478c`, build type RelWithDebInfo, CPU 11th Gen Intel(R) Core(TM) i5-1135G7 @ 2.40GHz, 100,000 timed calls per row, seed 1.

Method: `bench_dynamics_timing` (C++, no ROS graph, no simulator, one thread) loads
the simulation's URDF into each backend through the same `DynamicsModel` interface the
controllers use, draws the states up front (q uniform inside the URDF position limits,
q̇ inside the velocity limits, |q̈| ≤ 10 rad/s², seeded), makes 1 000 warm-up calls
and then times every call on its own with `std::chrono::steady_clock`. Times include
the wrapper's joint reordering and the timer itself (the *two clock reads only* row);
nothing is subtracted. *Share of 2 ms* is the time divided by one 500 Hz control cycle.
`scripts/bench_dynamics_timing.sh` refuses to run next to a simulator, build, pip
install or training run, and its log records the load average and the busiest
processes at start; on the shared dev laptop it runs under the machine's heavy, build
and Gazebo locks. In three back-to-back runs on 29 Oct the medians moved by up to ~12 %
(NOTES.md). These are laptop numbers: CI runs the executable as a 1 000-call smoke
test only, and runner timings are not published.

| Backend | Call | Median (µs) | p95 (µs) | Mean (µs) | Max (µs) | Median, share of 2 ms | p95, share of 2 ms |
|---|---|---|---|---|---|---|---|
| Pinocchio | g(q) | 0.538 | 0.740 | 0.568 | 39.23 | 0.0269 % | 0.0370 % |
| Pinocchio | RNEA | 0.887 | 1.123 | 0.921 | 41.70 | 0.0443 % | 0.0562 % |
| KDL | g(q) | 1.678 | 1.748 | 1.719 | 51.41 | 0.0839 % | 0.0874 % |
| KDL | RNEA | 1.702 | 2.155 | 1.765 | 130.96 | 0.0851 % | 0.1077 % |
| — | two clock reads only | 0.016 | 0.017 | 0.017 | 39.68 | 0.0008 % | 0.0008 % |
