# Results

Every number on this page is computed by `scripts/make_results.py` from the CSVs in
`results/`, which the benchmark scripts write. Regenerate everything with
`scripts/regenerate_all.sh`. Simulation only (Gazebo Jetty, DART); see
[the README](../README.md#where-it-fails) for what does not transfer to a real UR5e.

## Summary

**Joint-space tracking** (PidFfTrajectoryController, 3-segment 6-joint path; worst joint per repetition, then mean ± std over repetitions):

| Speed (× nominal) | PID: worst-joint RMSE (mrad) | PID + RNEA FF: worst-joint RMSE (mrad) | PID: worst max \|e\| (mrad) | PID + RNEA FF: worst max \|e\| (mrad) | Regenerate with |
|---|---|---|---|---|---|
| 0.5 | 5.34 ± 0.00 | 0.03 ± 0.01 | 11.89 ± 0.04 | 0.48 ± 0.00 | `scripts/bench_tracking.sh` |
| 1 | 8.83 ± 0.01 | 0.05 ± 0.02 | 20.60 ± 0.05 | 0.87 ± 0.16 | `scripts/bench_tracking.sh` |
| 2 | 10.70 ± 0.03 | 0.13 ± 0.02 | 26.11 ± 0.06 | 1.89 ± 0.00 | `scripts/bench_tracking.sh` |

**Hand guiding with a wrong payload model** (GravityCompHandGuidingController, model payload fixed, actual Gazebo payload varied; mean ± std over repetitions):

| Model / actual payload (kg) | Payload error (kg) | TCP drift speed at 1 s (mm/s) | TCP drift in 1 s (mm) | Effort to hold, F_z (N) | Expected (Δm·g, N) | Regenerate with |
|---|---|---|---|---|---|---|
| 1 / 0 | -1 | 374.40 ± 1.96 | 295.94 ± 4.14 | -9.81 ± 0.00 | -9.81 | `scripts/bench_handguide.sh` |
| 1 / 1 | +0 | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.00 | `scripts/bench_handguide.sh` |
| 1 / 2 | +1 | 202.04 ± 2.48 | 200.91 ± 3.35 | 9.81 ± 0.00 | 9.81 | `scripts/bench_handguide.sh` |

## Tracking — details

`results/tracking.csv`: commit `26e5920ad554`, seed 1, physics step 0.001 s, controller update rate 500 Hz; 3 repetitions per condition.

Method: the arm is homed by the tracking controller itself (feedforward on) to a
seeded random offset (≤ 0.05 rad per joint) from the home pose; for the PID-only
variant feedforward is then switched off and the integrator given 3 s to carry
gravity. A 3-waypoint path (2.5 s per segment at 1×) is sent on
`~/joint_trajectory`. The error is the controller's own `q_des − q` at 500 Hz,
from trajectory start to 0.5 s after the last
reference motion. "Worst joint" = the joint with the largest RMSE in that run.
Whole-benchmark reruns reproduce the PID-only rows closely; the feedforward rows sit
near the noise floor and vary more between reruns than the within-run std shows
(the run log is in NOTES.md).

![tracking](assets/tracking_rmse_light.png)

Per joint, RMSE / max |e| in mrad, mean ± std over repetitions:

| Speed | Variant | shoulder_pan_joint RMSE / max (mrad) | shoulder_lift_joint RMSE / max (mrad) | elbow_joint RMSE / max (mrad) | wrist_1_joint RMSE / max (mrad) | wrist_2_joint RMSE / max (mrad) | wrist_3_joint RMSE / max (mrad) |
|---|---|---|---|---|---|---|---|
| 0.5 | PID | 0.16 ± 0.01 / 1.00 ± 0.02 | 1.00 ± 0.01 / 2.11 ± 0.07 | 0.08 ± 0.01 / 0.36 ± 0.15 | 1.24 ± 0.12 / 3.06 ± 0.34 | 5.34 ± 0.00 / 11.89 ± 0.04 | 0.04 ± 0.01 / 0.76 ± 0.24 |
| 0.5 | PID + RNEA FF | 0.02 ± 0.01 / 0.36 ± 0.00 | 0.01 ± 0.00 / 0.12 ± 0.00 | 0.01 ± 0.00 / 0.15 ± 0.00 | 0.02 ± 0.01 / 0.27 ± 0.00 | 0.02 ± 0.01 / 0.37 ± 0.01 | 0.03 ± 0.01 / 0.48 ± 0.00 |
| 1 | PID | 0.89 ± 0.04 / 3.06 ± 0.33 | 1.92 ± 0.01 / 4.10 ± 0.09 | 0.15 ± 0.00 / 0.53 ± 0.02 | 2.02 ± 0.23 / 4.85 ± 0.59 | 8.83 ± 0.01 / 20.60 ± 0.05 | 0.14 ± 0.03 / 1.36 ± 0.15 |
| 1 | PID + RNEA FF | 0.04 ± 0.02 / 0.65 ± 0.11 | 0.01 ± 0.00 / 0.24 ± 0.00 | 0.02 ± 0.00 / 0.30 ± 0.00 | 0.03 ± 0.01 / 0.49 ± 0.09 | 0.04 ± 0.02 / 0.67 ± 0.14 | 0.05 ± 0.02 / 0.87 ± 0.16 |
| 2 | PID | 4.60 ± 0.20 / 11.42 ± 0.99 | 3.65 ± 0.01 / 7.65 ± 0.13 | 0.41 ± 0.02 / 1.08 ± 0.05 | 2.85 ± 0.32 / 6.44 ± 0.97 | 10.70 ± 0.03 / 26.11 ± 0.06 | 0.50 ± 0.02 / 2.94 ± 0.80 |
| 2 | PID + RNEA FF | 0.11 ± 0.01 / 1.40 ± 0.10 | 0.04 ± 0.01 / 0.48 ± 0.02 | 0.05 ± 0.01 / 0.60 ± 0.03 | 0.08 ± 0.01 / 1.06 ± 0.02 | 0.11 ± 0.01 / 1.45 ± 0.14 | 0.13 ± 0.02 / 1.89 ± 0.00 |

## Hand guiding — details

`results/handguide.csv`: commit `26e5920ad554`, seed 1, physics step 0.001 s, controller update rate 500 Hz; 3 repetitions per condition.

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
