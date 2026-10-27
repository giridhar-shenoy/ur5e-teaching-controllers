"""bench_tracking: joint-space tracking error of PidFfTrajectoryController.

Grid (v0.1): speed scale {0.5, 1.0, 2.0} x feedforward {off, on} x reps. One sim launch; the
arm is re-homed (same controller, feedforward on) before every run. The reference is a fixed
three-segment, six-joint path around the home pose; each repetition shifts the whole path by a
seeded random offset (|offset| <= 0.05 rad per joint) so repetitions sample nearby
configurations instead of replaying an identical deterministic run.

Metrics per joint over [trajectory start, end + settle]: RMSE and max |e| of q_des - q, where
q_des is the controller's own profiled reference (published on ~/controller_state at 500 Hz).

    ros2 run ur5e_teaching_bench bench_tracking --out results/tracking.csv --reps 3 --seed 1
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import rclpy

from ur5e_teaching_bench.common import (
    HOME,
    JOINTS,
    BenchNode,
    append_rows,
    git_commit,
    world_step_size,
)

CONTROLLER = "pid_ff_trajectory_controller"
# Offsets from HOME of the three waypoints; nominal (1.0x) time per segment.
PATH_OFFSETS = np.array(
    [
        [0.6, 0.4, -0.5, 0.5, 0.6, 0.8],
        [-0.6, 0.2, -0.3, -0.4, -0.6, -0.8],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]
)
NOMINAL_SEGMENT_S = 2.5
SETTLE_S = 0.5


def run_once(
    node: BenchNode, speed: float, feedforward: bool, offset: np.ndarray, dump: str | None = None
) -> dict:
    start = HOME + offset
    node.home(start)  # same controller, feedforward on
    node.set_params(CONTROLLER, {"feedforward": feedforward})
    # Without feedforward the arm sags until the integrator carries gravity; start the run from
    # that steady state, as a PID-only arm would be in practice (integral time 0.5 s).
    node.sleep_sim(0.3 if feedforward else 3.0)

    waypoints = [start + p for p in PATH_OFFSETS]
    seg = NOMINAL_SEGMENT_S / speed
    times = [seg * (k + 1) for k in range(len(waypoints))]
    node.start_recording(CONTROLLER)
    node.publish_trajectory(CONTROLLER, waypoints, times)
    node.sleep_sim(times[-1] + SETTLE_S + 0.5)
    rec = node.stop_recording()
    if dump:
        np.savez_compressed(dump, **rec)

    moving = np.any(np.abs(rec["reference_velocity"]) > 1e-9, axis=1)
    if not moving.any():
        raise RuntimeError("the reference never moved: trajectory not received?")
    t_start = rec["stamps"][np.argmax(moving)]
    t_last_motion = rec["stamps"][len(moving) - 1 - np.argmax(moving[::-1])]
    window = (rec["stamps"] >= t_start) & (rec["stamps"] <= t_last_motion + SETTLE_S)
    err = rec["error"][window]
    return {
        "rmse": np.sqrt(np.mean(err**2, axis=0)),
        "max_abs": np.max(np.abs(err), axis=0),
        "profiled_s": t_last_motion - t_start,
        "requested_s": times[-1],
        "samples": int(window.sum()),
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", required=True)
    parser.add_argument("--reps", type=int, default=3)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--speeds", type=float, nargs="+", default=[0.5, 1.0, 2.0])
    parser.add_argument("--ff", type=int, nargs="+", default=[0, 1], help="variants to run")
    parser.add_argument("--dump-dir", help="save every raw recording as .npz here")
    args = parser.parse_args(argv)

    rclpy.init()
    node = BenchNode("bench_tracking")
    node.wait_for_clock()
    rng = np.random.default_rng(args.seed)
    commit = git_commit()
    step = world_step_size()
    rate = node.get_param("controller_manager", "update_rate")
    node.switcher.ensure_configured(CONTROLLER)  # its parameter services exist only once loaded

    for rep in range(args.reps):
        offset = rng.uniform(-0.05, 0.05, size=6)
        for speed in args.speeds:
            for ff in (bool(v) for v in args.ff):
                dump = None
                if args.dump_dir:
                    os.makedirs(args.dump_dir, exist_ok=True)
                    dump = os.path.join(args.dump_dir, f"rep{rep}_speed{speed:g}_ff{int(ff)}.npz")
                r = run_once(node, speed, ff, offset, dump)
                rows = [
                    {
                        "bench": "tracking",
                        "git_commit": commit,
                        "sim_step_s": step,
                        "update_rate_hz": rate,
                        "seed": args.seed,
                        "rep": rep,
                        "speed_scale": speed,
                        "feedforward": int(ff),
                        "joint": joint,
                        "rmse_rad": f"{r['rmse'][j]:.6e}",
                        "max_abs_err_rad": f"{r['max_abs'][j]:.6e}",
                        "requested_s": f"{r['requested_s']:.3f}",
                        "profiled_s": f"{r['profiled_s']:.3f}",
                        "samples": r["samples"],
                    }
                    for j, joint in enumerate(JOINTS)
                ]
                append_rows(args.out, rows)
                node.get_logger().info(
                    f"rep {rep} speed {speed} ff {int(ff)}: max RMSE "
                    f"{r['rmse'].max():.2e} rad, max |e| {r['max_abs'].max():.2e} rad"
                )
    node.home()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
