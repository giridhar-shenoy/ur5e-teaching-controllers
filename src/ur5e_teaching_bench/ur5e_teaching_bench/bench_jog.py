"""bench_jog: command-to-motion latency and velocity tracking of TcpJogController.

Per repetition the tracking controller homes the arm to a seeded random offset (|offset| <= 0.05
rad per joint) from JOG_START, the jog controller takes over, and for each base axis x, y, z a
TCP twist of SPEED_MPS along +axis is streamed at STREAM_HZ (sim time, header stamped) for
STREAM_S, then stopped. Everything is measured from ~/jog_state, which carries the controller's
own update time and the stamp of the command it acts on (both sim time):

  latency_50 / latency_90  first command stamp -> measured TCP speed along the commanded
                           direction reaches 50 % / 90 % of the command (includes ROS transport,
                           the acceleration limit and the joint loop)
  tracking_rel_rms         RMS of |v_meas - v_cmd| (linear) over the second half of the stream,
                           relative to |v_cmd|
  stop                     last command stamp -> |v_meas| below 1 % of the command (includes the
                           command_timeout deadman and the deceleration)

    ros2 run ur5e_teaching_bench bench_jog --out results/jog.csv --reps 3 --seed 1
"""

from __future__ import annotations

import argparse
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray

from ur5e_teaching_bench.common import BenchNode, append_rows, git_commit, world_step_size

CONTROLLER = "tcp_jog_controller"
# Elbow bent further than HOME: manipulability ~0.056, so a 1 s jog at 5 cm/s along any axis
# stays above the damping threshold (w0 = 0.02).
JOG_START = np.array([0.0, -1.9, 1.9, -1.57, -1.57, 0.0])
SPEED_MPS = 0.05
STREAM_HZ = 100.0
STREAM_S = 1.0
AFTER_S = 0.6
AXES = {"x": 0, "y": 1, "z": 2}


class JogRecorder:
    def __init__(self, node: BenchNode):
        self.rows: list[list[float]] = []
        self.recording = False
        node.create_subscription(Float64MultiArray, f"/{CONTROLLER}/jog_state", self._on_state, 100)

    def _on_state(self, msg: Float64MultiArray) -> None:
        if self.recording:
            self.rows.append(list(msg.data))


def run_once(node: BenchNode, recorder: JogRecorder, pub, axis: int) -> dict:
    v = np.zeros(6)
    v[axis] = SPEED_MPS
    recorder.rows, recorder.recording = [], True
    node.sleep_sim(0.2)  # baseline at rest
    end = node.now_s() + STREAM_S
    while node.now_s() < end:
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = v[:3]
        pub.publish(msg)
        node.sleep_sim(1.0 / STREAM_HZ)
    node.sleep_sim(AFTER_S)
    recorder.recording = False

    data = np.asarray(recorder.rows, dtype=float)
    if data.size == 0:
        raise RuntimeError("no jog_state received")
    t, t_cmd, w = data[:, 0], data[:, 1], data[:, 2]
    v_cmd, v_meas = data[:, 5:11], data[:, 11:17]
    commanded = t_cmd > 0.0
    if not commanded.any():
        raise RuntimeError("the controller never used a command")
    t_first = t_cmd[commanded].min()
    t_last = t_cmd[commanded].max()
    direction = v[:3] / np.linalg.norm(v[:3])
    along = v_meas[:, :3] @ direction

    def first_after(t0, mask):
        hit = np.flatnonzero((t >= t0) & mask)
        return t[hit[0]] - t0 if hit.size else np.nan

    latency_50 = first_after(t_first, along >= 0.5 * SPEED_MPS)
    latency_90 = first_after(t_first, along >= 0.9 * SPEED_MPS)
    stop = first_after(t_last, np.linalg.norm(v_meas[:, :3], axis=1) < 0.01 * SPEED_MPS)
    window = commanded & (t >= t_first + STREAM_S / 2) & (t <= t_last)
    err = np.linalg.norm(v_meas[window, :3] - v_cmd[window, :3], axis=1)
    return {
        "latency_50_s": latency_50,
        "latency_90_s": latency_90,
        "tracking_rel_rms": float(np.sqrt(np.mean(err**2)) / SPEED_MPS),
        "stop_s": stop,
        "min_manipulability": float(w.min()),
        "samples": int(window.sum()),
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", required=True)
    parser.add_argument("--reps", type=int, default=3)
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args(argv)

    rclpy.init()
    node = BenchNode("bench_jog")
    node.wait_for_clock()
    rng = np.random.default_rng(args.seed)
    commit = git_commit()
    step = world_step_size()
    rate = node.get_param("controller_manager", "update_rate")
    node.switcher.ensure_configured(CONTROLLER)
    backend = node.get_param(CONTROLLER, "dynamics_backend")
    timeout = node.get_param(CONTROLLER, "command_timeout")
    recorder = JogRecorder(node)
    pub = node.create_publisher(TwistStamped, f"/{CONTROLLER}/twist", 10)

    for rep in range(args.reps):
        offset = rng.uniform(-0.05, 0.05, size=6)
        for name, axis in AXES.items():
            node.home(JOG_START + offset)
            node.switcher.activate_exclusive(CONTROLLER)
            node.sleep_sim(0.3)
            r = run_once(node, recorder, pub, axis)
            append_rows(
                args.out,
                [
                    {
                        "bench": "jog",
                        "git_commit": commit,
                        "sim_step_s": step,
                        "update_rate_hz": rate,
                        "dynamics_backend": backend,
                        "seed": args.seed,
                        "rep": rep,
                        "axis": name,
                        "speed_mps": SPEED_MPS,
                        "stream_hz": STREAM_HZ,
                        "command_timeout_s": timeout,
                        "latency_50_s": f"{r['latency_50_s']:.4f}",
                        "latency_90_s": f"{r['latency_90_s']:.4f}",
                        "tracking_rel_rms": f"{r['tracking_rel_rms']:.5f}",
                        "stop_s": f"{r['stop_s']:.4f}",
                        "min_manipulability": f"{r['min_manipulability']:.5f}",
                        "samples": r["samples"],
                    }
                ],
            )
            node.get_logger().info(
                f"rep {rep} axis {name}: latency 50/90 % {r['latency_50_s'] * 1e3:.1f} / "
                f"{r['latency_90_s'] * 1e3:.1f} ms, tracking {100 * r['tracking_rel_rms']:.2f} %, "
                f"stop {r['stop_s'] * 1e3:.1f} ms"
            )
    node.home()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
