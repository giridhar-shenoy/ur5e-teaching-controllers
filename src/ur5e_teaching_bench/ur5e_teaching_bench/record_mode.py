"""Record one mode demo from the fixed world camera, with the mode name and a live metric burned in.

Frames come from the Gazebo camera sensor in bench_camera.sdf (gz sim -s --headless-rendering,
bridged to /world_camera/image) and are kept at a fixed simulated-time interval, so the GIF
plays in real (simulated) time even if the laptop renders slower than real time.

    ros2 run ur5e_teaching_bench record_mode --mode track --out-dir /tmp/frames_track

track:     the bench_tracking path at 1x; overlay = worst-joint |q_des - q| (mrad)
handguide: the arm floats under gravity compensation while a scripted push (ApplyLinkWrench on
           wrist_3_link) moves it around; overlay = applied force (N) and TCP speed (mm/s)
"""

from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np
import rclpy
from ros_gz_interfaces.msg import Entity, EntityWrench
from sensor_msgs.msg import Image

from ur5e_teaching_bench.bench_tracking import NOMINAL_SEGMENT_S, PATH_OFFSETS
from ur5e_teaching_bench.common import HOME, BenchNode

WORLD = "bench"
PUSH_LINK = "ur::wrist_3_link"
PAYLOAD_KG = 1.0  # record_gifs.sh launches the camera world with this actual payload
# (start s, duration s, force xyz N) of the scripted push for the hand-guiding demo
PUSHES = [
    (0.8, 1.2, (0.0, 10.0, 0.0)),
    (3.4, 1.2, (-10.0, 0.0, 0.0)),
    (6.0, 1.2, (0.0, -10.0, 0.0)),
]


class Recorder(BenchNode):
    def __init__(self, out_dir: str, fps: float):
        super().__init__("record_mode")
        self.out_dir = out_dir
        self.period = 1.0 / fps
        self.next_t = None
        self.count = 0
        self.label = ""
        self.metric = ""
        os.makedirs(out_dir, exist_ok=True)
        self.create_subscription(Image, "/world_camera/image", self.on_image, 5)
        self.wrench_pub = self.create_publisher(
            EntityWrench, f"/world/{WORLD}/wrench/persistent", 10
        )
        self.clear_pub = self.create_publisher(Entity, f"/world/{WORLD}/wrench/clear", 10)
        self.recording = False

    def on_image(self, msg: Image) -> None:
        if not self.recording:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.next_t is None:
            self.next_t = t
        if t + 1e-6 < self.next_t:
            return
        self.next_t += self.period
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        h = frame.shape[0]
        cv2.rectangle(frame, (0, h - 54), (frame.shape[1], h), (20, 20, 20), -1)
        cv2.putText(
            frame,
            self.label,
            (12, h - 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (240, 240, 240),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            self.metric,
            (12, h - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
            cv2.LINE_AA,
        )
        cv2.imwrite(os.path.join(self.out_dir, f"{self.count:05d}.png"), frame)
        self.count += 1

    def push(self, force) -> None:
        msg = EntityWrench()
        msg.entity.name = PUSH_LINK
        msg.entity.type = Entity.LINK
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = (float(f) for f in force)
        self.wrench_pub.publish(msg)

    def clear(self) -> None:
        self.clear_pub.publish(Entity(name=PUSH_LINK, type=Entity.LINK))


def record_track(node: Recorder, duration: float) -> None:
    node.set_params("pid_ff_trajectory_controller", {"payload_mass": PAYLOAD_KG})
    node.home(HOME)
    node.label = "mode: track - PID + Pinocchio RNEA feedforward"
    node.start_recording("pid_ff_trajectory_controller")
    waypoints = [HOME + p for p in PATH_OFFSETS]
    times = [NOMINAL_SEGMENT_S * (k + 1) for k in range(len(waypoints))]
    node.recording = True
    node.publish_trajectory("pid_ff_trajectory_controller", waypoints, times)
    end = node.now_s() + duration
    while node.now_s() < end:
        rclpy.spin_once(node, timeout_sec=0.01)
        err = node._recording.error
        if err:
            node.metric = f"worst-joint |q_des - q| = {1e3 * np.max(np.abs(err[-1])):.2f} mrad"
    node.recording = False


def record_handguide(node: Recorder, duration: float) -> None:
    import pinocchio as pin

    from ur5e_teaching_bench.bench_handguide import robot_model, to_pin

    model = robot_model(node)
    data = model.createData()
    tool = model.getFrameId("tool0")
    node.home(HOME)
    node.set_params("gravity_comp_handguide_controller", {"payload_mass": PAYLOAD_KG})
    node.start_recording("gravity_comp_handguide_controller")
    node.switcher.activate_exclusive("gravity_comp_handguide_controller")
    node.label = "mode: handguide - gravity comp + damping"
    node.recording = True
    t0 = node.now_s()
    active = None
    while node.now_s() < t0 + duration:
        rclpy.spin_once(node, timeout_sec=0.01)
        t = node.now_s() - t0
        current = next((p for p in PUSHES if p[0] <= t < p[0] + p[1]), None)
        if current is not active:
            node.clear() if current is None else node.push(current[2])
            active = current
        force = np.linalg.norm(current[2]) if current else 0.0
        rec = node._recording
        speed = 0.0
        if rec.position:
            q = to_pin(model, np.asarray(rec.position[-1]))
            v = to_pin(model, np.asarray(rec.velocity[-1]))
            J = pin.computeFrameJacobian(model, data, q, tool, pin.LOCAL_WORLD_ALIGNED)
            speed = np.linalg.norm((J @ v)[:3])
        node.metric = f"push at wrist {force:4.1f} N | TCP speed {1e3 * speed:5.1f} mm/s"
    node.clear()
    node.recording = False


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--mode", choices=["track", "handguide"], required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--duration", type=float, default=9.0)
    parser.add_argument("--fps", type=float, default=10.0)
    args = parser.parse_args(argv)
    rclpy.init()
    node = Recorder(args.out_dir, args.fps)
    node.wait_for_clock()
    node.switcher.ensure_configured("pid_ff_trajectory_controller")
    node.switcher.ensure_configured("gravity_comp_handguide_controller")
    (record_track if args.mode == "track" else record_handguide)(node, args.duration)
    node.home(HOME)
    node.get_logger().info(f"{node.count} frames -> {args.out_dir}")
    ok = node.count > 0
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
