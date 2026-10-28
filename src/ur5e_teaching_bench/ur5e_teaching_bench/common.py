"""Shared plumbing for the benchmark drivers: sim-time waits, homing, parameters, recording."""

from __future__ import annotations

import csv
import os
import subprocess
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import JointTrajectoryControllerState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter as RclpyParameter
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ur5e_teaching_bench.controller_switching import ControllerSwitcher

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
# Same pose as the xacro's initial_positions: elbow up, tool pointing down.
HOME = np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])


def git_commit() -> str:
    """Commit of the repo the benchmark runs from (+'-dirty' if there are local changes)."""
    repo = os.environ.get("UR5E_TEACHING_REPO", os.getcwd())
    try:
        sha = subprocess.check_output(
            ["git", "-C", repo, "rev-parse", "--short=12", "HEAD"], text=True
        ).strip()
        dirty = subprocess.run(
            ["git", "-C", repo, "diff", "--quiet", "HEAD", "--", "src", "scripts"]
        ).returncode
        return sha + ("-dirty" if dirty else "")
    except (subprocess.CalledProcessError, FileNotFoundError):
        return "unknown"


def world_step_size(world: str = "bench.sdf") -> float:
    path = os.path.join(get_package_share_directory("ur5e_teaching_bringup"), "worlds", world)
    step = ET.parse(path).getroot().find("./world/physics/max_step_size")
    return float(step.text)


@dataclass
class Recording:
    stamps: list = field(default_factory=list)
    reference: list = field(default_factory=list)
    reference_velocity: list = field(default_factory=list)
    position: list = field(default_factory=list)
    velocity: list = field(default_factory=list)
    error: list = field(default_factory=list)
    effort: list = field(default_factory=list)
    model_effort: list = field(default_factory=list)  # reference.effort: the model torque

    def arrays(self):
        return {k: np.asarray(v, dtype=float) for k, v in self.__dict__.items()}


class BenchNode(Node):
    """rclpy node with sim time, controller switching and controller_state recording."""

    def __init__(self, name: str):
        super().__init__(name, parameter_overrides=[RclpyParameter("use_sim_time", value=True)])
        self.switcher = ControllerSwitcher(self)
        self._recording: Recording | None = None
        self._recording_topic: str | None = None
        self._subs = {}
        self._pubs = {}

    # ---- time ------------------------------------------------------------------------------
    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_for_clock(self, timeout_s: float = 30.0) -> None:
        start = time.monotonic()
        while self.now_s() == 0.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - start > timeout_s:
                raise RuntimeError("no /clock: is the sim running?")

    def sleep_sim(self, seconds: float) -> None:
        """Spin (so recordings keep filling) until `seconds` of simulated time have passed."""
        end = self.now_s() + seconds
        while self.now_s() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    # ---- parameters ------------------------------------------------------------------------
    def _param_client(self, node: str, srv_type, suffix: str):
        key = (node, suffix)
        if key not in self._subs:
            self._subs[key] = self.create_client(srv_type, f"/{node}/{suffix}")
        client = self._subs[key]
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"/{node}/{suffix} not available")
        return client

    def set_params(self, node: str, values: dict) -> None:
        params = []
        for name, value in values.items():
            if isinstance(value, bool):
                pv = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
            elif isinstance(value, float | int):
                pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
            else:
                pv = ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                    double_array_value=[float(x) for x in value],
                )
            params.append(Parameter(name=name, value=pv))
        client = self._param_client(node, SetParameters, "set_parameters")
        future = client.call_async(SetParameters.Request(parameters=params))
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        results = future.result().results if future.result() else []
        if len(results) != len(params) or not all(r.successful for r in results):
            raise RuntimeError(f"setting {values} on {node} failed: {results}")

    def get_param(self, node: str, name: str):
        client = self._param_client(node, GetParameters, "get_parameters")
        future = client.call_async(GetParameters.Request(names=[name]))
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        value = future.result().values[0]
        return {
            ParameterType.PARAMETER_INTEGER: value.integer_value,
            ParameterType.PARAMETER_DOUBLE: value.double_value,
        }.get(value.type, None)

    # ---- trajectories ----------------------------------------------------------------------
    def publish_trajectory(self, controller: str, waypoints, times) -> None:
        topic = f"/{controller}/joint_trajectory"
        if topic not in self._pubs:
            self._pubs[topic] = self.create_publisher(JointTrajectory, topic, 10)
            # give discovery a moment so the first message is not lost
            deadline = time.monotonic() + 5.0
            while self._pubs[topic].get_subscription_count() == 0:
                rclpy.spin_once(self, timeout_sec=0.05)
                if time.monotonic() > deadline:
                    raise RuntimeError(f"nobody subscribes to {topic}")
        msg = JointTrajectory(joint_names=JOINTS)
        for wp, t in zip(waypoints, times, strict=True):
            point = JointTrajectoryPoint(positions=[float(x) for x in wp])
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int(round((t - int(t)) * 1e9))
            msg.points.append(point)
        self._pubs[topic].publish(msg)

    def home(self, pose=HOME, duration_s: float = 2.0, settle_s: float = 1.0) -> None:
        """Move to `pose` with the effort tracking controller, feedforward on.

        Not with the stock position JTC: switching a joint from effort back to position in
        gz_ros2_control 3.0.9 leaves it uncommanded and the arm falls (see NOTES.md).
        """
        controller = "pid_ff_trajectory_controller"
        self.switcher.ensure_configured(controller)
        self.set_params(controller, {"feedforward": True})
        self.switcher.activate_exclusive(controller)
        self.publish_trajectory(controller, [pose], [duration_s])
        self.sleep_sim(duration_s + settle_s)

    # ---- recording -------------------------------------------------------------------------
    def start_recording(self, controller: str) -> None:
        topic = f"/{controller}/controller_state"
        if topic not in self._subs:
            self._subs[topic] = self.create_subscription(
                JointTrajectoryControllerState,
                topic,
                lambda msg, topic=topic: self._on_state(topic, msg),
                1000,
            )
        # Only this controller's messages count: during a switch the previous controller still
        # publishes for a few cycles, and its samples must not leak into this recording.
        self._recording_topic = topic
        self._recording = Recording()

    def stop_recording(self) -> dict:
        rec, self._recording = self._recording, None
        return rec.arrays()

    def _on_state(self, topic: str, msg: JointTrajectoryControllerState) -> None:
        rec = self._recording
        if rec is None or topic != self._recording_topic:
            return
        rec.stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        rec.reference.append(msg.reference.positions)
        rec.reference_velocity.append(msg.reference.velocities)
        rec.position.append(msg.feedback.positions)
        rec.velocity.append(msg.feedback.velocities)
        rec.error.append(msg.error.positions)
        rec.effort.append(msg.output.effort)
        rec.model_effort.append(msg.reference.effort)


def append_rows(path: str, rows: list[dict]) -> None:
    """Append rows to a CSV, writing the header if the file is new."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        if new:
            writer.writeheader()
        writer.writerows(rows)
