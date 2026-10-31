"""Record the README GIF takes from the fixed world camera: raw frames plus telemetry.

Frames come from the Gazebo camera sensor in bench_camera.sdf (gz sim -s --headless-rendering,
bridged to /world_camera/image). Every frame is saved with its simulated-time stamp, together with
the joint states, the scripted push, the controller's reference and the jog controller's state, so
scripts/compose_gifs.py can draw the overlays (force arrow, tool-point trace, live numbers) from
recorded data and play the GIF in simulated time even if the laptop renders slower than real time.

    ros2 run ur5e_teaching_bench record_mode --scene hero --out-dir /tmp/raw/hero

Each scene writes one directory per take, <out-dir>/<take>/{frames/NNNNN.png, take.json}:

hero      (sim launched with payload_mass:=HERO_ACTUAL_KG) hand guiding from HERO_START, two takes:
          `correct` (model payload = actual payload) and `wrong` (model payload HERO_MODEL_ERROR_KG
          lighter). The tracking controller holds the arm for HERO_LEAD_S, then hand guiding takes
          over and PUSH_FORCE_N is applied to wrist_3_link (ApplyLinkWrench) for PUSH_S; the arm
          is then let go for HERO_AFTER_S.
tracking  (payload_mass:=0.0, as bench_tracking) the bench_tracking path at 1x from HOME, two
          takes: `pid` (feedforward off, started after the integrator has settled, as in
          bench_tracking) and `pid_ff` (PID + RNEA feedforward). Same gains in both.
jogging   (payload_mass:=0.0, as bench_jog) one take: from JOG_GIF_START a TCP twist of
          JOG_SPEED_MPS is streamed on /tcp_jog_controller/twist, first along +z (well away from
          any singularity), then along -y, which turns the arm towards the wrist singularity
          (wrist_2_joint -> 0) while the tool keeps its orientation.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import cv2
import numpy as np
import pinocchio as pin
import rclpy
from geometry_msgs.msg import TwistStamped
from ros_gz_interfaces.msg import Entity, EntityWrench
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray

from ur5e_teaching_bench.bench_handguide import robot_model, to_pin
from ur5e_teaching_bench.bench_tracking import NOMINAL_SEGMENT_S, PATH_OFFSETS, VARIANTS
from ur5e_teaching_bench.common import HOME, JOINTS, BenchNode

WORLD = "bench"
TRACKER = "pid_ff_trajectory_controller"
HANDGUIDE = "gravity_comp_handguide_controller"
JOG = "tcp_jog_controller"

# ---- hero: hand guiding, correct vs wrong payload model -----------------------------------------
HERO_ACTUAL_KG = 1.0  # record_gifs.sh launches the hero sim with payload_mass:=1.0
HERO_MODEL_ERROR_KG = 1.0  # the wrong take's model payload is this much lighter than the actual
# HOME with the forearm raised 0.67 rad: the tool starts about 0.73 m up, so the wrongly modelled
# arm can sink for the whole take without the payload reaching the floor.
HERO_START = np.array([0.0, -1.57, 0.9, -0.9, -1.57, 0.0])
PUSH_LINK = "ur::wrist_3_link"  # ApplyLinkWrench applies the force at this link's origin
PUSH_FORCE_N = np.array([0.0, 10.0, 0.0])  # world frame: sideways, along +y
HERO_LEAD_S = 1.0  # held still by the tracking controller before hand guiding takes over
PUSH_S = 1.5
HERO_AFTER_S = 2.0

# ---- tracking: PID alone vs PID + feedforward ----------------------------------------------------
TRACK_LEAD_S = 0.4
TRACK_AFTER_S = 0.6

# ---- jogging: straight line, then towards the wrist singularity ----------------------------------
# Tool horizontal (wrist_1 = -pi + 0.1) and wrist_2 = -0.6: manipulability about 0.024, above the
# damping threshold w0 = 0.02 and constant while the tool moves vertically. Moving along -y at a
# fixed tool orientation then drives wrist_2 towards 0 (the wrist singularity) and the damping on.
JOG_GIF_START = np.array([0.0, -1.9, 1.9, -np.pi + 0.1, -0.6, 0.0])
JOG_SPEED_MPS = 0.05  # bench_jog's command speed
JOG_PHASES = [  # (name, unit direction in the world frame, duration s)
    ("+z", (0.0, 0.0, 1.0), 3.0),
    ("-y", (0.0, -1.0, 0.0), 6.0),
]
JOG_STREAM_HZ = 100.0
JOG_LEAD_S = 0.5
JOG_AFTER_S = 0.6

# Points the compositor projects into the camera image (frame names in the URDF).
LINK_FRAMES = [
    "base_link",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
    "tool0",
]


class Recorder(BenchNode):
    def __init__(self, out_dir: str):
        super().__init__("record_mode")
        self.out_dir = out_dir
        self.take_dir = ""
        self.recording = False
        self.frames: list[tuple[float, str]] = []
        self.joint_states: list[tuple[float, list[float], list[float]]] = []
        self.jog_states: list[list[float]] = []
        self.events: list[dict] = []
        self.images_seen = 0
        self.create_subscription(Image, "/world_camera/image", self.on_image, 5)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 100)
        self.create_subscription(Float64MultiArray, f"/{JOG}/jog_state", self.on_jog_state, 100)
        self.wrench_pub = self.create_publisher(
            EntityWrench, f"/world/{WORLD}/wrench/persistent", 10
        )
        self.clear_pub = self.create_publisher(Entity, f"/world/{WORLD}/wrench/clear", 10)
        self.twist_pub = self.create_publisher(TwistStamped, f"/{JOG}/twist", 10)
        self.model = robot_model(self)  # kinematics only (the payload does not change FK)
        self.data = self.model.createData()

    # ---- subscriptions -------------------------------------------------------------------------
    @staticmethod
    def _stamp(msg) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_image(self, msg: Image) -> None:
        self.images_seen += 1
        if not self.recording:
            return
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        name = f"{len(self.frames):05d}.png"
        cv2.imwrite(os.path.join(self.take_dir, "frames", name), frame[:, :, ::-1])
        self.frames.append((self._stamp(msg), name))

    def on_joint_state(self, msg: JointState) -> None:
        if not self.recording:
            return
        index = {n: i for i, n in enumerate(msg.name)}
        if all(j in index for j in JOINTS) and len(msg.velocity) == len(msg.name):
            q = [float(msg.position[index[j]]) for j in JOINTS]
            qd = [float(msg.velocity[index[j]]) for j in JOINTS]
            self.joint_states.append((self._stamp(msg), q, qd))

    def on_jog_state(self, msg: Float64MultiArray) -> None:
        if self.recording:
            self.jog_states.append(list(msg.data))

    # ---- actions -------------------------------------------------------------------------------
    def push(self, force) -> None:
        msg = EntityWrench()
        msg.entity.name = PUSH_LINK
        msg.entity.type = Entity.LINK
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = (float(f) for f in force)
        self.wrench_pub.publish(msg)
        self.events.append({"t": self.now_s(), "kind": "push", "force": [float(f) for f in force]})

    def clear_push(self) -> None:
        self.clear_pub.publish(Entity(name=PUSH_LINK, type=Entity.LINK))
        self.events.append({"t": self.now_s(), "kind": "push", "force": [0.0, 0.0, 0.0]})

    def stream_twist(self, linear, seconds: float) -> None:
        end = self.now_s() + seconds
        while self.now_s() < end:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = (float(v) for v in linear)
            self.twist_pub.publish(msg)
            self.sleep_sim(1.0 / JOG_STREAM_HZ)

    def wait_for_camera(self, timeout_s: float = 60.0) -> None:
        end = self.now_s() + timeout_s
        while self.images_seen < 3:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.now_s() > end:
                raise RuntimeError("no /world_camera/image: launched with world:=bench_camera.sdf?")

    # ---- takes ---------------------------------------------------------------------------------
    def begin_take(self, name: str) -> None:
        self.take_dir = os.path.join(self.out_dir, name)
        os.makedirs(os.path.join(self.take_dir, "frames"), exist_ok=True)
        self.frames, self.joint_states, self.jog_states, self.events = [], [], [], []
        self.recording = True

    def fk_points(self, q_ctrl) -> dict:
        pin.framesForwardKinematics(self.model, self.data, to_pin(self.model, np.asarray(q_ctrl)))
        pts = {f: self.data.oMf[self.model.getFrameId(f)].translation.tolist() for f in LINK_FRAMES}
        tool = self.data.oMf[self.model.getFrameId("tool0")]
        pts["tool_axis"] = tool.rotation[:, 2].tolist()
        # Bottom of the 6 cm payload cube (only drawn in the hero sim; harmless elsewhere).
        pts["payload_bottom"] = tool.act(np.array([0.0, 0.0, 0.06])).tolist()
        return pts

    def end_take(self, t0: float, meta: dict, series: dict | None = None) -> None:
        self.recording = False
        if not self.frames or not self.joint_states:
            raise RuntimeError(f"take {self.take_dir}: no frames or no joint states recorded")
        js_t = np.array([s[0] for s in self.joint_states])
        js_q = np.array([s[1] for s in self.joint_states])
        js_qd = np.array([s[2] for s in self.joint_states])
        frames = []
        for stamp, name in self.frames:
            k = max(int(np.searchsorted(js_t, stamp, side="right")) - 1, 0)
            push = [0.0, 0.0, 0.0]
            for e in self.events:
                if e["kind"] == "push" and e["t"] <= stamp:
                    push = e["force"]
            frames.append(
                {
                    "file": name,
                    "t": stamp - t0,
                    "q": js_q[k].tolist(),
                    "qd": js_qd[k].tolist(),
                    "points": self.fk_points(js_q[k]),
                    "push_force": push,
                }
            )
        # Tool-point path at <= 50 Hz for the jog trace.
        trace = []
        last = -1.0
        for stamp, q in zip(js_t, js_q, strict=True):
            if stamp - last >= 0.02:
                trace.append([stamp - t0, *self.fk_points(q)["tool0"]])
                last = stamp
        out = {
            "meta": meta,
            "frames": frames,
            "trace": trace,
            "events": [{**e, "t": e["t"] - t0} for e in self.events],
            "series": series or {},
        }
        if self.jog_states:
            # ~/jog_state: [t, t_cmd, w, lambda, scale, v_cmd(6), v_meas(6)]
            jog = np.array(self.jog_states)
            out["series"]["jog"] = {
                "t": (jog[:, 0] - t0).tolist(),
                "manipulability": jog[:, 2].tolist(),
                "damping": jog[:, 3].tolist(),
                "scale": jog[:, 4].tolist(),
                "v_cmd": jog[:, 5:8].tolist(),
                "v_meas": jog[:, 11:14].tolist(),
            }
        with open(os.path.join(self.take_dir, "take.json"), "w") as f:
            json.dump(out, f)
        self.get_logger().info(f"{len(frames)} frames -> {self.take_dir}")


def home_verified(node: Recorder, pose, duration_s: float = 2.0, settle_s: float = 1.0) -> None:
    """BenchNode.home(), then check that the arm got there and send the move again if not: the
    first message on a freshly created publisher can be lost even after discovery has matched
    (seen once here: the jog take started from HOME instead of its start pose)."""
    pose = np.asarray(pose, float)
    for _ in range(3):
        node.home(pose, duration_s=duration_s, settle_s=settle_s)
        node.start_recording(TRACKER)
        node.sleep_sim(0.05)
        rec = node.stop_recording()
        if len(rec["position"]) and np.max(np.abs(rec["position"][-1] - pose)) < 0.02:
            return
        node.get_logger().warning(f"not at {np.round(pose, 3)} after homing; sending it again")
    raise RuntimeError(f"could not move the arm to {pose}")


def record_hero(node: Recorder) -> None:
    models = {"correct": HERO_ACTUAL_KG, "wrong": HERO_ACTUAL_KG - HERO_MODEL_ERROR_KG}
    node.switcher.ensure_configured(HANDGUIDE)
    node.set_params(TRACKER, {"payload_mass": HERO_ACTUAL_KG})
    for take, model_kg in models.items():
        home_verified(node, HERO_START, settle_s=1.5)
        node.set_params(HANDGUIDE, {"payload_mass": model_kg})
        node.begin_take(take)
        node.sleep_sim(HERO_LEAD_S)
        node.switcher.activate_exclusive(HANDGUIDE)
        t0 = node.now_s()
        node.push(PUSH_FORCE_N)
        node.sleep_sim(PUSH_S)
        node.clear_push()
        node.sleep_sim(HERO_AFTER_S)
        meta = {
            "scene": "hero",
            "take": take,
            "actual_payload_kg": HERO_ACTUAL_KG,
            "model_payload_kg": model_kg,
            "push_force_n": PUSH_FORCE_N.tolist(),
            "push_link": PUSH_LINK,
            "push_s": PUSH_S,
            "start_pose": HERO_START.tolist(),
            "t_start": -HERO_LEAD_S,
            "t_end": PUSH_S + HERO_AFTER_S,
        }
        node.end_take(t0, meta)
        node.clear_push()


def record_tracking(node: Recorder) -> None:
    node.set_params(TRACKER, {"payload_mass": 0.0})
    waypoints = [HOME + p for p in PATH_OFFSETS]
    times = [NOMINAL_SEGMENT_S * (k + 1) for k in range(len(waypoints))]
    for take in ("pid", "pid_ff"):
        params, settle = VARIANTS[take]
        home_verified(node, HOME)
        node.set_params(TRACKER, params)
        node.sleep_sim(settle)
        node.start_recording(TRACKER)
        node.begin_take(take)
        node.sleep_sim(TRACK_LEAD_S)
        node.publish_trajectory(TRACKER, waypoints, times)
        node.sleep_sim(times[-1] + TRACK_AFTER_S + 0.2)
        rec = node.stop_recording()
        moving = np.any(np.abs(rec["reference_velocity"]) > 1e-9, axis=1)
        if not moving.any():
            raise RuntimeError("the reference never moved: trajectory not received?")
        t0 = float(rec["stamps"][np.argmax(moving)])
        # Worst-joint error from the recorded reference and measured joint positions.
        err = np.max(np.abs(rec["reference"] - rec["position"]), axis=1)
        series = {
            "tracking": {"t": (rec["stamps"] - t0).tolist(), "worst_abs_err_rad": err.tolist()}
        }
        meta = {
            "scene": "tracking",
            "take": take,
            "params": params,
            "segment_s": NOMINAL_SEGMENT_S,
            "path_offsets": PATH_OFFSETS.tolist(),
            "start_pose": HOME.tolist(),
            "t_start": -TRACK_LEAD_S,
            "t_end": times[-1] + TRACK_AFTER_S,
        }
        node.end_take(t0, meta, series)


def record_jogging(node: Recorder) -> None:
    node.switcher.ensure_configured(JOG)
    home_verified(node, JOG_GIF_START, duration_s=3.0)
    node.switcher.activate_exclusive(JOG)
    node.sleep_sim(0.3)
    node.begin_take("jog")
    node.sleep_sim(JOG_LEAD_S)
    t0 = node.now_s()
    phases = []
    for name, direction, seconds in JOG_PHASES:
        start = node.now_s() - t0
        node.stream_twist(JOG_SPEED_MPS * np.asarray(direction), seconds)
        phases.append(
            {"name": name, "direction": list(direction), "t0": start, "t1": node.now_s() - t0}
        )
    node.sleep_sim(JOG_AFTER_S)
    meta = {
        "scene": "jogging",
        "take": "jog",
        "speed_mps": JOG_SPEED_MPS,
        "phases": phases,
        "start_pose": JOG_GIF_START.tolist(),
        "manipulability_threshold": node.get_param(JOG, "manipulability_threshold"),
        "max_damping": node.get_param(JOG, "max_damping"),
        "t_start": -JOG_LEAD_S,
        "t_end": phases[-1]["t1"] + JOG_AFTER_S,
    }
    node.end_take(t0, meta)


SCENES = {"hero": record_hero, "tracking": record_tracking, "jogging": record_jogging}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scene", choices=list(SCENES), required=True)
    parser.add_argument("--out-dir", required=True)
    args = parser.parse_args(argv)
    rclpy.init()
    node = Recorder(args.out_dir)
    node.wait_for_clock()
    node.wait_for_camera()
    for controller in (TRACKER, HANDGUIDE, JOG):
        node.switcher.ensure_configured(controller)
    try:
        SCENES[args.scene](node)
    finally:
        node.clear_push()
    node.home(HOME)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
