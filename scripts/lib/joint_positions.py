#!/usr/bin/env python3
"""Print one /joint_states sample as six positions in UR joint order (shoulder_pan ... wrist_3).

/joint_states lists joints in whatever order the broadcaster uses (on this sim: elbow first), so
anything that compares against a goal has to map by name.

    python3 scripts/lib/joint_positions.py [timeout_s]
"""

import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def main() -> int:
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rclpy.init()
    node = Node("smoke_joint_positions")
    got: list[JointState] = []
    node.create_subscription(JointState, "/joint_states", got.append, 10)
    end = time.monotonic() + timeout  # wall clock on purpose: /clock may not be running yet
    while not got and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if not got:
        print("no /joint_states message", file=sys.stderr)
        return 1
    by_name = dict(zip(got[0].name, got[0].position, strict=True))
    print(" ".join(f"{by_name[j]:.9f}" for j in JOINTS))
    return 0


if __name__ == "__main__":
    sys.exit(main())
