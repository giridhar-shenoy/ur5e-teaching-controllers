"""Switch the UR5e teaching sim between controller modes.

    ros2 run ur5e_teaching_bench mode_manager --ros-args -p mode:=handguide
    ros2 launch ur5e_teaching_bringup mode.launch.py mode:=track

With oneshot:=true (default) the node switches once and exits with 0 on success, 1 on failure.
With oneshot:=false it stays up and switches again whenever the `mode` parameter is set.
"""

from __future__ import annotations

import sys

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

from ur5e_teaching_bench.controller_switching import MODES, ControllerSwitcher, SwitchError


class ModeManager(Node):
    def __init__(self):
        super().__init__("mode_manager")
        self.declare_parameter("mode", "track")
        self.declare_parameter("oneshot", True)
        self.declare_parameter("controller_manager", "/controller_manager")
        self.switcher = ControllerSwitcher(self, self.get_parameter("controller_manager").value)

    def switch(self, mode: str) -> bool:
        if mode not in MODES:
            self.get_logger().error(f"unknown mode '{mode}', expected one of {sorted(MODES)}")
            return False
        target = MODES[mode]
        try:
            stopped = self.switcher.activate_exclusive(target)
        except SwitchError as e:
            self.get_logger().error(str(e))
            return False
        self.get_logger().info(
            f"mode '{mode}': activated {target}, deactivated {stopped or 'nothing'}; "
            f"active now: {self.switcher.active()}"
        )
        return True

    def on_parameters(self, params) -> SetParametersResult:
        for p in params:
            if p.name == "mode" and p.value not in MODES:
                return SetParametersResult(successful=False, reason=f"unknown mode {p.value}")
        # Switch after the parameter is accepted (cannot spin inside this callback).
        for p in params:
            if p.name == "mode":
                self._pending = p.value
        return SetParametersResult(successful=True)


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = ModeManager()
    ok = node.switch(node.get_parameter("mode").value)
    if node.get_parameter("oneshot").value:
        node.destroy_node()
        rclpy.shutdown()
        return 0 if ok else 1
    node._pending = None
    node.add_on_set_parameters_callback(node.on_parameters)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node._pending is not None:
                mode, node._pending = node._pending, None
                node.switch(mode)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
