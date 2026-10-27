"""Strict, conflict-aware controller switching through the controller_manager services."""

from __future__ import annotations

import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)
from rclpy.node import Node

# Modes exposed by mode.launch.py / mode_manager, and the controller each one activates.
# There is deliberately no way back to the position-interface JTC: with gz_ros2_control 3.0.9 a
# joint switched from effort back to position is left without any command and the arm falls
# (observed 2026-10-28, see NOTES.md). Homing therefore uses the effort tracking controller.
MODES = {
    "track": "pid_ff_trajectory_controller",
    "handguide": "gravity_comp_handguide_controller",
}


class SwitchError(RuntimeError):
    pass


def _joint(interface: str) -> str:
    return interface.split("/")[0]


class ControllerSwitcher:
    """Activate one controller and deactivate every active controller that commands its joints.

    "Strict" here means two things: the switch request uses STRICT strictness (all or nothing),
    and any active controller that claims a command interface on one of the target's joints is
    deactivated in the same request, even if it claims a different interface type (position vs
    effort on the same joint would otherwise both be accepted by the resource manager).
    """

    def __init__(self, node: Node, controller_manager: str = "/controller_manager"):
        self._node = node
        cm = controller_manager.rstrip("/")
        self._list = node.create_client(ListControllers, f"{cm}/list_controllers")
        self._load = node.create_client(LoadController, f"{cm}/load_controller")
        self._configure = node.create_client(ConfigureController, f"{cm}/configure_controller")
        self._switch = node.create_client(SwitchController, f"{cm}/switch_controller")

    def _call(self, client, request, timeout_s: float = 10.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise SwitchError(f"service {client.srv_name} not available")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            raise SwitchError(f"service {client.srv_name} did not answer in {timeout_s} s")
        return future.result()

    def controllers(self) -> dict:
        return {c.name: c for c in self._call(self._list, ListControllers.Request()).controller}

    def ensure_configured(self, name: str) -> None:
        states = self.controllers()
        if name not in states:
            if not self._call(self._load, LoadController.Request(name=name)).ok:
                raise SwitchError(f"could not load {name}")
            states = self.controllers()
        if states[name].state == "unconfigured":
            if not self._call(self._configure, ConfigureController.Request(name=name)).ok:
                raise SwitchError(f"could not configure {name}")

    def activate_exclusive(self, name: str, timeout_s: float = 10.0) -> list[str]:
        """Activate `name`; returns the controllers that were deactivated to make room."""
        # Every mode controller is loaded and configured first: the controller_manager refuses to
        # activate a controller whose fallback_controllers are not at least inactive.
        for controller in MODES.values():
            self.ensure_configured(controller)
        states = self.controllers()
        target = states[name]
        if target.state == "active":
            return []
        joints = {_joint(i) for i in target.required_command_interfaces}
        conflicting = [
            c.name
            for c in states.values()
            if c.state == "active"
            and c.name != name
            and any(_joint(i) in joints for i in c.claimed_interfaces)
        ]
        request = SwitchController.Request(
            activate_controllers=[name],
            deactivate_controllers=conflicting,
            strictness=SwitchController.Request.STRICT,
            activate_asap=True,
            timeout=Duration(sec=int(timeout_s)),
        )
        response = self._call(self._switch, request, timeout_s + 5.0)
        if not response.ok:
            raise SwitchError(f"switch to {name} failed: {response.message}")
        return conflicting

    def active(self) -> list[str]:
        return sorted(n for n, c in self.controllers().items() if c.state == "active")
