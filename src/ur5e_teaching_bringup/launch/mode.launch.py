"""Switch the running sim to a controller mode (strict: conflicting controllers are stopped).

    ros2 launch ur5e_teaching_bringup mode.launch.py mode:=track
    ros2 launch ur5e_teaching_bringup mode.launch.py mode:=handguide

track     -> pid_ff_trajectory_controller (effort; send trajectories to
             /pid_ff_trajectory_controller/joint_trajectory)
handguide -> gravity_comp_handguide_controller (effort; push the arm, e.g. with
             /world/bench/wrench/persistent)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", choices=["track", "handguide"]),
            Node(
                package="ur5e_teaching_bench",
                executable="mode_manager",
                name="mode_manager",
                output="screen",
                parameters=[{"mode": LaunchConfiguration("mode"), "oneshot": True}],
            ),
        ]
    )
