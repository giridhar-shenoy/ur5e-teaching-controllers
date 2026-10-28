"""Bring up the UR5e teaching sim: Gazebo (server-only by default), robot, controllers, bridges.

    ros2 launch ur5e_teaching_bringup sim.launch.py                    # headless, mode none
    ros2 launch ur5e_teaching_bringup sim.launch.py mode:=track        # + switch to a mode
    ros2 launch ur5e_teaching_bringup sim.launch.py payload_mass:=2.0  # actual payload in Gazebo
    ros2 launch ur5e_teaching_bringup sim.launch.py dynamics_backend:=kdl  # KDL dynamics

Why this is not a thin wrapper around ur_simulation_gz/ur_sim_control.launch.py (as first
planned): on Lyrical that launch (2.5.0) fails in two ways that cannot be fixed through its
arguments (details in NOTES.md):
  1. Its spawners pass no parameter file, and ros2_control 6.10.1 no longer hands the
     controller_manager's --params-file to controller nodes, so joint_trajectory_controller
     dies with "Length of parameter 'joints' is '0'".
  2. Its `ros_gz_sim create` starts before Gazebo serves /world/<w>/create. On a loaded machine
     the first request times out, is delivered anyway, the retry succeeds too and, with the
     hard-coded `-allow_renaming true`, a second robot `ur_0` with a second controller_manager is
     spawned on top of the first.
This launch keeps the same building blocks (ros_gz_sim's gz_sim.launch.py, robot_state_publisher,
ros_gz_sim create, the controller_manager spawner) and fixes both: every controller gets a
`params_file`, and the robot is spawned once, after the create service exists, with renaming off.
"""

import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

WORLD_NAME = "bench"
# The effort controllers that take a dynamics_backend parameter.
DYNAMICS_CONTROLLERS = ("pid_ff_trajectory_controller", "gravity_comp_handguide_controller")


def _controllers_with_params_file(src: str, dynamics_backend: str) -> str:
    """Copy the controllers YAML and point each controller at it (ros2_control 6.10.1 quirk).

    The copy also carries the dynamics_backend launch argument into both effort controllers.
    """
    with open(src) as f:
        data = yaml.safe_load(f)
    out_dir = tempfile.mkdtemp(prefix="ur5e_teaching_")
    dst = os.path.join(out_dir, "controllers.yaml")
    for value in data["controller_manager"]["ros__parameters"].values():
        if isinstance(value, dict) and "type" in value:
            value["params_file"] = dst
    for name in DYNAMICS_CONTROLLERS:
        data[name]["ros__parameters"]["dynamics_backend"] = dynamics_backend
    with open(dst, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return dst


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory("ur5e_teaching_bringup")

    def arg(name):
        return context.launch_configurations[name]

    world = arg("world")
    world_path = world if os.path.isabs(world) else os.path.join(pkg, "worlds", world)
    controllers_file = _controllers_with_params_file(
        arg("controllers_file") or os.path.join(pkg, "config", "controllers.yaml"),
        arg("dynamics_backend"),
    )
    description_file = arg("description_file") or os.path.join(
        pkg, "urdf", "ur5e_teaching.urdf.xacro"
    )
    robot_description = xacro.process_file(
        description_file,
        mappings={
            "ur_type": "ur5e",
            "simulation_controllers": controllers_file,
            "payload_mass": arg("payload_mass"),
            "payload_com_z": arg("payload_com_z"),
        },
    ).toxml()

    gui = arg("gazebo_gui").lower() == "true"
    gz_args = f"-r -v 2 {world_path}" if gui else f"-s -r -v 2 {world_path}"
    if arg("headless_rendering").lower() == "true" and not gui:
        gz_args += " --headless-rendering"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items(),
    )

    # Spawn only once Gazebo serves the create service (see module docstring, failure 2).
    wait_for_world = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            f"until gz service -l 2>/dev/null | grep -qx /world/{WORLD_NAME}/create; "
            "do sleep 0.5; done",
        ],
        name="wait_for_world",
        output="log",
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world",
            WORLD_NAME,
            "-string",
            robot_description,
            "-name",
            "ur",
            "-allow_renaming",
            "false",
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[{"use_sim_time": True, "robot_description": robot_description}],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="log",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # scripted "human push" (one step / every step until cleared), ROS -> Gazebo
            f"/world/{WORLD_NAME}/wrench@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench",
            f"/world/{WORLD_NAME}/wrench/persistent@ros_gz_interfaces/msg/EntityWrench]"
            "gz.msgs.EntityWrench",
            f"/world/{WORLD_NAME}/wrench/clear@ros_gz_interfaces/msg/Entity]gz.msgs.Entity",
            "/world_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
    )

    def spawner(name, *extra):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                name,
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                "120",
                "--param-file",
                controllers_file,
                *extra,
            ],
            output="screen",
        )

    jsb = spawner("joint_state_broadcaster")
    fts = spawner("force_torque_sensor_broadcaster")
    initial = spawner(arg("initial_joint_controller"))

    after_spawn = [jsb, fts, initial]
    mode = arg("mode")
    if mode != "none":
        mode_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "mode.launch.py")),
            launch_arguments={"mode": mode}.items(),
        )
        after_spawn.append(
            RegisterEventHandler(OnProcessExit(target_action=initial, on_exit=[mode_launch]))
        )

    return [
        gazebo,
        robot_state_publisher,
        bridge,
        wait_for_world,
        RegisterEventHandler(OnProcessExit(target_action=wait_for_world, on_exit=[spawn_robot])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=after_spawn)),
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument("gazebo_gui", default_value="false"),
        DeclareLaunchArgument(
            "launch_rviz", default_value="false", description="Accepted for CLI parity; unused."
        ),
        DeclareLaunchArgument("world", default_value="bench.sdf"),
        DeclareLaunchArgument(
            "headless_rendering",
            default_value="false",
            description="Pass --headless-rendering to gz sim (needed for camera sensors).",
        ),
        DeclareLaunchArgument("description_file", default_value=""),
        DeclareLaunchArgument("controllers_file", default_value=""),
        DeclareLaunchArgument("payload_mass", default_value="0.0"),
        DeclareLaunchArgument("payload_com_z", default_value="0.03"),
        DeclareLaunchArgument(
            "initial_joint_controller", default_value="joint_trajectory_controller"
        ),
        DeclareLaunchArgument(
            "dynamics_backend",
            default_value="pinocchio",
            choices=["pinocchio", "kdl"],
            description="Dynamics library of both effort controllers (v0.1 numbers: pinocchio).",
        ),
        DeclareLaunchArgument(
            "mode",
            default_value="none",
            choices=["none", "track", "handguide"],
            description="Controller mode to switch to once the robot is up (mode.launch.py).",
        ),
    ]
    return LaunchDescription([*args, OpaqueFunction(function=launch_setup)])
