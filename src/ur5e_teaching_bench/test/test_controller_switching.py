"""Pure-Python checks of the mode table (no ROS graph needed)."""

from ur5e_teaching_bench.controller_switching import MODES, _joint


def test_modes_map_to_the_effort_controllers():
    assert MODES == {
        "track": "pid_ff_trajectory_controller",
        "handguide": "gravity_comp_handguide_controller",
    }


def test_joint_name_is_taken_from_the_interface_name():
    assert _joint("elbow_joint/effort") == "elbow_joint"
    assert _joint("wrist_3_joint/position") == "wrist_3_joint"
