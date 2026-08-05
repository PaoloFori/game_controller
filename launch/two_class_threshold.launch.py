"""Launch the two-class threshold controller with overridable thresholds.

Usage:
    ros2 launch game_controller two_class_threshold.launch.py
    ros2 launch game_controller two_class_threshold.launch.py threshold_1:=0.25 threshold_4:=0.75
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_THRESHOLDS = {
    "threshold_1": "0.3",
    "threshold_2": "0.4",
    "threshold_3": "0.6",
    "threshold_4": "0.7",
}


def generate_launch_description() -> LaunchDescription:
    threshold_args = [
        DeclareLaunchArgument(name, default_value=default, description=f"{name} for the two-class controller")
        for name, default in DEFAULT_THRESHOLDS.items()
    ]

    node = Node(
        package="game_controller",
        executable="two_class_threshold_controller",
        name="two_class_threshold_controller",
        output="screen",
        parameters=[{name: LaunchConfiguration(name) for name in DEFAULT_THRESHOLDS}],
    )

    return LaunchDescription([*threshold_args, node])
