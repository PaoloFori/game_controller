"""Launch training_controller (C++ orchestrator) plus the passive wheel in
training mode, sharing `classes`/`control_topic`/`event_topic`. See package
README for the calibration/evaluation details.

Usage:
    ros2 launch game_controller training.launch.py
    ros2 launch game_controller training.launch.py modality:=evaluation classes:="[773, 771, 783]" trials:="[10, 10, 5]"
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULTS = {
    "modality": "calibration",
    "classes": "[773, 771]",
    "trials": "[10, 10]",
    "thresholds": "[0.8, 0.2]",
    "control_topic": "/game_controller/control",
    "event_topic": "/neuroevent",
    "probability_topic": "/integrated/raw",
}

WHEEL_THRESHOLD_DEFAULTS = {
    "threshold_1": "0.3",
    "threshold_2": "0.4",
    "threshold_3": "0.6",
    "threshold_4": "0.7",
}


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(name, default_value=default, description=f"{name} for the training controller")
        for name, default in DEFAULTS.items()
    ]
    wheel_threshold_args = [
        DeclareLaunchArgument(name, default_value=default, description=f"{name} for the wheel's visual markers")
        for name, default in WHEEL_THRESHOLD_DEFAULTS.items()
    ]

    training_node = Node(
        package="game_controller",
        executable="training_controller",
        name="training_controller",
        output="screen",
        parameters=[{name: LaunchConfiguration(name) for name in DEFAULTS}],
    )

    wheel_node = Node(
        package="ros2neuro_feedback_wheel",
        executable="wheel",
        name="wheel",
        output="screen",
        parameters=[
            {
                "mode": "training",
                "input_topic": LaunchConfiguration("control_topic"),
                "event_topic": LaunchConfiguration("event_topic"),
                "classes": LaunchConfiguration("classes"),
                **{name: LaunchConfiguration(name) for name in WHEEL_THRESHOLD_DEFAULTS},
            }
        ],
    )

    return LaunchDescription([*args, *wheel_threshold_args, training_node, wheel_node])
