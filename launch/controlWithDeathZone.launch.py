"""Launch the two-class threshold controller (with dead zone) with
overridable thresholds. See controlNoDeadZone.launch.py for the variant with
no dead zone, whose outer thresholds trigger the integrator reset instead of
the absolute 0.0/1.0 extremes.

Usage:
    ros2 launch game_controller controlWithDeathZone.launch.py
    ros2 launch game_controller controlWithDeathZone.launch.py th_extreme_right:=0.25 th_extreme_left:=0.75
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Zones along the derived probability (values[0]/(values[0]+values[1]), see
# TwoClassThresholdController._derive_position) -- class_a (values[0]) is
# the numerator on purpose, so it dominates towards probability 1 -> wheel
# LEFT (SingleWheel::input2angle: low probability = visual RIGHT, high =
# visual LEFT). class_a must therefore be the FIRST class configured via
# the `classes` launch arg used by calibration/evaluation (classes[0] ==
# Direction::Left == the blue threshold in calibration, see
# training_controller.cpp) for this to stay consistent across
# calibration/control/evaluation -- the classifier feeding /integrated/raw
# has to publish that class's evidence as values[0].
#   probability < th_extreme_right                -> INPUT_B, wheel all the way RIGHT (class_b)
#   th_extreme_right <= probability < th_right     -> dead zone (right-of-center)
#   th_right <= probability < th_left              -> INPUT_C, wheel CENTER (up)
#   th_left <= probability < th_extreme_left        -> dead zone (left-of-center)
#   probability >= th_extreme_left                 -> INPUT_A, wheel all the way LEFT (class_a)
DEFAULT_THRESHOLDS = {
    "th_extreme_right": "0.3",  # right edge (dead zone starts here)
    "th_right": "0.4",  # center-right edge (dead zone ends, INPUT_C starts)
    "th_left": "0.6",  # center-left edge (INPUT_C ends, dead zone starts)
    "th_extreme_left": "0.7",  # left edge (dead zone ends here, INPUT_A starts)
}

DEFAULT_EXTRA_PARAMS = {
    "command_period_sec": "0.5",
    "with_reset": "false",
    "reset_service_name": "/integrator/reset",
    "control_topic": "/game_controller/control",
}


def generate_launch_description() -> LaunchDescription:
    threshold_args = [
        DeclareLaunchArgument(name, default_value=default, description=f"{name} for the two-class controller")
        for name, default in DEFAULT_THRESHOLDS.items()
    ]
    extra_args = [
        DeclareLaunchArgument(name, default_value=default, description=f"{name} for the two-class controller")
        for name, default in DEFAULT_EXTRA_PARAMS.items()
    ]

    node = Node(
        package="game_controller",
        executable="two_class_threshold_controller",
        name="two_class_threshold_controller",
        output="screen",
        parameters=[
            {name: LaunchConfiguration(name) for name in {**DEFAULT_THRESHOLDS, **DEFAULT_EXTRA_PARAMS}}
        ],
    )

    return LaunchDescription([*threshold_args, *extra_args, node])
