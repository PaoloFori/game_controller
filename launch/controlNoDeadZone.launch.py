"""Launch the no-dead-zone threshold controller with overridable thresholds.

Same th_extreme_right/th_right/th_left/th_extreme_left parameters as
controlWithDeathZone.launch.py, but th_right/th_left alone split [0, 1]
into three commands with no gap (a command is always sent), and
th_extreme_right/th_extreme_left -- instead of the absolute 0.0/1.0
extremes -- are what triggers the integrator reset.

Usage:
    ros2 launch game_controller controlNoDeadZone.launch.py
    ros2 launch game_controller controlNoDeadZone.launch.py th_extreme_right:=0.25 th_extreme_left:=0.75
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Zones along the derived probability (values[0]/(values[0]+values[1]), see
# NoDeadZoneThresholdController._derive_position) -- class_a (values[0]) is
# the numerator on purpose, so it dominates towards probability 1 -> wheel
# LEFT. class_a must therefore be the FIRST class configured via the
# `classes` launch arg used by calibration/evaluation (classes[0] ==
# Direction::Left == the blue threshold in calibration, see
# training_controller.cpp) for this to stay consistent across
# calibration/control/evaluation. No dead zone here, a command is always
# sent, and th_extreme_right/th_extreme_left don't bound a command zone,
# they only mark when (with_reset:=true) the integrator reset fires:
#   probability < th_right                 -> INPUT_B, wheel all the way RIGHT (class_b)
#   th_right <= probability < th_left      -> INPUT_C, wheel CENTER (up)
#   probability >= th_left                 -> INPUT_A, wheel all the way LEFT (class_a)
#   probability <= th_extreme_right or >= th_extreme_left -> integrator reset
DEFAULT_THRESHOLDS = {
    "th_extreme_right": "0.3",  # right-side reset trigger
    "th_right": "0.4",  # center-right edge (INPUT_B ends, INPUT_C starts)
    "th_left": "0.6",  # center-left edge (INPUT_C ends, INPUT_A starts)
    "th_extreme_left": "0.7",  # left-side reset trigger
}

DEFAULT_EXTRA_PARAMS = {
    # RIGHT/LEFT are one-shot turns, re-sent at most once per this period
    # while the probability stays in the same zone; CENTER doubles as
    # "keep going forward" so it defaults far shorter, to re-send much more
    # often. See ControlState/_maybe_send in the node itself.
    "right_command_period_sec": "0.5",
    "center_command_period_sec": "0.1",
    "left_command_period_sec": "0.5",
    "with_reset": "false",
    "reset_service_name": "/integrator/reset",
    "control_topic": "/game_controller/control",
}

# Which command (INPUT_A..INPUT_D) each zone sends -- see
# NoDeadZoneThresholdController.STATE_COMMAND_PARAMS.
DEFAULT_COMMANDS = {
    "right_command": "INPUT_B",
    "center_command": "INPUT_C",
    "left_command": "INPUT_A",
}


def generate_launch_description() -> LaunchDescription:
    threshold_args = [
        DeclareLaunchArgument(name, default_value=default, description=f"{name} for the no-dead-zone controller")
        for name, default in DEFAULT_THRESHOLDS.items()
    ]
    extra_args = [
        DeclareLaunchArgument(name, default_value=default, description=f"{name} for the no-dead-zone controller")
        for name, default in DEFAULT_EXTRA_PARAMS.items()
    ]
    command_args = [
        DeclareLaunchArgument(
            name, default_value=default, description=f"{name} for the no-dead-zone controller's command mapping"
        )
        for name, default in DEFAULT_COMMANDS.items()
    ]

    node = Node(
        package="game_controller",
        executable="no_dead_zone_threshold_controller",
        name="no_dead_zone_threshold_controller",
        output="screen",
        parameters=[
            {
                name: LaunchConfiguration(name)
                for name in {**DEFAULT_THRESHOLDS, **DEFAULT_EXTRA_PARAMS, **DEFAULT_COMMANDS}
            }
        ],
    )

    return LaunchDescription([*threshold_args, *extra_args, *command_args, node])
