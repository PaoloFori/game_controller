#!/usr/bin/env python3
"""No-dead-zone threshold controller. See package README for full details."""

from __future__ import annotations

from enum import Enum

import rclpy
from rcl_interfaces.msg import SetParametersResult
from ros2neuro_msgs.msg import NeuroControl
from std_srvs.srv import Empty

from game_controller.base_controller import BaseController

# Ascending along the derived probability axis (see _derive_position): the
# "extreme" pair is closer to 0/1, the plain pair is closer to 0.5. Unlike
# TwoClassThresholdController, th_right/th_left alone bound the three
# commands (no dead zone); th_extreme_right/th_extreme_left only control
# when the integrator reset fires (see _maybe_reset).
THRESHOLD_NAMES = ("th_extreme_right", "th_right", "th_left", "th_extreme_left")
THRESHOLD_DEFAULTS = (0.3, 0.4, 0.6, 0.7)


class ControlState(Enum):
    """The three zones th_right/th_left split [0, 1] into (see _state_for_probability), no dead zone between them."""

    RIGHT = "right"
    CENTER = "center"
    LEFT = "left"


# Which command each state sends is itself a parameter (set from the
# no_dead_zone_control.xml config, see controlNoDeadZone.launch.py), not
# hardcoded, so the mapping can be retuned without touching this file.
STATE_COMMAND_PARAMS = {
    ControlState.RIGHT: "right_command",
    ControlState.CENTER: "center_command",
    ControlState.LEFT: "left_command",
}
COMMAND_PARAM_DEFAULTS = {
    "right_command": "INPUT_B",  # per pong: input_D
    "center_command": "INPUT_C",  # per pong: input_A
    "left_command": "INPUT_A",  # per pong: input_C
}

# Each state re-sends its command on its own cooldown (see _maybe_send).
# CENTER defaults much shorter than LEFT/RIGHT: it doubles as "keep going
# forward" rather than a one-shot turn, so it needs to be re-sent far more
# often for continuous motion in-game.
STATE_PERIOD_PARAMS = {
    ControlState.RIGHT: "right_command_period_sec",
    ControlState.CENTER: "center_command_period_sec",
    ControlState.LEFT: "left_command_period_sec",
}
PERIOD_PARAM_DEFAULTS = {
    "right_command_period_sec": 0.5,
    "center_command_period_sec": 0.1,
    "left_command_period_sec": 0.5,
}


class NoDeadZoneThresholdController(BaseController):
    """Same 4-threshold parameter set as TwoClassThresholdController, but with
    no dead zone: th_right/th_left alone split [0, 1] into three states (see
    ControlState) with no gap, and the outer th_extreme_right/th_extreme_left
    (instead of the absolute 0.0/1.0 extremes) are what triggers the
    integrator reset.
    """

    def __init__(self) -> None:
        super().__init__("no_dead_zone_threshold_controller")

        for name, default in zip(THRESHOLD_NAMES, THRESHOLD_DEFAULTS, strict=True):
            self.declare_parameter(name, default)
        for name, default in PERIOD_PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)
        for name, default in COMMAND_PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)
        self.declare_parameter("with_reset", False)
        self.declare_parameter("reset_service_name", "/integrator/reset")
        self.declare_parameter("control_topic", "/game_controller/control")

        error = self._validation_error(self._read_thresholds())
        if error:
            raise ValueError(error)

        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._last_sent_state: ControlState | None = None
        self._last_sent_time_ns: int | None = None
        self._reset_client = self.create_client(
            Empty, self.get_parameter("reset_service_name").get_parameter_value().string_value
        )
        self._control_pub = self.create_publisher(
            NeuroControl, self.get_parameter("control_topic").get_parameter_value().string_value, 10
        )

    def _read_thresholds(self) -> dict[str, float]:
        return {name: self.get_parameter(name).get_parameter_value().double_value for name in THRESHOLD_NAMES}

    @staticmethod
    def _validation_error(values: dict[str, float]) -> str | None:
        t_er, t_r, t_l, t_el = (values[name] for name in THRESHOLD_NAMES)
        if not (t_er < t_r <= t_l < t_el):
            return (
                "Thresholds must satisfy th_extreme_right < th_right <= th_left < th_extreme_left, "
                f"got ({t_er}, {t_r}, {t_l}, {t_el})"
            )
        return None

    def _on_set_parameters(self, params: list) -> SetParametersResult:
        candidate = self._read_thresholds()
        for param in params:
            if param.name in candidate:
                candidate[param.name] = param.value
            elif param.name in PERIOD_PARAM_DEFAULTS and param.value < 0.0:
                return SetParametersResult(successful=False, reason=f"{param.name} must be >= 0")

        error = self._validation_error(candidate)
        if error:
            return SetParametersResult(successful=False, reason=error)
        return SetParametersResult(successful=True)

    @staticmethod
    def _state_for_probability(probability: float, t: dict[str, float]) -> ControlState:
        # No dead zone: th_right/th_left alone cover [0, 1] with no gap, so
        # the controller is always in one of the three states. Same
        # class_a/class_b <-> left/right convention as TwoClassThresholdController
        # -- see that controller's comment for why class_a (values[0]) has to
        # be the numerator in _derive_position for this to line up with
        # calibration/evaluation.
        if probability < t["th_right"]:
            return ControlState.RIGHT
        if probability < t["th_left"]:
            return ControlState.CENTER
        return ControlState.LEFT

    def _maybe_send(self, state: ControlState) -> None:
        now_ns = self.get_clock().now().nanoseconds
        period_param = STATE_PERIOD_PARAMS[state]
        period_ns = int(self.get_parameter(period_param).get_parameter_value().double_value * 1e9)

        is_new_state = state != self._last_sent_state
        cooldown_elapsed = self._last_sent_time_ns is None or (now_ns - self._last_sent_time_ns) >= period_ns

        if is_new_state or cooldown_elapsed:
            command_param = STATE_COMMAND_PARAMS[state]
            command = self.get_parameter(command_param).get_parameter_value().string_value
            self.send_command(command)
            self._last_sent_state = state
            self._last_sent_time_ns = now_ns

    def _maybe_reset(self, probability: float, t: dict[str, float]) -> None:
        if not self.get_parameter("with_reset").get_parameter_value().bool_value:
            return
        if not (probability <= t["th_extreme_right"] or probability >= t["th_extreme_left"]):
            return
        if not self._reset_client.service_is_ready():
            self.get_logger().warning(
                f"Integrator reset service '{self._reset_client.srv_name}' not available, skipping reset"
            )
            return
        self._reset_client.call_async(Empty.Request())

    @staticmethod
    def _derive_position(class_a: float, class_b: float) -> float:
        # Renormalizes the integrator's two independent per-class
        # accumulators (not guaranteed to sum to 1, see package README) into
        # a single [0, 1] position -- class_a is the numerator on purpose:
        # class_a must be the class that drives the position towards 1 (see
        # _state_for_probability's comment on why).
        total = class_a + class_b
        if total <= 0.0:
            return 0.5
        return class_a / total

    def on_integrated(self, msg: NeuroControl) -> None:
        if len(msg.values) < 2:
            self.get_logger().warning("Received NeuroControl with fewer than 2 values, ignoring")
            return
        probability = self._derive_position(msg.values[0], msg.values[1])

        self._control_pub.publish(NeuroControl(header=msg.header, values=[probability]))

        thresholds = self._read_thresholds()
        self._maybe_send(self._state_for_probability(probability, thresholds))
        self._maybe_reset(probability, thresholds)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NoDeadZoneThresholdController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
