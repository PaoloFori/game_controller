#!/usr/bin/env python3
"""Two-class threshold controller. See package README for full details."""

from __future__ import annotations

import rclpy
from rcl_interfaces.msg import SetParametersResult
from ros2neuro_msgs.msg import NeuroControl
from std_srvs.srv import Empty

from game_controller.base_controller import BaseController

THRESHOLD_NAMES = ("threshold_1", "threshold_2", "threshold_3", "threshold_4")
THRESHOLD_DEFAULTS = (0.3, 0.4, 0.6, 0.7)


class TwoClassThresholdController(BaseController):
    def __init__(self) -> None:
        super().__init__("two_class_threshold_controller")

        for name, default in zip(THRESHOLD_NAMES, THRESHOLD_DEFAULTS, strict=True):
            self.declare_parameter(name, default)
        self.declare_parameter("command_period_sec", 0.5)
        self.declare_parameter("with_reset", False)
        self.declare_parameter("reset_service_name", "/integrator/reset")
        self.declare_parameter("control_topic", "/game_controller/control")

        error = self._validation_error(self._read_thresholds())
        if error:
            raise ValueError(error)

        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._last_sent_command: str | None = None
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
        t1, t2, t3, t4 = (values[name] for name in THRESHOLD_NAMES)
        if not (t1 < t2 <= t3 < t4):
            return (
                "Thresholds must satisfy threshold_1 < threshold_2 <= threshold_3 < threshold_4, "
                f"got ({t1}, {t2}, {t3}, {t4})"
            )
        return None

    def _on_set_parameters(self, params: list) -> SetParametersResult:
        candidate = self._read_thresholds()
        for param in params:
            if param.name in candidate:
                candidate[param.name] = param.value
            elif param.name == "command_period_sec" and param.value < 0.0:
                return SetParametersResult(successful=False, reason="command_period_sec must be >= 0")

        error = self._validation_error(candidate)
        if error:
            return SetParametersResult(successful=False, reason=error)
        return SetParametersResult(successful=True)

    @staticmethod
    def _command_for_probability(probability: float, t: dict[str, float]) -> str | None:
        # Swapped vs. the "naive" low->A/high->B mapping: the wheel node
        # places low probability (classes[0] dominant) on the visual right
        # and high probability (classes[1] dominant) on the visual left
        # (SingleWheel::input2angle + neurodraw's Shape::rotate use the
        # standard math convention, 0deg = +x/right, increasing
        # counterclockwise towards left) -- so low probability must map to
        # INPUT_B ("rotate right" in Brainski2, see PROTOCOL.md) and high
        # probability to INPUT_A ("rotate left") for the wheel's visual side
        # to match the direction the game actually turns.
        if probability < t["threshold_1"]:
            return "INPUT_B" # right
        if probability < t["threshold_2"]:
            return None
        if probability < t["threshold_3"]:
            return "INPUT_C" # up
        if probability < t["threshold_4"]:
            return None
        return "INPUT_A" # left

    def _maybe_send(self, command: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        period_ns = int(self.get_parameter("command_period_sec").get_parameter_value().double_value * 1e9)

        is_new_command = command != self._last_sent_command
        cooldown_elapsed = self._last_sent_time_ns is None or (now_ns - self._last_sent_time_ns) >= period_ns

        if is_new_command or cooldown_elapsed:
            self.send_command(command)
            self._last_sent_command = command
            self._last_sent_time_ns = now_ns

    def _maybe_reset(self, probability: float) -> None:
        if not self.get_parameter("with_reset").get_parameter_value().bool_value:
            return
        if not (probability <= 0.0 or probability >= 1.0):
            return
        if not self._reset_client.service_is_ready():
            self.get_logger().warning(
                f"Integrator reset service '{self._reset_client.srv_name}' not available, skipping reset"
            )
            return
        self._reset_client.call_async(Empty.Request())

    @staticmethod
    def _derive_position(class_a: float, class_b: float) -> float:
        total = class_a + class_b
        if total <= 0.0:
            return 0.5
        return class_b / total

    def on_integrated(self, msg: NeuroControl) -> None:
        if len(msg.values) < 2:
            self.get_logger().warning("Received NeuroControl with fewer than 2 values, ignoring")
            return
        probability = self._derive_position(msg.values[0], msg.values[1])

        self._control_pub.publish(NeuroControl(header=msg.header, values=[probability]))

        command = self._command_for_probability(probability, self._read_thresholds())
        if command is not None:
            self._maybe_send(command)

        self._maybe_reset(probability)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TwoClassThresholdController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
