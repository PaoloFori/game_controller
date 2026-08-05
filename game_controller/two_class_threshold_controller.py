"""Two-class threshold controller.

Maps a single integrated class probability (`msg.values[0]` of the
`/integrated/raw` NeuroControl message) to a game command using four
thresholds, splitting [0, 1] into five zones:

    prob < threshold_1                  -> INPUT_A
    threshold_1 <= prob < threshold_2   -> (dead zone, no command)
    threshold_2 <= prob < threshold_3   -> INPUT_C (forward)
    threshold_3 <= prob < threshold_4   -> (dead zone, no command)
    prob >= threshold_4                 -> INPUT_B

Defaults: threshold_1=0.3, threshold_2=0.4, threshold_3=0.6, threshold_4=0.7.
All four are ROS2 parameters, overridable at launch time (see the launch
file / config) and live at runtime via `ros2 param set` or `rqt_reconfigure`
-- every set is validated (threshold_1 < threshold_2 <= threshold_3 <
threshold_4) and rejected if it would break that ordering.
"""

from __future__ import annotations

import rclpy
from rcl_interfaces.msg import SetParametersResult
from ros2neuro_msgs.msg import NeuroControl

from game_controller.base_controller import BaseController

THRESHOLD_NAMES = ("threshold_1", "threshold_2", "threshold_3", "threshold_4")
THRESHOLD_DEFAULTS = (0.3, 0.4, 0.6, 0.7)


class TwoClassThresholdController(BaseController):
    def __init__(self) -> None:
        super().__init__("two_class_threshold_controller")

        for name, default in zip(THRESHOLD_NAMES, THRESHOLD_DEFAULTS, strict=True):
            self.declare_parameter(name, default)

        error = self._validation_error(self._read_thresholds())
        if error:
            raise ValueError(error)

        self.add_on_set_parameters_callback(self._on_set_parameters)

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

        error = self._validation_error(candidate)
        if error:
            return SetParametersResult(successful=False, reason=error)
        return SetParametersResult(successful=True)

    def on_integrated(self, msg: NeuroControl) -> None:
        if not msg.values:
            self.get_logger().warning("Received NeuroControl with no values, ignoring")
            return
        probability = msg.values[0]
        t = self._read_thresholds()

        if probability < t["threshold_1"]:
            self.send_command("INPUT_A")
        elif probability < t["threshold_2"]:
            pass
        elif probability < t["threshold_3"]:
            self.send_command("INPUT_C")
        elif probability < t["threshold_4"]:
            pass
        else:
            self.send_command("INPUT_B")


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
