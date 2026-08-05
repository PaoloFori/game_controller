"""Base class for nodes that decide game commands for game_bridge.

A controller subscribes to an "integrated" signal topic (by default the
integrated class probability from the acquisition/classification pipeline,
`/integrated/raw`) and, based on its own decision logic, publishes commands
on the same topic game_bridge listens to (`/game/command`, `std_msgs/String`
with content `INPUT_A`/`INPUT_B`/`INPUT_C`/`INPUT_D`).

Subclasses only need to implement `on_integrated`. A subclass that doesn't
actually drive itself from the integrated topic (see
`dummy_keyboard_controller.py`) can simply make `on_integrated` a no-op and
call `send_command` from wherever its own input comes from instead -- the
subscription still exists (so the node behaves uniformly on the graph), it's
just unused.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from rclpy.node import Node
from ros2neuro_msgs.msg import NeuroControl
from std_msgs.msg import String

VALID_COMMANDS = {"INPUT_A", "INPUT_B", "INPUT_C", "INPUT_D"}


class BaseController(Node, ABC):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.declare_parameter("integrated_topic", "/integrated/raw")
        self.declare_parameter("command_topic", "/game/command")

        integrated_topic = self.get_parameter("integrated_topic").get_parameter_value().string_value
        command_topic = self.get_parameter("command_topic").get_parameter_value().string_value

        self._cmd_pub = self.create_publisher(String, command_topic, 10)
        self.create_subscription(NeuroControl, integrated_topic, self.on_integrated, 10)

        self.get_logger().info(f"{node_name}: listening on {integrated_topic}, sending commands on {command_topic}")

    @abstractmethod
    def on_integrated(self, msg: NeuroControl) -> None:
        """Called for every message received on the integrated-signal topic."""

    def send_command(self, cmd: str) -> None:
        if cmd not in VALID_COMMANDS:
            self.get_logger().warning(f"Refusing to send unrecognised command {cmd!r}")
            return
        self._cmd_pub.publish(String(data=cmd))
