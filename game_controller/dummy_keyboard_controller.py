#!/usr/bin/env python3
"""Dummy controller driven by arrow keys instead of /integrated/raw.

Useful to exercise game_bridge (and the game itself) without a real
acquisition/classification pipeline running. Mapping:

    Left arrow  -> INPUT_A
    Right arrow -> INPUT_B
    Up arrow    -> INPUT_C
    Down arrow  -> INPUT_D

Must be run with `ros2 run` in an interactive terminal, NOT `ros2 launch`:
`ros2 launch` does not connect child processes to a real terminal on stdin
(same reason ROS2's own `teleop_twist_keyboard` warns against launching it),
so reading raw keypresses via `termios` fails there. `ros2 run` inherits the
calling shell's stdin directly, so it works.
"""

from __future__ import annotations

import sys
import termios
import threading
import tty

import rclpy
from ros2neuro_msgs.msg import NeuroControl

from game_controller.base_controller import BaseController

# Final byte of the "ESC [ <byte>" escape sequence for each arrow key.
_ARROW_TO_COMMAND = {
    "D": "INPUT_A",  # left
    "C": "INPUT_B",  # right
    "A": "INPUT_C",  # up
    "B": "INPUT_D",  # down
}


class DummyKeyboardController(BaseController):
    def __init__(self) -> None:
        super().__init__("dummy_keyboard_controller")

        self._fd = sys.stdin.fileno()
        try:
            self._old_termios = termios.tcgetattr(self._fd)
        except termios.error as exc:
            err = (
                "stdin isn't a real terminal, so raw keypresses can't be read. "
                "This node needs `ros2 run game_controller dummy_keyboard_controller` "
                "in an interactive shell -- `ros2 launch` doesn't give child "
                "processes a terminal on stdin."
            )
            raise RuntimeError(err) from exc
        tty.setcbreak(self._fd)

        self._running = True
        self._keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._keyboard_thread.start()

        self.get_logger().info(
            "Dummy keyboard controller ready: <-  A   ->  B   ^  C   v  D   (Ctrl+C to quit)"
        )

    def on_integrated(self, msg: NeuroControl) -> None:
        pass  # driven by the keyboard, not by /integrated/raw

    def _keyboard_loop(self) -> None:
        while self._running:
            ch = sys.stdin.read(1)
            if ch != "\x1b":
                continue
            bracket = sys.stdin.read(1)
            arrow = sys.stdin.read(1)
            if bracket == "[" and arrow in _ARROW_TO_COMMAND:
                self.send_command(_ARROW_TO_COMMAND[arrow])

    def destroy_node(self) -> None:
        self._running = False
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_termios)
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        node = DummyKeyboardController()
    except RuntimeError:
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
