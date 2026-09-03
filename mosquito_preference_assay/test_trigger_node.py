#!/usr/bin/env python3
"""Publish the std_msgs/Bool trigger that starts (or aborts) a triggered assay
run -- for bench testing without the real tracking nodes.

    # interactive (run from a terminal):
    ros2 run mosquito_preference_assay test_trigger \\
        --ros-args -p topic:=/arena/mosquito_present
      [Enter] -> fire (true)   a[Enter] -> abort (false)   q[Enter] -> quit

    # timed (for scripts / launch, where stdin is not a terminal):
    ros2 run mosquito_preference_assay test_trigger --ros-args \\
        -p topic:=/arena/mosquito_present -p mode:=timer -p delay_sec:=3.0

Parameters:
    topic       string  /stimulus_publisher/trigger   where to publish
    mode        string  keypress                      keypress | timer
    delay_sec   double  2.0        (timer) fire once this long after startup
    repeat_sec  double  0.0        (timer) if > 0, keep firing every this many s
"""

import sys
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class TestTrigger(Node):

    def __init__(self):
        super().__init__("test_trigger")
        topic = str(self.declare_parameter("topic", "/stimulus_publisher/trigger").value)
        self._mode = str(self.declare_parameter("mode", "keypress").value).strip()
        self._delay_sec = float(self.declare_parameter("delay_sec", 2.0).value)
        self._repeat_sec = float(self.declare_parameter("repeat_sec", 0.0).value)

        self._pub = self.create_publisher(Bool, topic, 10)
        self.get_logger().info(f"test_trigger -> std_msgs/Bool on '{topic}'")

        if self._mode == "timer":
            self._once = self.create_timer(max(0.01, self._delay_sec), self._fire_once)
        else:
            self.get_logger().info(
                "keypress mode: [Enter] fire  a[Enter] abort  q[Enter] quit"
            )
            threading.Thread(target=self._stdin_loop, daemon=True).start()

    def _send(self, value):
        self._pub.publish(Bool(data=value))
        self.get_logger().info(f"published {value}")

    def _fire_once(self):
        self._once.cancel()
        self._send(True)
        if self._repeat_sec > 0:
            self.create_timer(self._repeat_sec, lambda: self._send(True))

    def _stdin_loop(self):
        for line in sys.stdin:
            cmd = line.strip().lower()
            if cmd in ("", "t", "go", "fire", "true", "1"):
                self._send(True)
            elif cmd in ("a", "abort", "f", "false", "0", "stop"):
                self._send(False)
            elif cmd in ("q", "quit", "exit"):
                break
        rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TestTrigger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
