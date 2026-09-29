#!/usr/bin/env python3
"""Print which stimulus is on which side, live, while a run is happening.

    python3 tools/watch_stimuli.py

Run it in a spare terminal alongside the experiment to check by eye that what
is on the projector matches what the assay thinks it is showing.

    armed     LEFT=blank             RIGHT=telescope_inward
    running   LEFT=blank             RIGHT=telescope_inward   trial_seed=2873312514
    complete  LEFT=blank             RIGHT=telescope_inward

With `stimuli_when_armed: true` the sides are known while still ARMED -- before
anything triggers -- so you can stand at the arena, look at the projection and
confirm the sides before an animal is ever introduced.

Prints only on change, so it stays readable next to a 10 Hz heartbeat.

(`ros2 topic echo /stimulus_publisher/stimulus_state` shows the same thing as
raw JSON; this just makes it legible.)
"""

import argparse
import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


class Watcher(Node):
    def __init__(self, topic):
        super().__init__("watch_stimuli")
        self._last = None
        # TRANSIENT_LOCAL to match the latched publisher, so starting this
        # AFTER the run has begun still shows the current state immediately
        # rather than waiting for the next heartbeat.
        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, topic, self._on_state, qos)
        print(f"watching {topic} -- Ctrl-C to stop")

    def _on_state(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        phase = d.get("phase", "?")
        left = (d.get("left") or {}).get("name", "-")
        right = (d.get("right") or {}).get("name", "-")
        key = (phase, left, right)
        if key == self._last:
            return
        self._last = key
        seed = d.get("trial_seed")
        extra = f"   trial_seed={seed}" if seed is not None else ""
        print(f"  {phase:9s} LEFT={left:<20} RIGHT={right:<20}{extra}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/stimulus_publisher/stimulus_state")
    args = ap.parse_args()

    rclpy.init()
    node = Watcher(args.topic)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass            # Ctrl-C, or SIGTERM from a wrapper like `timeout`
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
