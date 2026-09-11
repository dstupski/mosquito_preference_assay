#!/usr/bin/env python3
"""Pair two tracker_node position streams into one synchronized stereo_track
message -- the second half of real-time 2D stereo tracking (see
tracker_node.py for the first half: one node per camera).

Synchronizes geometry_msgs/PointStamped from `topic_a` / `topic_b` by
header.stamp (message_filters.ApproximateTimeSynchronizer, `sync_slop_sec`
tolerance) and republishes the pair as std_msgs/String JSON -- the same
schema dual_tracker_node used to publish directly, so anything downstream
(eventually: feed x_a,y_a / x_b,y_b into a 3D calibration) doesn't care that
tracking is now two processes instead of one.

Because tracker_node only publishes when it actually detects something, a
synchronized pair here means BOTH cameras detected something at (about) the
same instant -- exactly what 3D triangulation needs, and it's the absence of
a message rather than a null field, so nothing needs to check "detected".

    ros2 run mosquito_preference_assay tracker --ros-args \\
        -p image_topic:=/cam_a/image_raw -p topic:=/tracking/cam_a/position -p roi:=... &
    ros2 run mosquito_preference_assay tracker --ros-args \\
        -p image_topic:=/cam_b/image_raw -p topic:=/tracking/cam_b/position -p roi:=... &
    ros2 run mosquito_preference_assay stereo_sync

Parameters:
    topic_a / topic_b   string  /tracking/cam_a/position / /tracking/cam_b/position
    topic               string  /tracking/stereo_track    output (std_msgs/String JSON)
    sync_slop_sec       double  0.05      max stamp difference to count as one instant
    queue_size           int     100       buffered messages per side awaiting a match
    log_every_n           int     200       log the achieved synchronized-pair
                                            rate every N (0 disables)

JSON schema "mosquito_preference_assay/stereo_track/1" (identical to
dual_tracker_node's): schema, stamp_wall, frame_stamp, a: {detected, x, y,
area}, b: {detected, x, y, area} -- detected is always true here (see above).
"""

import json
import time

import message_filters
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

SCHEMA = "mosquito_preference_assay/stereo_track/1"


class StereoSync(Node):

    def __init__(self):
        super().__init__("stereo_sync")

        topic_a = str(self.declare_parameter(
            "topic_a", "/tracking/cam_a/position").value)
        topic_b = str(self.declare_parameter(
            "topic_b", "/tracking/cam_b/position").value)
        out_topic = str(self.declare_parameter("topic", "/tracking/stereo_track").value)
        sync_slop_sec = float(self.declare_parameter("sync_slop_sec", 0.05).value)
        queue_size = int(self.declare_parameter("queue_size", 100).value)
        self._log_every_n = int(self.declare_parameter("log_every_n", 200).value)

        self._pub = self.create_publisher(String, out_topic, 10)

        sub_a = message_filters.Subscriber(self, PointStamped, topic_a)
        sub_b = message_filters.Subscriber(self, PointStamped, topic_b)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [sub_a, sub_b], queue_size=queue_size, slop=max(sync_slop_sec, 1e-9),
        )
        self._sync.registerCallback(self._on_pair)

        self._n_pairs = 0
        self._log_window_start = time.monotonic()

        self.get_logger().info(
            f"pairing '{topic_a}' + '{topic_b}' (slop={sync_slop_sec}s, "
            f"queue_size={queue_size}), publishing to '{out_topic}'"
        )

    def _on_pair(self, pt_a, pt_b):
        event = {
            "schema": SCHEMA,
            "stamp_wall": time.time(),
            "frame_stamp": pt_a.header.stamp.sec + pt_a.header.stamp.nanosec / 1e9,
            "a": _side_dict(pt_a),
            "b": _side_dict(pt_b),
        }
        msg = String()
        msg.data = json.dumps(event, separators=(",", ":"))
        self._pub.publish(msg)

        self._n_pairs += 1
        if self._log_every_n and self._n_pairs % self._log_every_n == 0:
            now = time.monotonic()
            rate = self._log_every_n / max(1e-9, now - self._log_window_start)
            self.get_logger().info(
                f"{self._n_pairs} synchronized pairs so far "
                f"(~{rate:.1f} Hz over the last {self._log_every_n})"
            )
            self._log_window_start = now


def _side_dict(pt):
    return {"detected": True, "x": pt.point.x, "y": pt.point.y, "area": pt.point.z}


def main(args=None):
    rclpy.init(args=args)
    node = StereoSync()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
