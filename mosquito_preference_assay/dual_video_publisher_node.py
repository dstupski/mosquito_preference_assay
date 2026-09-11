#!/usr/bin/env python3
"""Play two video files / frame directories as a SYNCHRONIZED pseudo dual
camera feed -- one timer publishes frame N of source A and frame N of source
B together, with the same ROS timestamp, every tick. Emulates a
hardware-triggered stereo rig (like the real cam_a/cam_b setup) for
exercising real-time tracking performance, without needing the real cameras.

For a single feed, use video_publisher instead.

    ros2 run mosquito_preference_assay dual_video_publisher --ros-args \\
        -p source_a:=/path/to/cam_a -p source_b:=/path/to/cam_b \\
        -p topic_a:=/cam_a/image_raw -p topic_b:=/cam_b/image_raw \\
        -p rate_hz:=140 -p loop:=false

Parameters:
    source_a / source_b        string  (required)   video file or frame
                                        directory, one per camera
    topic_a / topic_b          string  /cam_a/image_raw / /cam_b/image_raw
    rate_hz                    double  20.0
    loop                        bool    True     when EITHER source runs out,
                                        restart both together (never let them
                                        drift out of index sync); false stops both
    frame_id_a / frame_id_b    string  cam_a / cam_b
"""

import sys

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

from .frame_source import FrameSource


class DualVideoPublisher(Node):

    def __init__(self):
        super().__init__("dual_video_publisher")

        source_a = str(self.declare_parameter("source_a", "").value)
        source_b = str(self.declare_parameter("source_b", "").value)
        if not source_a or not source_b:
            raise RuntimeError(
                "dual_video_publisher requires -p source_a:=... -p source_b:=... "
                "(a video file or frame directory, one per camera)"
            )
        topic_a = str(self.declare_parameter("topic_a", "/cam_a/image_raw").value)
        topic_b = str(self.declare_parameter("topic_b", "/cam_b/image_raw").value)
        rate_hz = float(self.declare_parameter("rate_hz", 20.0).value)
        self._loop = bool(self.declare_parameter("loop", True).value)
        self._frame_id_a = str(self.declare_parameter("frame_id_a", "cam_a").value)
        self._frame_id_b = str(self.declare_parameter("frame_id_b", "cam_b").value)

        self._bridge = CvBridge()
        self._src_a = FrameSource(source_a)
        self._src_b = FrameSource(source_b)
        self._pub_a = self.create_publisher(Image, topic_a, 10)
        self._pub_b = self.create_publisher(Image, topic_b, 10)

        self.get_logger().info(
            f"publishing A: {self._src_a.description} -> '{topic_a}'; "
            f"B: {self._src_b.description} -> '{topic_b}'; "
            f"synchronized @ {rate_hz} Hz (loop={self._loop})"
        )

        self._sent = 0
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

    def _tick(self):
        frame_a = self._src_a.next_frame()
        frame_b = self._src_b.next_frame()
        if frame_a is None or frame_b is None:
            if self._loop:
                # Restart BOTH together -- a source running out first must
                # never leave the pair on mismatched frame indices.
                self._src_a.restart()
                self._src_b.restart()
                frame_a = self._src_a.next_frame()
                frame_b = self._src_b.next_frame()
            if frame_a is None or frame_b is None:
                self.get_logger().info(
                    f"a source exhausted after {self._sent} synchronized frames; stopping"
                )
                self._timer.cancel()
                return

        # One stamp for both -- this is the "synchronized" part: downstream
        # (e.g. message_filters.TimeSynchronizer) sees a matching pair.
        stamp = self.get_clock().now().to_msg()
        self._publish(self._pub_a, frame_a, self._frame_id_a, stamp)
        self._publish(self._pub_b, frame_b, self._frame_id_b, stamp)
        self._sent += 1

    def _publish(self, pub, frame, frame_id, stamp):
        encoding = "bgr8" if frame.ndim == 3 else "mono8"
        msg = self._bridge.cv2_to_imgmsg(frame, encoding=encoding)
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = DualVideoPublisher()
    except RuntimeError as exc:
        print(f"[dual_video_publisher] {exc}", file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(2)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
