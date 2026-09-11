#!/usr/bin/env python3
"""Play a video file, or a directory of frame images, as a pseudo camera feed
-- publishes sensor_msgs/Image so mosquito_detector (or anything else that
wants a camera topic) can be exercised without real hardware.

For two synchronized feeds (e.g. a stereo rig), use dual_video_publisher
instead.

    # a video file:
    ros2 run mosquito_preference_assay video_publisher --ros-args \\
        -p source:=/path/to/clip.mp4 -p topic:=/camera/image_raw -p rate_hz:=20

    # a directory of frames (any of .bmp/.png/.jpg/.jpeg, sorted by filename) --
    # e.g. a session from test_videos_particle_tracking/data/raw/<session>/cam_a:
    ros2 run mosquito_preference_assay video_publisher --ros-args \\
        -p source:=/path/to/cam_a -p rate_hz:=20 -p loop:=false

Parameters:
    source     string  (required)   video file path, or a directory of frame images
    topic      string  /camera/image_raw
    rate_hz    double  20.0
    loop       bool    True         restart from the beginning when the source runs out
    frame_id   string  camera       image header.frame_id
"""

import sys

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

from .frame_source import FrameSource


class VideoPublisher(Node):

    def __init__(self):
        super().__init__("video_publisher")

        source = str(self.declare_parameter("source", "").value)
        if not source:
            raise RuntimeError(
                "video_publisher requires -p source:=<video file or frame directory>")
        topic = str(self.declare_parameter("topic", "/camera/image_raw").value)
        rate_hz = float(self.declare_parameter("rate_hz", 20.0).value)
        self._loop = bool(self.declare_parameter("loop", True).value)
        self._frame_id = str(self.declare_parameter("frame_id", "camera").value)

        self._bridge = CvBridge()
        self._pub = self.create_publisher(Image, topic, 10)
        self._source = FrameSource(source)
        self.get_logger().info(
            f"publishing {self._source.description} to '{topic}' "
            f"@ {rate_hz} Hz (loop={self._loop})"
        )

        self._sent = 0
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

    def _tick(self):
        frame = self._source.next_frame()
        if frame is None:
            if self._loop:
                self._source.restart()
                frame = self._source.next_frame()
            if frame is None:
                self.get_logger().info(f"source exhausted after {self._sent} frames; stopping")
                self._timer.cancel()
                return

        encoding = "bgr8" if frame.ndim == 3 else "mono8"
        msg = self._bridge.cv2_to_imgmsg(frame, encoding=encoding)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._pub.publish(msg)
        self._sent += 1


def main(args=None):
    rclpy.init(args=args)
    try:
        node = VideoPublisher()
    except RuntimeError as exc:
        print(f"[video_publisher] {exc}", file=sys.stderr)
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
