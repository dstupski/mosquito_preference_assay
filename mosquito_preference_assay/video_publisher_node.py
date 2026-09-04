#!/usr/bin/env python3
"""Play a video file, or a directory of frame images, as a pseudo camera feed
-- publishes sensor_msgs/Image so mosquito_detector (or anything else that
wants a camera topic) can be exercised without real hardware.

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
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

_FRAME_EXTS = (".bmp", ".png", ".jpg", ".jpeg")


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

        path = Path(source)
        if not path.exists():
            raise RuntimeError(f"source not found: {path}")

        if path.is_dir():
            self._frames = sorted(
                (f for f in path.iterdir() if f.suffix.lower() in _FRAME_EXTS),
                key=lambda f: f.name,
            )
            if not self._frames:
                raise RuntimeError(f"no frame images ({_FRAME_EXTS}) found in {path}")
            self._cap = None
            self._idx = 0
            self.get_logger().info(
                f"publishing {len(self._frames)} frames from '{path}' to '{topic}' @ {rate_hz} Hz"
            )
        else:
            self._cap = cv2.VideoCapture(str(path))
            if not self._cap.isOpened():
                raise RuntimeError(f"could not open video {path}")
            self._frames = None
            self.get_logger().info(
                f"publishing video '{path}' to '{topic}' @ {rate_hz} Hz (loop={self._loop})"
            )

        self._sent = 0
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

    def _tick(self):
        frame = self._next_frame()
        if frame is None:
            if self._loop:
                self._restart()
                frame = self._next_frame()
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

    def _next_frame(self):
        if self._frames is not None:
            if self._idx >= len(self._frames):
                return None
            frame = cv2.imread(str(self._frames[self._idx]))
            self._idx += 1
            return frame
        ok, frame = self._cap.read()
        return frame if ok else None

    def _restart(self):
        if self._frames is not None:
            self._idx = 0
        else:
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)


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
