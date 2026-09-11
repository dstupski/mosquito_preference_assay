#!/usr/bin/env python3
"""Real-time 2D blob tracking on ONE camera feed -- run one instance per
camera (like video_publisher) so each camera's detection work is its own OS
process and gets its own CPU core, rather than two cameras sharing one
Python callback. Pair two instances' output with stereo_sync_node.

Same background-subtraction approach as mosquito_detector / detection.py, but
publishes continuously (every frame with a qualifying blob) rather than
firing a debounced trigger event.

Publishes geometry_msgs/PointStamped -- point.x / point.y are the pixel
position, point.z is the blob AREA (not a real z coordinate; reused so the
message carries a `header.stamp` for message_filters to synchronize two
cameras' tracks by, without inventing a custom message type). Nothing is
published for a frame where nothing qualifying was found -- a "no detection"
is the absence of a message, not a null field, which is also exactly the
right behavior for stereo_sync_node.

    ros2 run mosquito_preference_assay tracker --ros-args \\
        -p image_topic:=/cam_a/image_raw -p topic:=/tracking/cam_a/position \\
        -p roi:=340,40,1260,1070

Parameters:
    image_topic    string  /camera/image_raw          sensor_msgs/Image input
    image_qos      string  reliable                    reliable | sensor_data
    topic          string  ~/position                  geometry_msgs/PointStamped output
    frame_id       string  camera                       point header.frame_id
    roi            string  ""                          "x0,y0,x1,y1" px, exclusive;
                                                       "" = whole frame
    diff_threshold  int     25
    min_area_px / max_area_px  double  4.0 / 5000.0
    morph_kernel     int     3
    log_every_n      int     200        log the achieved detected-frame rate
                                        every N (0 disables)
    publish_debug_image  bool  False    publish ~/debug_image: the frame with the
                                        ROI box and the accepted blob drawn on it,
                                        for eyeballing WHEN and WHERE detection is
                                        good. Off by default -- it costs a color
                                        conversion + encode per frame, so it is not
                                        in the normal hot path.
"""

import time

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .detection import find_candidates, parse_roi


class Tracker(Node):

    def __init__(self):
        super().__init__("tracker")

        image_topic = str(self.declare_parameter("image_topic", "/camera/image_raw").value)
        image_qos_kind = str(self.declare_parameter("image_qos", "reliable").value)
        out_topic = str(self.declare_parameter("topic", "~/position").value)
        self._frame_id = str(self.declare_parameter("frame_id", "camera").value)
        self._roi = parse_roi(self.declare_parameter("roi", "").value)
        self._diff_threshold = int(self.declare_parameter("diff_threshold", 25).value)
        self._min_area = float(self.declare_parameter("min_area_px", 4.0).value)
        self._max_area = float(self.declare_parameter("max_area_px", 5000.0).value)
        self._morph_kernel = int(self.declare_parameter("morph_kernel", 3).value)
        self._log_every_n = int(self.declare_parameter("log_every_n", 200).value)
        publish_debug = bool(self.declare_parameter("publish_debug_image", False).value)

        self._bridge = CvBridge()
        self._background = None
        self._n_frames = 0
        self._pub = self.create_publisher(PointStamped, out_topic, 10)
        self._debug_pub = (
            self.create_publisher(Image, "~/debug_image", 1) if publish_debug else None
        )

        image_qos = qos_profile_sensor_data if image_qos_kind == "sensor_data" else 10
        self._sub = self.create_subscription(Image, image_topic, self._on_image, image_qos)

        self._n_detected = 0
        self._log_window_start = time.monotonic()

        self.get_logger().info(
            f"tracking '{image_topic}' (qos={image_qos_kind}, roi={self._roi}), "
            f"publishing to '{out_topic}'"
        )

    def _on_image(self, msg):
        try:
            gray = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge conversion failed: {exc!r}")
            return

        if self._background is None or self._background.shape != gray.shape:
            self._background = gray
            self.get_logger().info(f"captured background frame ({gray.shape[1]}x{gray.shape[0]})")
            return

        self._n_frames += 1
        candidates = find_candidates(
            gray, self._background, diff_threshold=self._diff_threshold,
            min_area=self._min_area, max_area=self._max_area,
            morph_kernel=self._morph_kernel, roi=self._roi,
        )
        best = candidates[0] if candidates else None
        if self._debug_pub is not None:
            self._publish_debug(gray, best, len(candidates), msg.header)
        if best is None:
            return

        point = PointStamped()
        point.header.stamp = msg.header.stamp
        point.header.frame_id = self._frame_id
        point.point.x = float(best["cx"])
        point.point.y = float(best["cy"])
        point.point.z = float(best["area"])  # reused as area -- see module docstring
        self._pub.publish(point)

        self._n_detected += 1
        if self._log_every_n and self._n_detected % self._log_every_n == 0:
            now = time.monotonic()
            rate = self._log_every_n / max(1e-9, now - self._log_window_start)
            self.get_logger().info(
                f"{self._n_detected} detections so far "
                f"(~{rate:.1f} Hz over the last {self._log_every_n})"
            )
            self._log_window_start = now

    def _publish_debug(self, gray, best, n_candidates, header):
        # BGR only for the annotation colors -- this path only runs when
        # publish_debug_image is on, not in the normal hot path.
        annotated = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if self._roi is not None:
            x0, y0, x1, y1 = self._roi
            cv2.rectangle(annotated, (x0, y0), (x1, y1), (0, 255, 0), 1)

        if best is not None:
            cx, cy = int(round(best["cx"])), int(round(best["cy"]))
            cv2.circle(annotated, (cx, cy), 10, (0, 255, 255), 2)
            cv2.drawMarker(annotated, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 13, 1)
            label = (f"frame {self._n_frames}  ({cx}, {cy})  area {best['area']:.0f}"
                     f"  {n_candidates} candidate(s)")
            color = (0, 255, 255)
        else:
            label = f"frame {self._n_frames}  NO DETECTION"
            color = (0, 0, 255)
        cv2.putText(annotated, label, (20, 35), cv2.FONT_HERSHEY_SIMPLEX,
                    0.75, color, 2, cv2.LINE_AA)

        out = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out.header = header
        self._debug_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Tracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
