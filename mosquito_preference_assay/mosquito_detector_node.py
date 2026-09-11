#!/usr/bin/env python3
"""Watch a camera feed, detect a mosquito entering a region, publish a
detection-event message -- which doubles as the assay's trigger (see
`trigger_msg_type: string` in stimulus_publisher_node.py / the experiment's
`trigger:` block).

Algorithm: background subtraction against a static reference frame (captured
once, from the first image received), restricted to `roi`, blob-area
filtered -- see detection.py (ported from
test_videos_particle_tracking/src/particle_tracking/detection.py). A
detection fires after `consecutive_frames` frames with a qualifying blob, at
most once per `cooldown_sec`.

Configure it with a params file -- copy config/detector_params.yaml, edit it
for your rig, and:

    ros2 launch mosquito_preference_assay detector.launch.py \\
        params_file:=/path/to/my_detector.yaml
    # or:
    ros2 run mosquito_preference_assay mosquito_detector --ros-args \\
        --params-file /path/to/my_detector.yaml

Every parameter, its type, default and meaning is documented in
config/detector_params.yaml. In brief:

    image_topic (str)   camera feed (sensor_msgs/Image)
    image_qos   (str)   reliable | sensor_data
    topic       (str)   detection-event output (std_msgs/String JSON)
    roi         (str)   "x0,y0,x1,y1" px box, exclusive; "" = whole frame
    diff_threshold (int), min_area_px / max_area_px (double), morph_kernel (int)
    consecutive_frames (int), cooldown_sec (double)
    publish_debug_image (bool)

Test without a real camera: video_publisher_node.py plays a video file or a
directory of frames as a pseudo camera feed on the same image topic.

JSON schema "mosquito_preference_assay/detection_event/1":
    schema, stamp_wall, event ("mosquito_detected"), position_px [cx,cy],
    bbox_px [x,y,w,h], area_px, consecutive_frames, roi_px, image_topic,
    frame_stamp (image header stamp, seconds)
"""

import json
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .detection import find_candidates, parse_roi

SCHEMA = "mosquito_preference_assay/detection_event/1"


class MosquitoDetector(Node):

    def __init__(self):
        super().__init__("mosquito_detector")

        self._image_topic = str(self.declare_parameter("image_topic", "/camera/image_raw").value)
        image_qos_kind = str(self.declare_parameter("image_qos", "reliable").value)
        out_topic = str(self.declare_parameter("topic", "/arena/mosquito_present").value)
        self._roi = parse_roi(self.declare_parameter("roi", "").value)
        self._diff_threshold = int(self.declare_parameter("diff_threshold", 25).value)
        self._min_area = float(self.declare_parameter("min_area_px", 4.0).value)
        self._max_area = float(self.declare_parameter("max_area_px", 5000.0).value)
        self._morph_kernel = int(self.declare_parameter("morph_kernel", 3).value)
        self._consecutive_needed = max(
            1, int(self.declare_parameter("consecutive_frames", 3).value))
        self._cooldown_sec = float(self.declare_parameter("cooldown_sec", 10.0).value)
        publish_debug = bool(self.declare_parameter("publish_debug_image", False).value)

        self._bridge = CvBridge()
        self._background = None
        self._consecutive = 0
        self._last_fire_monotonic = None

        self._pub = self.create_publisher(String, out_topic, 10)
        self._debug_pub = (
            self.create_publisher(Image, "~/debug_image", 1) if publish_debug else None
        )

        image_qos = qos_profile_sensor_data if image_qos_kind == "sensor_data" else 10
        self._sub = self.create_subscription(Image, self._image_topic, self._on_image, image_qos)

        self.get_logger().info(
            f"watching '{self._image_topic}' (qos={image_qos_kind}), roi={self._roi}, "
            f"publishing detections to '{out_topic}'"
        )

    def _on_image(self, msg):
        try:
            # mono8 in one cv_bridge conversion -- whatever the source encoding
            # actually is (mono8, bgr8, bayer, ...), rather than forcing bgr8
            # and then manually cvtColor-ing back to gray.
            gray = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge conversion failed: {exc!r}")
            return

        if self._background is None or self._background.shape != gray.shape:
            self._background = gray
            self._consecutive = 0
            self.get_logger().info(f"captured background frame ({gray.shape[1]}x{gray.shape[0]})")
            return

        candidates = find_candidates(
            gray, self._background,
            diff_threshold=self._diff_threshold, min_area=self._min_area,
            max_area=self._max_area, morph_kernel=self._morph_kernel, roi=self._roi,
        )
        best = candidates[0] if candidates else None

        if self._debug_pub is not None:
            self._publish_debug(gray, best)

        if best is None:
            self._consecutive = 0
            return

        self._consecutive += 1
        if self._consecutive < self._consecutive_needed:
            return

        now = time.monotonic()
        since_last = None if self._last_fire_monotonic is None else now - self._last_fire_monotonic
        if since_last is not None and since_last < self._cooldown_sec:
            return  # still cooling down from the last fire

        self._last_fire_monotonic = now
        self._consecutive = 0
        self._fire(best, msg)

    def _fire(self, blob, image_msg):
        x, y, w, h = blob["bbox"]
        event = {
            "schema": SCHEMA,
            "stamp_wall": time.time(),
            "event": "mosquito_detected",
            "position_px": [blob["cx"], blob["cy"]],
            "bbox_px": [x, y, w, h],
            "area_px": blob["area"],
            "consecutive_frames": self._consecutive_needed,
            "roi_px": list(self._roi) if self._roi else None,
            "image_topic": self._image_topic,
            "frame_stamp": image_msg.header.stamp.sec + image_msg.header.stamp.nanosec / 1e9,
        }
        msg = String()
        msg.data = json.dumps(event, separators=(",", ":"))
        self._pub.publish(msg)
        self.get_logger().info(
            f"mosquito_detected at ({blob['cx']:.0f},{blob['cy']:.0f}) area={blob['area']:.0f}"
        )

    def _publish_debug(self, gray, best):
        # BGR only for the annotation colors -- this debug path only runs
        # when publish_debug_image is on, not in the normal hot path.
        annotated = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if self._roi is not None:
            x0, y0, x1, y1 = self._roi
            cv2.rectangle(annotated, (x0, y0), (x1, y1), (0, 255, 255), 2)
        if best is not None:
            x, y, w, h = best["bbox"]
            cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(annotated, f"area={best['area']:.0f}", (x, max(0, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        out = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out.header.stamp = self.get_clock().now().to_msg()
        self._debug_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MosquitoDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
