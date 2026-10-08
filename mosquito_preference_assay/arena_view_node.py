"""A live camera feed with the trigger zone drawn on it.

    ros2 launch mosquito_preference_assay arena_view.launch.py

Read-only, and safe to leave open for a whole session. It shows one thing:
the only region of the frame where a mosquito can start a trial. Everything
outside it is dimmed, because that is what the detector is blind to.

It reads the same config the detector reads, so the box on screen is the box
that fires trials -- not a drawing of where you think it is.

Press q or Esc to close it; the experiment is unaffected.
"""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .detection import parse_roi

WINDOW = "arena -- trigger zone"


class ArenaViewNode(Node):
    def __init__(self):
        super().__init__("arena_view")

        self.image_topic = str(self.declare_parameter(
            "image_topic", "/cam_sync/cam1/image_raw").value)
        qos_kind = str(self.declare_parameter("image_qos", "sensor_data").value)
        self.roi = parse_roi(str(self.declare_parameter("roi", "").value))

        # Render far below the camera's rate, dropping frames in between
        # BEFORE converting them: at 200 fps a window that drew every frame
        # would compete with the detector for CPU during the trial.
        self.display_hz = float(self.declare_parameter("display_hz", 15.0).value)
        self.max_display_px = int(
            self.declare_parameter("max_display_px", 1100).value)

        self.bridge = CvBridge()
        self.frame = None
        self.last_render = 0.0

        qos = qos_profile_sensor_data if qos_kind == "sensor_data" else 10
        self.create_subscription(Image, self.image_topic, self._on_image, qos)
        self.get_logger().info(
            f"arena_view: {self.image_topic} (qos={qos_kind}), zone={self.roi}")

    def _on_image(self, msg):
        now = time.monotonic()
        if now - self.last_render < 1.0 / max(self.display_hz, 1.0):
            return                  # dropped before the expensive conversion
        self.last_render = now
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:                        # noqa: BLE001
            self.get_logger().warn(f"cannot convert image: {exc}", once=True)
            return
        self.frame = (cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                      if frame.ndim == 2 else frame)


def _draw(node):
    frame = node.frame
    if frame is None:
        canvas = np.full((360, 640, 3), 32, np.uint8)
        cv2.putText(canvas, f"waiting for {node.image_topic}", (20, 180),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (190, 190, 190), 1, cv2.LINE_AA)
        return canvas

    view = frame.copy()
    h, w = view.shape[:2]
    roi = node.roi

    if roi is not None:
        x0, y0 = max(0, roi[0]), max(0, roi[1])
        x1, y1 = min(w, roi[2]), min(h, roi[3])
        dim = (view * 0.5).astype(np.uint8)
        dim[y0:y1, x0:x1] = view[y0:y1, x0:x1]
        view = dim
        cv2.rectangle(view, (x0, y0), (x1, y1), (0, 200, 255), 2)

    scale = min(1.0, node.max_display_px / max(w, 1))
    if scale < 1.0:
        view = cv2.resize(view, None, fx=scale, fy=scale,
                          interpolation=cv2.INTER_AREA)

    label = (f"trigger zone  {roi[0]},{roi[1]} -> {roi[2]},{roi[3]}" if roi
             else "NO ZONE SET -- the whole frame can trigger")
    colour = (0, 200, 255) if roi else (80, 80, 255)
    cv2.putText(view, label, (13, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(view, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                colour, 1, cv2.LINE_AA)
    return view


def main(args=None):
    rclpy.init(args=args)
    node = ArenaViewNode()
    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            cv2.imshow(WINDOW, _draw(node))
            if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
