"""A live camera feed with the trigger zone drawn on it.

    ros2 launch mosquito_preference_assay arena_view.launch.py

Read-only, and safe to leave open for a whole session. It shows the only
region of the frame where a mosquito can start a trial -- everything outside
it is dimmed -- and whether that region is live right now.

The zone is GREEN while the rig is armed and a mosquito entering it would
start a trial, and GREY when it would not: during a trial, after one, or
before the display has finished starting up. Colour rather than text alone,
so the state reads from across the room.

It reads the same config the detector reads, so the box on screen is the box
that fires trials -- not a drawing of where you think it is.

Press q or Esc to close it; the experiment is unaffected.
"""

import json
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .detection import parse_roi

WINDOW = "arena -- trigger zone"

# BGR. Green reads as "live", grey as "nothing you do here matters yet".
ARMED_COLOUR = (90, 220, 70)
IDLE_COLOUR = (150, 150, 150)
NO_ZONE_COLOUR = (80, 80, 255)


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

        self.state_topic = str(self.declare_parameter(
            "state_topic", "/stimulus_publisher/stimulus_state").value).strip()

        self.bridge = CvBridge()
        self.frame = None
        self.last_render = 0.0
        self.phase = None          # None until the assay says otherwise
        self.left = None
        self.right = None
        self.conflict = 0          # >1 publisher on the state topic

        qos = qos_profile_sensor_data if qos_kind == "sensor_data" else 10
        self.create_subscription(Image, self.image_topic, self._on_image, qos)

        if self.state_topic:
            # TRANSIENT_LOCAL to match the sketch's latched publisher -- with
            # volatile QoS this silently never connects and the window would
            # read NOT ARMED for ever.
            self.create_subscription(
                String, self.state_topic, self._on_state,
                QoSProfile(depth=1,
                           reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        # Two experiments publishing at once makes the banner flicker between
        # their pairings, which looks like a rendering fault and is really a
        # stale launch that never exited. Name it rather than let it confuse.
        self.create_timer(2.0, self._check_publishers)

        self.get_logger().info(
            f"arena_view: {self.image_topic} (qos={qos_kind}), zone={self.roi}")

    def _check_publishers(self):
        if not self.state_topic:
            return
        n = self.count_publishers(self.state_topic)
        if n != self.conflict and n > 1:
            self.get_logger().warn(
                f"{n} experiments are publishing on {self.state_topic} -- an "
                f"earlier launch is still running. The banner will flicker "
                f"between their pairings until it is stopped.")
        self.conflict = n

    def armed(self):
        return self.phase == "armed"

    def _on_state(self, msg):
        try:
            state = json.loads(msg.data)
        except (ValueError, AttributeError):
            return
        self.phase = state.get("phase")
        left = (state.get("left") or {}).get("name")
        right = (state.get("right") or {}).get("name")
        if left and right:
            self.left, self.right = left, right
        elif self.phase == "armed":
            # Armed with no stimuli is the classic mode, where the pairing is
            # not drawn until the trigger. Only then is a blank line correct;
            # otherwise keep showing the pair rather than blinking it away.
            self.left = self.right = None

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
    armed = node.armed()
    colour = ARMED_COLOUR if armed else IDLE_COLOUR

    if roi is not None:
        x0, y0 = max(0, roi[0]), max(0, roi[1])
        x1, y1 = min(w, roi[2]), min(h, roi[3])
        dim = (view * 0.5).astype(np.uint8)
        dim[y0:y1, x0:x1] = view[y0:y1, x0:x1]
        view = dim
        # thicker while armed, so the live state carries weight as well as hue
        cv2.rectangle(view, (x0, y0), (x1, y1), colour, 3 if armed else 2)

    scale = min(1.0, node.max_display_px / max(w, 1))
    if scale < 1.0:
        view = cv2.resize(view, None, fx=scale, fy=scale,
                          interpolation=cv2.INTER_AREA)

    if node.phase is None:
        state = "NOT ARMED  (no assay running)"
    elif armed:
        state = "ARMED"
    else:
        state = f"NOT ARMED  ({node.phase})"

    zone_label = (f"trigger zone  {roi[0]},{roi[1]} -> {roi[2]},{roi[3]}" if roi
                  else "NO ZONE SET -- the whole frame can trigger")
    rows = [(state, colour, 0.8)]
    if node.left and node.right:
        rows.append((f"LEFT {node.left}      RIGHT {node.right}",
                     (235, 235, 235), 0.62))
    rows.append((zone_label, colour if roi else NO_ZONE_COLOUR, 0.55))
    if node.conflict > 1:
        rows.append((f"{node.conflict} EXPERIMENTS RUNNING -- stop the older one",
                     NO_ZONE_COLOUR, 0.6))

    y = 34
    for text, col, size in rows:
        cv2.putText(view, text, (13, y + 1), cv2.FONT_HERSHEY_SIMPLEX, size,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(view, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, size,
                    col, 1, cv2.LINE_AA)
        y += 30
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
