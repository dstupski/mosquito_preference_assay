"""Draw the trigger zone on a live camera feed, the way display_check draws
the stimulus circles on the projector.

    ros2 launch mosquito_preference_assay trigger_roi.launch.py

The zone is the `roi` the detector watches: a mosquito-sized blob inside it
fires the trial, and anything outside it -- the equipment stand, indicator
LEDs, the mesh edge, a reflection off the arena wall -- is invisible. Getting
it right is the difference between a trial that starts when the animal arrives
and one that starts when someone walks past the rig.

Controls
    drag inside the box      move the whole zone
    drag a corner            resize from that corner
    drag on empty image      draw a new zone from scratch
    arrow keys               nudge 1 px  (shift = 10 px)
    [  ]                     shrink / grow about the centre
    d                        detection overlay on/off
    b                        re-capture the background frame
    f                        reset to the full frame
    s                        SAVE
    q / Esc                  quit

Pressing `s` writes trigger_roi.local.yaml, which every launch that starts a
detector layers over detector_params -- so, as with the projector, aligning is
the whole workflow and there is nothing to copy afterwards.

It saves `image_topic` alongside `roi`, on purpose. A pixel box only means
something on the camera it was drawn on, so the file records which camera the
zone belongs to and the experiment inherits that choice. Yesterday's lesson
from the projector, applied before it can bite: calibrating on one device and
running on another is a silent failure, and the fix is to never let the two
facts live apart.

The detection overlay runs the SAME code the detector runs
(detection.find_candidates), with the same parameters, so what you see boxed
here is what would fire a trial -- not an approximation of it.
"""

import datetime
import json
import os
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .config_paths import resolve_config
from .detection import find_candidates, parse_roi

WINDOW = "trigger zone -- drag to place, s to save, q to quit"

# Grab radius (screen px) for a corner handle. Generous: at a display scale of
# 0.5 a tight radius makes the corners genuinely hard to hit.
HANDLE_PX = 14
MIN_SIZE_PX = 20


def _dated_name():
    return f"{datetime.date.today():%Y%m%d}_trigger_roi.yaml"


def _resolve_out_file(spec, save_dir):
    """Where the dated archive goes. A zone is a measurement of the rig on a
    day -- after the camera is bumped the old numbers are wrong and you want
    both files to still exist."""
    spec = (spec or "").strip()
    if spec.lower() == "none":
        return ""
    if spec:
        path = Path(spec).expanduser()
        return str(path / _dated_name()) if path.is_dir() else str(path)
    base = Path((save_dir or "").strip()).expanduser() if save_dir else Path.cwd()
    return str(base / _dated_name())


def _resolve_live_file(spec):
    """The file the detector reads back. Defaults to the package's
    config/trigger_roi.local.yaml -- gitignored, layered by every launch."""
    spec = (spec or "").strip()
    if spec.lower() == "none":
        return ""
    if spec:
        return str(Path(spec).expanduser())
    try:
        shipped = resolve_config("detector_params.yaml")
        return str(Path(shipped).resolve().parent / "trigger_roi.local.yaml")
    except Exception:                                   # noqa: BLE001
        return "trigger_roi.local.yaml"


class TriggerRoiNode(Node):
    def __init__(self):
        super().__init__("trigger_roi")

        self.image_topic = str(
            self.declare_parameter("image_topic", "/cam_sync/cam0/image_raw").value)
        qos_kind = str(self.declare_parameter("image_qos", "sensor_data").value)
        self.save_dir = str(self.declare_parameter("save_dir", "").value)
        self.live_file = _resolve_live_file(
            str(self.declare_parameter("live_file", "").value))
        self.out_file = _resolve_out_file(
            str(self.declare_parameter("out_file", "").value), self.save_dir)

        # Same knobs the detector uses, so the overlay is a true preview.
        self.diff_threshold = int(self.declare_parameter("diff_threshold", 25).value)
        self.min_area = float(self.declare_parameter("min_area_px", 4.0).value)
        self.max_area = float(self.declare_parameter("max_area_px", 5000.0).value)
        self.morph_kernel = int(self.declare_parameter("morph_kernel", 3).value)

        # Fit a 1440x1080 feed on a laptop screen without the user resizing.
        self.max_display_px = int(self.declare_parameter("max_display_px", 1100).value)

        self.roi = parse_roi(str(self.declare_parameter("roi", "").value))
        self.show_detection = bool(
            self.declare_parameter("show_detection", True).value)

        self.bridge = CvBridge()
        self.frame = None            # latest BGR frame, full resolution
        self.gray = None
        self.background = None
        self.scale = 1.0
        self.saved_note = ""
        self.frames_seen = 0

        qos = qos_profile_sensor_data if qos_kind == "sensor_data" else 10
        self.create_subscription(Image, self.image_topic, self._on_image, qos)
        self.get_logger().info(
            f"trigger_roi: watching '{self.image_topic}' (qos={qos_kind})\n"
            f"  live file : {self.live_file or '(disabled)'}\n"
            f"  archive   : {self.out_file or '(disabled)'}")

    # --- input ----------------------------------------------------------- #
    def _on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:                        # noqa: BLE001
            self.get_logger().warn(f"cannot convert image: {exc}", once=True)
            return

        if frame.ndim == 2:
            gray = frame
            bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            bgr = frame
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        self.gray = gray
        self.frame = bgr
        self.frames_seen += 1
        if self.background is None:
            self.background = gray.copy()
        if self.roi is None:
            h, w = gray.shape[:2]
            # A centred half-frame box: obviously provisional, so it reads as
            # "put me somewhere" rather than as a working default.
            self.roi = (w // 4, h // 4, 3 * w // 4, 3 * h // 4)

    # --- geometry -------------------------------------------------------- #
    def clamp(self, roi):
        if self.gray is None:
            return roi
        h, w = self.gray.shape[:2]
        x0, y0, x1, y1 = (int(round(v)) for v in roi)
        x0, x1 = sorted((x0, x1))
        y0, y1 = sorted((y0, y1))
        x0, y0 = max(0, x0), max(0, y0)
        x1, y1 = min(w, x1), min(h, y1)
        if x1 - x0 < MIN_SIZE_PX:
            x1 = min(w, x0 + MIN_SIZE_PX)
            x0 = max(0, x1 - MIN_SIZE_PX)
        if y1 - y0 < MIN_SIZE_PX:
            y1 = min(h, y0 + MIN_SIZE_PX)
            y0 = max(0, y1 - MIN_SIZE_PX)
        return (x0, y0, x1, y1)

    def save(self):
        if self.roi is None:
            return "nothing to save yet -- no frames received"
        x0, y0, x1, y1 = self.roi
        text = (
            "# Written by trigger_roi against the live camera feed.\n"
            "# The detector's trigger zone: a mosquito-sized blob inside this\n"
            "# box fires a trial; everything outside it is invisible.\n"
            "#\n"
            "# image_topic travels WITH the box on purpose -- a pixel box only\n"
            "# means something on the camera it was drawn on, so this file is\n"
            "# also what selects the detection camera.\n"
            "/**:\n"
            "  ros__parameters:\n"
            f"    image_topic: \"{self.image_topic}\"\n"
            f"    roi: \"{x0},{y0},{x1},{y1}\"\n"
            f"    # zone {x1 - x0}x{y1 - y0} px"
            f" at ({x0},{y0}) in a {self.gray.shape[1]}x{self.gray.shape[0]} frame\n"
        )
        written = []
        for path in (self.live_file, self.out_file):
            if not path:
                continue
            try:
                Path(path).parent.mkdir(parents=True, exist_ok=True)
                Path(path).write_text(text)
                written.append(path)
            except OSError as exc:
                self.get_logger().error(f"could not write {path}: {exc}")
        if not written:
            return "could not write anything"
        for path in written:
            self.get_logger().info(f"saved {path}")
        return f"saved -> {os.path.basename(written[0])}"


def _hit_corner(roi, x, y, scale):
    """Which corner handle (if any) is under the cursor, in image coords."""
    x0, y0, x1, y1 = roi
    radius = HANDLE_PX / max(scale, 1e-6)
    for name, (cx, cy) in (("tl", (x0, y0)), ("tr", (x1, y0)),
                           ("bl", (x0, y1)), ("br", (x1, y1))):
        if abs(x - cx) <= radius and abs(y - cy) <= radius:
            return name
    return None


def _draw(node, drag):
    frame = node.frame
    if frame is None:
        canvas = np.full((360, 640, 3), 40, np.uint8)
        cv2.putText(canvas, f"waiting for {node.image_topic}", (20, 180),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)
        return canvas

    view = frame.copy()
    roi = node.roi

    # Everything outside the zone is what the detector cannot see -- dim it, so
    # the blind region is visible at a glance rather than inferred from a line.
    if roi is not None:
        x0, y0, x1, y1 = roi
        shade = view.copy()
        shade[:] = (0, 0, 0)
        shade[y0:y1, x0:x1] = view[y0:y1, x0:x1]
        view = cv2.addWeighted(view, 0.35, shade, 0.65, 0)

    inside = outside = 0
    if node.show_detection and node.background is not None and node.gray is not None:
        if node.gray.shape == node.background.shape:
            for blob in find_candidates(
                    node.gray, node.background, diff_threshold=node.diff_threshold,
                    min_area=node.min_area, max_area=node.max_area,
                    morph_kernel=node.morph_kernel, roi=None):
                bx, by, bw, bh = blob["bbox"]
                hit = (roi is not None and roi[0] <= blob["cx"] < roi[2]
                       and roi[1] <= blob["cy"] < roi[3])
                inside += hit
                outside += not hit
                # green = would fire a trial, grey = seen but ignored
                colour = (0, 230, 0) if hit else (130, 130, 130)
                cv2.rectangle(view, (bx - 4, by - 4), (bx + bw + 4, by + bh + 4),
                              colour, 2)

    if roi is not None:
        x0, y0, x1, y1 = roi
        cv2.rectangle(view, (x0, y0), (x1, y1), (0, 200, 255), 2)
        for cx, cy in ((x0, y0), (x1, y0), (x0, y1), (x1, y1)):
            cv2.rectangle(view, (cx - 7, cy - 7), (cx + 7, cy + 7), (0, 200, 255), -1)

    scale = min(1.0, node.max_display_px / max(view.shape[1], 1))
    if scale < 1.0:
        view = cv2.resize(view, None, fx=scale, fy=scale,
                          interpolation=cv2.INTER_AREA)
    node.scale = scale

    h, w = node.gray.shape[:2]
    lines = [
        f"{node.image_topic}   {w}x{h}   frames {node.frames_seen}",
        (f"zone {roi[0]},{roi[1]} -> {roi[2]},{roi[3]}"
         f"   ({roi[2] - roi[0]}x{roi[3] - roi[1]} px)" if roi else "no zone"),
        f"blobs: {inside} inside (would fire), {outside} outside (ignored)"
        if node.show_detection else "detection overlay off (d)",
        "drag=move  corner=resize  [ ]=size  arrows=nudge  b=background  s=SAVE  q=quit",
    ]
    if node.saved_note:
        lines.append(node.saved_note)
    if drag:
        lines.append(f"dragging: {drag}")

    y = 22
    for i, line in enumerate(lines):
        colour = (120, 255, 120) if node.saved_note and i == len(lines) - 1 \
            else (255, 255, 255)
        cv2.putText(view, line, (11, y + 1), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(view, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    colour, 1, cv2.LINE_AA)
        y += 22
    return view


def main(args=None):
    rclpy.init(args=args)
    node = TriggerRoiNode()

    state = {"mode": None, "anchor": None, "roi0": None}

    def on_mouse(event, sx, sy, _flags, _param):
        if node.roi is None or node.gray is None:
            return
        scale = max(node.scale, 1e-6)
        x, y = sx / scale, sy / scale          # screen -> image coordinates

        if event == cv2.EVENT_LBUTTONDOWN:
            corner = _hit_corner(node.roi, x, y, scale)
            x0, y0, x1, y1 = node.roi
            if corner:
                state.update(mode=corner, anchor=(x, y), roi0=node.roi)
            elif x0 <= x <= x1 and y0 <= y <= y1:
                state.update(mode="move", anchor=(x, y), roi0=node.roi)
            else:
                # drawing a fresh box: anchor one corner, drag out the other
                state.update(mode="new", anchor=(x, y), roi0=(x, y, x, y))
                node.roi = node.clamp((x, y, x + MIN_SIZE_PX, y + MIN_SIZE_PX))

        elif event == cv2.EVENT_MOUSEMOVE and state["mode"]:
            ax, ay = state["anchor"]
            x0, y0, x1, y1 = state["roi0"]
            mode = state["mode"]
            if mode == "move":
                dx, dy = x - ax, y - ay
                node.roi = node.clamp((x0 + dx, y0 + dy, x1 + dx, y1 + dy))
            elif mode == "new":
                node.roi = node.clamp((ax, ay, x, y))
            else:
                nx0 = x if "l" in mode else x0
                nx1 = x if "r" in mode else x1
                ny0 = y if "t" in mode else y0
                ny1 = y if "b" in mode else y1
                node.roi = node.clamp((nx0, ny0, nx1, ny1))

        elif event == cv2.EVENT_LBUTTONUP:
            state.update(mode=None, anchor=None, roi0=None)

    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW, on_mouse)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            cv2.imshow(WINDOW, _draw(node, state["mode"]))
            key = cv2.waitKey(1) & 0xFF
            if key == 255:
                continue
            node.saved_note = ""

            if key in (ord("q"), 27):
                break
            elif key == ord("s"):
                node.saved_note = node.save()
            elif key == ord("b"):
                if node.gray is not None:
                    node.background = node.gray.copy()
                    node.saved_note = "background re-captured"
            elif key == ord("d"):
                node.show_detection = not node.show_detection
            elif key == ord("f") and node.gray is not None:
                h, w = node.gray.shape[:2]
                node.roi = (0, 0, w, h)
            elif key in (ord("["), ord("]")) and node.roi is not None:
                step = -10 if key == ord("[") else 10
                x0, y0, x1, y1 = node.roi
                node.roi = node.clamp((x0 - step, y0 - step, x1 + step, y1 + step))
            elif node.roi is not None:
                # arrow keys: OpenCV reports them differently per backend, so
                # accept both the GTK codes and WASD as a guaranteed fallback.
                nudge = {81: (-1, 0), 82: (0, -1), 83: (1, 0), 84: (0, 1),
                         ord("a"): (-1, 0), ord("w"): (0, -1),
                         ord("e"): (1, 0), ord("x"): (0, 1)}.get(key)
                if nudge:
                    dx, dy = nudge
                    x0, y0, x1, y1 = node.roi
                    node.roi = node.clamp((x0 + dx, y0 + dy, x1 + dx, y1 + dy))
    except KeyboardInterrupt:
        pass
    finally:
        if node.roi is not None:
            node.get_logger().info(
                f"final zone: {json.dumps(list(node.roi))} on {node.image_topic}")
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
