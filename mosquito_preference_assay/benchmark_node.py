#!/usr/bin/env python3
"""Per-component micro-benchmark -- what this machine can actually do, before
any ROS messaging is involved. Run it first on a new PC: it tells you the
per-frame cost of each stage and the frame rate ceiling each one implies, so
when the live pipeline (see pipeline_monitor) misses a target rate you know
which part to blame.

    ros2 run mosquito_preference_assay benchmark --ros-args \\
        -p frames:=/path/to/session/cam_a -p roi:=340,40,1260,1070

Measures, on real frames from `frames`:
  * decode      -- reading a frame file (only relevant for replayed footage)
  * detect      -- find_candidates() at the given ROI, and full-frame for
                   comparison, since ROI area drives this cost linearly
  * cv_bridge   -- Image message encode + decode, which do NOT shrink with the
                   ROI: they always pay for the full frame
  * triangulate -- one stereo pair -> 3D point (needs the calibration files)

Each line reports ms/frame and the resulting Hz ceiling for that stage alone.
The pipeline's real ceiling is set by the slowest stage on its busiest node --
tracker runs decode+detect per camera, so compare those against your target.

Parameters:
    frames        string  ""      REQUIRED -- a directory of frame images
    pattern        string  *.bmp   glob for frames in that directory
    roi            string  ""      "x0,y0,x1,y1"; "" = full frame only
    n_frames        int     150     how many frames to time over
    diff_threshold / min_area_px / max_area_px / morph_kernel -- as in tracker
    checkerboard_file / plumbline_file  string  ""  optional; adds the
                                                    triangulation benchmark
"""

import time
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node

from .detection import find_candidates, parse_roi


def _time_over(items, function):
    """Seconds per call, cycling through `items` so each call works on a
    different buffer. Timing one cached frame over and over measures a
    warm-cache best case that the real pipeline never sees -- at 1.5 MB per
    frame the difference is roughly 2x, so always time over real data."""
    function(items[0])  # warm up
    start = time.perf_counter()
    for item in items:
        function(item)
    return (time.perf_counter() - start) / len(items)


def _row(label, seconds, note=""):
    return "%-34s %9.3f %12.0f  %s" % (label, seconds * 1000.0, 1.0 / seconds, note)


def run_benchmark(node, frames_dir, pattern, roi, n_frames, detect_kwargs,
                  checkerboard_file, plumbline_file):
    paths = sorted(Path(frames_dir).glob(pattern))
    if len(paths) < 2:
        raise ValueError(f"need at least 2 frames matching '{pattern}' in '{frames_dir}'")
    paths = paths[:max(2, n_frames)]

    log = node.get_logger().info
    images = [cv2.imread(str(p), cv2.IMREAD_UNCHANGED) for p in paths]
    background, sample = images[0], images[1]
    height, width = sample.shape[:2]
    megapixels = width * height / 1e6

    lines = [
        f"=== benchmark: {len(images)} frames, {width}x{height} "
        f"({megapixels:.2f} Mpx, {sample.dtype}, "
        f"{'grayscale' if sample.ndim == 2 else f'{sample.shape[2]} channels'}) ===",
        "%-34s %9s %12s  %s" % ("stage", "ms/frame", "Hz ceiling", "notes"),
    ]

    decode_seconds = _time_over(
        paths, lambda path: cv2.imread(str(path), cv2.IMREAD_UNCHANGED))
    lines.append(_row("decode frame file", decode_seconds,
                      "replay only; a real camera hands you the array"))

    full = dict(detect_kwargs, roi=None)
    lines.append(_row("detect, full frame", _time_over(
        images, lambda image: find_candidates(image, background, **full)),
        f"{megapixels:.2f} Mpx"))
    detect_roi_seconds = None
    if roi is not None:
        roi_kwargs = dict(detect_kwargs, roi=roi)
        roi_megapixels = (roi[2] - roi[0]) * (roi[3] - roi[1]) / 1e6
        detect_roi_seconds = _time_over(
            images, lambda image: find_candidates(image, background, **roi_kwargs))
        lines.append(_row("detect, ROI", detect_roi_seconds,
                          f"{roi_megapixels:.2f} Mpx "
                          f"({100 * roi_megapixels / megapixels:.0f}% of frame)"))

    try:
        from cv_bridge import CvBridge
    except ImportError:
        lines.append("cv_bridge not importable -- skipping message encode/decode")
    else:
        bridge = CvBridge()
        encoding = "mono8" if sample.ndim == 2 else "bgr8"
        # a rotation of distinct messages, for the same cache-realism reason
        messages = [bridge.cv2_to_imgmsg(image, encoding=encoding)
                    for image in images[:20]]
        lines.append(_row("cv_bridge encode (publisher)", _time_over(
            images, lambda image: bridge.cv2_to_imgmsg(image, encoding=encoding)),
            f"{sample.nbytes / 1e6:.2f} MB/frame -- does NOT shrink with ROI"))
        lines.append(_row("cv_bridge decode (subscriber)", _time_over(
            messages, lambda m: bridge.imgmsg_to_cv2(m, desired_encoding=encoding)),
            "likewise full-frame"))
        for rate in (100, 200):
            lines.append("%-34s %9s %12s  %s" % (
                f"  bandwidth at {rate} Hz x2 cameras", "", "",
                f"{sample.nbytes * rate * 2 / 1e6:.0f} MB/s"))

    if checkerboard_file and plumbline_file:
        from .triangulator_node import (_load_checkerboard, _load_plumbline,
                                        triangulate_point)
        calibration = _load_checkerboard(checkerboard_file)
        plumbline = _load_plumbline(plumbline_file)
        centre_x, centre_y = width / 2.0, height / 2.0
        jitter = [(centre_x + i % 50, centre_y + i % 37) for i in range(200)]
        triangulate_seconds = _time_over(
            jitter, lambda xy: triangulate_point(calibration, plumbline,
                                                 xy[0], xy[1], xy[0], xy[1]))
        lines.append(_row("triangulate one pair", triangulate_seconds,
                          "per stereo pair, not per camera"))

    if detect_roi_seconds is not None:
        per_camera = detect_roi_seconds + decode_seconds
        lines.append("")
        lines.append("tracker per camera (decode + detect at ROI): "
                     "%.2f ms -> %.0f Hz ceiling" % (per_camera * 1000, 1 / per_camera))
        lines.append("NB: each camera's tracker is its own process, so these run in "
                     "parallel given a free core each.")

    log("\n".join(lines))
    return lines


class Benchmark(Node):

    def __init__(self):
        super().__init__("benchmark")
        frames_dir = str(self.declare_parameter("frames", "").value)
        pattern = str(self.declare_parameter("pattern", "*.bmp").value)
        roi = parse_roi(self.declare_parameter("roi", "").value)
        n_frames = int(self.declare_parameter("n_frames", 150).value)
        detect_kwargs = dict(
            diff_threshold=int(self.declare_parameter("diff_threshold", 25).value),
            min_area=float(self.declare_parameter("min_area_px", 4.0).value),
            max_area=float(self.declare_parameter("max_area_px", 5000.0).value),
            morph_kernel=int(self.declare_parameter("morph_kernel", 3).value),
        )
        checkerboard_file = str(self.declare_parameter("checkerboard_file", "").value)
        plumbline_file = str(self.declare_parameter("plumbline_file", "").value)

        if not frames_dir:
            raise ValueError(
                "benchmark needs 'frames' -- a directory of frame images to time "
                "against, e.g. -p frames:=/path/to/session/cam_a"
            )
        run_benchmark(self, frames_dir, pattern, roi, n_frames, detect_kwargs,
                      checkerboard_file, plumbline_file)


def main(args=None):
    rclpy.init(args=args)
    node = Benchmark()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
