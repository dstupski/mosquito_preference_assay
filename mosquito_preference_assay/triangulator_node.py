#!/usr/bin/env python3
"""Turn a synchronized stereo 2D pair into a real-time 3D position -- the
last step of the pipeline: dual_video_publisher -> tracker x2 -> stereo_sync
-> HERE -> trajectory_plotter.

The triangulation math (undistort -> cv.triangulatePoints -> axis remap ->
plumbline rotation, plus the reprojection-error quality check) is ported
line-for-line from validate_mosquito_centroid_tracking_vid_output.py in the
test_videos_particle_tracking calibration set, just run per stereo_track
message instead of over a whole recorded array. Same two calibration files,
same array layout:

  Checkerboard_<date>.npy  (27, 5) float64, packed:
    [0:3, 0:3]    camera-0 intrinsic matrix (K0)
    [3:6, 0:3]    camera-1 intrinsic matrix (K1)
    [12:15, 0:4]  camera-0 projection matrix (P0 = K0 [I|0], cam0 is the
                  reference frame)
    [15:18, 0:4]  camera-1 projection matrix (P1 = K1 [R|t] into cam0's frame)
    [18:19, 0:5]  camera-0 distortion coefficients
    [19:20, 0:5]  camera-1 distortion coefficients
    (rows 6:12 and 20:27 are present in the file but unused here, same as
    in the source script)
  Plumbline_<date>.npy  (3, 3) float64 -- rotates the triangulated point
    (after an axis remap: cam-frame (X,Y,Z) -> (X,Z,-Y)) into the
    gravity-aligned arena/world frame, in mm.

Calibration is rig-specific and NOT shipped with this package -- point
`checkerboard_file` / `plumbline_file` at wherever your rig's pair lives
(e.g. Calibration_files_FF-arena/ in test_videos_particle_tracking).

    ros2 run mosquito_preference_assay triangulator --ros-args \\
        -p checkerboard_file:=/path/Checkerboard_2025_April_10.npy \\
        -p plumbline_file:=/path/Plumbline_2025_April_10.npy

Parameters:
    topic                      string  /tracking/stereo_track   input (stereo_track/1 JSON)
    output_topic                string  /tracking/position_3d    geometry_msgs/PointStamped
    checkerboard_file            string  ""    REQUIRED -- path to Checkerboard_*.npy
    plumbline_file                string  ""    REQUIRED -- path to Plumbline_*.npy
    frame_id                      string  arena
    max_reprojection_error_px      double  0.0   drop a triangulation whose reprojection
                                                 error exceeds this (px); <= 0 = no filtering
    log_every_n                    int     200   log the achieved rate + reprojection error
                                                 every N publishes (0 disables)
"""

import json
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

# cam-frame (X, Y, Z) -> (X, Z, -Y), applied before the plumbline rotation --
# identical to axis_remap in the source validation script.
_AXIS_REMAP = np.array([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, -1.0, 0.0, 0.0],
])


def _load_checkerboard(path):
    array = np.load(path, allow_pickle=False)
    if array.shape != (27, 5):
        raise ValueError(f"'{path}': expected a (27, 5) Checkerboard array, got {array.shape}")
    return {
        "matrix0": array[0:3, 0:3],
        "matrix1": array[3:6, 0:3],
        "projection0": array[12:15, 0:4],
        "projection1": array[15:18, 0:4],
        "distortion0": array[18:19, 0:5],
        "distortion1": array[19:20, 0:5],
    }


def _load_plumbline(path):
    array = np.load(path, allow_pickle=False)
    if array.shape != (3, 3):
        raise ValueError(f"'{path}': expected a (3, 3) Plumbline array, got {array.shape}")
    return array


def triangulate_point(calib, plumbline, x0, y0, x1, y1):
    """One (cam0 px, cam1 px) pair -> (world_xyz_mm, reprojection_error_px)."""
    point0 = np.array([[[x0, y0]]], dtype=np.float64)
    point1 = np.array([[[x1, y1]]], dtype=np.float64)
    undistorted0 = cv2.undistortPoints(
        point0, calib["matrix0"], calib["distortion0"], R=np.eye(3), P=calib["matrix0"]
    ).reshape(2)
    undistorted1 = cv2.undistortPoints(
        point1, calib["matrix1"], calib["distortion1"], R=np.eye(3), P=calib["matrix1"]
    ).reshape(2)

    homogeneous = cv2.triangulatePoints(
        calib["projection0"], calib["projection1"],
        undistorted0.reshape(2, 1), undistorted1.reshape(2, 1),
    )
    homogeneous = homogeneous / homogeneous[3]
    world = (plumbline @ _AXIS_REMAP @ homogeneous).reshape(3)

    projected0 = calib["projection0"] @ homogeneous
    projected1 = calib["projection1"] @ homogeneous
    projected0 = (projected0[:2] / projected0[2]).reshape(2)
    projected1 = (projected1[:2] / projected1[2]).reshape(2)
    squared_error = (np.sum((projected0 - undistorted0) ** 2)
                     + np.sum((projected1 - undistorted1) ** 2))
    reprojection_error = float(np.sqrt(squared_error / 2))
    return world, reprojection_error


class Triangulator(Node):

    def __init__(self):
        super().__init__("triangulator")

        in_topic = str(self.declare_parameter("topic", "/tracking/stereo_track").value)
        out_topic = str(self.declare_parameter("output_topic", "/tracking/position_3d").value)
        checkerboard_file = str(self.declare_parameter("checkerboard_file", "").value)
        plumbline_file = str(self.declare_parameter("plumbline_file", "").value)
        self._frame_id = str(self.declare_parameter("frame_id", "arena").value)
        self._max_reprojection_error = float(
            self.declare_parameter("max_reprojection_error_px", 0.0).value)
        self._log_every_n = int(self.declare_parameter("log_every_n", 200).value)

        if not checkerboard_file or not plumbline_file:
            raise ValueError(
                "triangulator needs 'checkerboard_file' and 'plumbline_file' -- "
                "rig-specific calibration, not shipped with this package. "
                "-p checkerboard_file:=/path/Checkerboard_<date>.npy "
                "-p plumbline_file:=/path/Plumbline_<date>.npy"
            )
        self._calib = _load_checkerboard(checkerboard_file)
        self._plumbline = _load_plumbline(plumbline_file)

        self._pub = self.create_publisher(PointStamped, out_topic, 10)
        self.create_subscription(String, in_topic, self._on_stereo_track, 10)

        self._n_published = 0
        self._n_dropped = 0
        self._last_reprojection_error = float("nan")
        self._log_window_start = time.monotonic()

        self.get_logger().info(
            f"triangulating '{in_topic}' -> '{out_topic}' using "
            f"'{checkerboard_file}' + '{plumbline_file}'"
        )

    def _on_stereo_track(self, msg):
        try:
            event = json.loads(msg.data)
            a, b = event["a"], event["b"]
            if not a.get("detected") or not b.get("detected"):
                return
            x0, y0 = float(a["x"]), float(a["y"])
            x1, y1 = float(b["x"]), float(b["y"])
            frame_stamp = float(event["frame_stamp"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warn(f"malformed stereo_track message: {exc!r}")
            return

        world, reprojection_error = triangulate_point(self._calib, self._plumbline, x0, y0, x1, y1)
        self._last_reprojection_error = reprojection_error
        if (self._max_reprojection_error > 0
                and reprojection_error > self._max_reprojection_error):
            self._n_dropped += 1
            return

        point = PointStamped()
        sec = int(frame_stamp)
        point.header.stamp.sec = sec
        point.header.stamp.nanosec = int(round((frame_stamp - sec) * 1e9))
        point.header.frame_id = self._frame_id
        point.point.x = float(world[0])
        point.point.y = float(world[1])
        point.point.z = float(world[2])
        self._pub.publish(point)

        self._n_published += 1
        if self._log_every_n and self._n_published % self._log_every_n == 0:
            now = time.monotonic()
            rate = self._log_every_n / max(1e-9, now - self._log_window_start)
            self.get_logger().info(
                f"{self._n_published} 3D points published, {self._n_dropped} dropped "
                f"(reprojection error) (~{rate:.1f} Hz over the last {self._log_every_n}, "
                f"last reprojection error {self._last_reprojection_error:.2f} px)"
            )
            self._log_window_start = now


def main(args=None):
    rclpy.init(args=args)
    node = Triangulator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
