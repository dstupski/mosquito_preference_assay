#!/usr/bin/env python3
"""Publish a made-up moving 3D point -- so `trajectory_plotter` (and anything
else downstream) can be exercised before a real 3D triangulation node exists.

Publishes geometry_msgs/PointStamped, the same message shape a real 3D node
is expected to produce eventually (real x/y/z, no field-reuse hack needed
since there are three real dimensions to fill).

    ros2 run mosquito_preference_assay synthetic_trajectory_publisher
    ros2 run mosquito_preference_assay trajectory_plotter

Parameters:
    topic         string  /tracking/position_3d   geometry_msgs/PointStamped output
    frame_id      string  arena
    rate_hz       double  30.0
    pattern       string  lissajous     lissajous | helix | random_walk
    period_sec    double  8.0           seconds per cycle (lissajous / helix)
    scale_xy      double  50.0          x/y extent (same units as the eventual real data)
    scale_z       double  20.0          z extent
    z_offset      double  0.0           z center
    step_std      double  2.0           random_walk: per-tick gaussian step size
    seed          int     0             random_walk RNG seed; 0 = nondeterministic
    log_every_n    int     200          log the achieved publish rate every N (0 disables)
"""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class SyntheticTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__("synthetic_trajectory_publisher")

        out_topic = str(self.declare_parameter("topic", "/tracking/position_3d").value)
        self._frame_id = str(self.declare_parameter("frame_id", "arena").value)
        rate_hz = float(self.declare_parameter("rate_hz", 30.0).value)
        self._pattern = str(self.declare_parameter("pattern", "lissajous").value)
        self._period_sec = float(self.declare_parameter("period_sec", 8.0).value)
        self._scale_xy = float(self.declare_parameter("scale_xy", 50.0).value)
        self._scale_z = float(self.declare_parameter("scale_z", 20.0).value)
        self._z_offset = float(self.declare_parameter("z_offset", 0.0).value)
        self._step_std = float(self.declare_parameter("step_std", 2.0).value)
        seed = int(self.declare_parameter("seed", 0).value)
        self._log_every_n = int(self.declare_parameter("log_every_n", 200).value)

        if self._pattern not in ("lissajous", "helix", "random_walk"):
            raise ValueError(f"unknown pattern '{self._pattern}'")

        self._rng = np.random.default_rng(seed or None)
        self._pos = [0.0, 0.0, self._z_offset]  # current point, for random_walk
        self._t0 = time.monotonic()

        self._pub = self.create_publisher(PointStamped, out_topic, 10)
        self._n_published = 0
        self._log_window_start = time.monotonic()
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f"publishing a synthetic '{self._pattern}' trajectory to '{out_topic}' "
            f"at {rate_hz} Hz"
        )

    def _next_point(self):
        t = time.monotonic() - self._t0
        w = 2.0 * math.pi / self._period_sec

        if self._pattern == "lissajous":
            # different frequency ratios per axis -> a non-repeating-looking curve
            x = self._scale_xy * math.sin(w * t)
            y = self._scale_xy * math.sin(w * t * 1.5 + math.pi / 4)
            z = self._z_offset + self._scale_z * math.sin(w * t * 0.7)
            return x, y, z

        if self._pattern == "helix":
            x = self._scale_xy * math.cos(w * t)
            y = self._scale_xy * math.sin(w * t)
            # z ramps up over 3 cycles then wraps back down -- stays bounded
            frac = (t / (self._period_sec * 3.0)) % 1.0
            z = self._z_offset + self._scale_z * (frac * 2.0 - 1.0)
            return x, y, z

        # random_walk: gaussian step, reflected at the configured bounds
        step = self._rng.normal(0.0, self._step_std, 3)
        bounds = (
            (-self._scale_xy, self._scale_xy),
            (-self._scale_xy, self._scale_xy),
            (self._z_offset - self._scale_z, self._z_offset + self._scale_z),
        )
        for i, (lo, hi) in enumerate(bounds):
            v = self._pos[i] + step[i]
            if v < lo:
                v = lo + (lo - v)
            elif v > hi:
                v = hi - (v - hi)
            self._pos[i] = v
        return tuple(self._pos)

    def _tick(self):
        x, y, z = self._next_point()

        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = self._frame_id
        point.point.x = float(x)
        point.point.y = float(y)
        point.point.z = float(z)
        self._pub.publish(point)

        self._n_published += 1
        if self._log_every_n and self._n_published % self._log_every_n == 0:
            now = time.monotonic()
            rate = self._log_every_n / max(1e-9, now - self._log_window_start)
            self.get_logger().info(
                f"{self._n_published} points published "
                f"(~{rate:.1f} Hz over the last {self._log_every_n})"
            )
            self._log_window_start = now


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
