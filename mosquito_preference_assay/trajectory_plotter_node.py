#!/usr/bin/env python3
"""Live 3D scatter/trail plot of a position stream -- watch a trajectory being
computed in real time instead of only reading numbers off a topic echo.

Subscribes to geometry_msgs/PointStamped (the same message type `tracker`
already publishes for 2D + area, and the natural shape for a future 3D
triangulation node's real x/y/z output) and redraws a matplotlib 3D axes: a
faint trail through the last `max_points` positions, colored oldest-to-newest,
with the single latest point highlighted in red.

The axis box does NOT rescale to whatever's currently on screen -- that would
make it visibly resize/jitter every redraw as the trailing window slides.
Instead each axis grows to fit the full trajectory seen so far and then holds
still (expands when a point falls outside it, never shrinks back), or, if
`xlim`/`ylim`/`zlim` is given, stays at that fixed range from the start.

Standalone visualizer, not a control-flow node -- it never publishes anything,
so it's safe to leave running against anything that emits PointStamped. Use
`synthetic_trajectory_publisher` to test it before a real 3D node exists.

    ros2 run mosquito_preference_assay trajectory_plotter --ros-args \\
        -p topic:=/tracking/position_3d -p max_points:=800

Parameters:
    topic          string  /tracking/position_3d   geometry_msgs/PointStamped input
    queue_depth     int     50          subscription queue depth
    max_points      int     500         trailing window kept/drawn; <=0 = unbounded
    redraw_hz       double  15.0        plot refresh rate (decoupled from message rate)
    title           string  "mosquito_preference_assay -- 3D trajectory"
    xlim / ylim / zlim  string  ""      "min,max" -- fixed axis range, known up front
                                        (e.g. the real arena size); "" = grow-to-fit
                                        the full trajectory and then hold still

A display is required (matplotlib GUI backend -- TkAgg/Qt5Agg/QtAgg). Closing
the plot window shuts the node down; so does Ctrl-C in the terminal.
"""

import threading
import time

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def _parse_range(text):
    text = (text or "").strip()
    if not text:
        return None
    lo, hi = (float(v) for v in text.split(","))
    return (lo, hi)


class TrajectoryPlotter(Node):

    def __init__(self):
        super().__init__("trajectory_plotter")

        self._topic = str(self.declare_parameter(
            "topic", "/tracking/position_3d").value)
        queue_depth = int(self.declare_parameter("queue_depth", 50).value)
        max_points = int(self.declare_parameter("max_points", 500).value)
        self._redraw_hz = float(self.declare_parameter("redraw_hz", 15.0).value)
        self._title = str(self.declare_parameter(
            "title", "mosquito_preference_assay -- 3D trajectory").value)
        self._xlim = _parse_range(self.declare_parameter("xlim", "").value)
        self._ylim = _parse_range(self.declare_parameter("ylim", "").value)
        self._zlim = _parse_range(self.declare_parameter("zlim", "").value)

        self._maxlen = max_points if max_points > 0 else None
        self._lock = threading.Lock()
        self._xs = []
        self._ys = []
        self._zs = []
        self._n_received = 0
        self._last_wall = None
        # running (x, y, z) bounds over EVERY point ever received, independent
        # of max_points -- this is what makes the axis box hold still instead
        # of tracking whatever's currently in the trailing window
        self._bounds = [[float("inf"), float("-inf")] for _ in range(3)]

        self.create_subscription(
            PointStamped, self._topic, self._on_point, queue_depth)

        self.get_logger().info(
            f"plotting '{self._topic}' (max_points={max_points}, "
            f"redraw_hz={self._redraw_hz}) -- close the plot window or Ctrl-C to stop"
        )

    def _on_point(self, msg):
        with self._lock:
            self._xs.append(msg.point.x)
            self._ys.append(msg.point.y)
            self._zs.append(msg.point.z)
            self._n_received += 1
            self._last_wall = time.monotonic()
            if self._maxlen is not None:
                self._xs = self._xs[-self._maxlen:]
                self._ys = self._ys[-self._maxlen:]
                self._zs = self._zs[-self._maxlen:]
            for bound, v in zip(self._bounds, (msg.point.x, msg.point.y, msg.point.z)):
                bound[0] = min(bound[0], v)
                bound[1] = max(bound[1], v)

    def snapshot(self):
        with self._lock:
            return (list(self._xs), list(self._ys), list(self._zs),
                    self._n_received, self._last_wall,
                    [tuple(b) for b in self._bounds])

    def run_plot(self):
        """Block on the matplotlib GUI loop; returns when the window closes."""
        fig = plt.figure(figsize=(7.5, 7.5))
        ax = fig.add_subplot(111, projection="3d")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        fig.suptitle(self._title)

        scatter = ax.scatter([], [], [], c=[], cmap="viridis", s=14, depthshade=True)
        (trail,) = ax.plot([], [], [], color="0.6", linewidth=0.8, zorder=1)
        (head,) = ax.plot([], [], [], "o", color="red", markersize=9, zorder=3)
        status = ax.text2D(0.02, 0.98, "", transform=ax.transAxes, va="top", fontsize=9)

        def _apply_axis_limits(bounds):
            for lim, (lo, hi), setter in (
                (self._xlim, bounds[0], ax.set_xlim3d),
                (self._ylim, bounds[1], ax.set_ylim3d),
                (self._zlim, bounds[2], ax.set_zlim3d),
            ):
                if lim is not None:
                    setter(*lim)
                    continue
                pad = (hi - lo) * 0.1 or 1.0
                setter(lo - pad, hi + pad)

        def _update(_frame):
            xs, ys, zs, n_total, last_wall, bounds = self.snapshot()
            if not xs:
                status.set_text(f"waiting for messages on '{self._topic}' ...")
                return scatter, trail, head, status

            scatter._offsets3d = (xs, ys, zs)
            scatter.set_array(np.arange(len(xs)))
            trail.set_data(xs, ys)
            trail.set_3d_properties(zs)
            head.set_data([xs[-1]], [ys[-1]])
            head.set_3d_properties([zs[-1]])
            _apply_axis_limits(bounds)

            age = time.monotonic() - last_wall if last_wall else float("inf")
            status.set_text(
                f"{n_total} pts received, {len(xs)} shown\n"
                f"latest: ({xs[-1]:.2f}, {ys[-1]:.2f}, {zs[-1]:.2f})  "
                f"{age:.2f}s ago"
            )
            return scatter, trail, head, status

        # keep a reference on self -- FuncAnimation is GC'd (and stops
        # ticking) otherwise, since nothing else holds it
        self._ani = animation.FuncAnimation(
            fig, _update, interval=1000.0 / max(self._redraw_hz, 1e-3),
            blit=False, cache_frame_data=False,
        )
        plt.tight_layout()
        plt.show()


def _spin(node):
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()
    spin_thread = threading.Thread(target=_spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run_plot()  # blocks until the plot window is closed
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
