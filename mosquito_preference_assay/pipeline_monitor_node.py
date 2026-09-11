#!/usr/bin/env python3
"""Live end-to-end latency / throughput monitor for the tracking pipeline --
run it alongside everything else to see, in real time, what rate each stage is
achieving, how far behind real time it is, and where frames are being lost.

Every stage carries the ORIGINAL camera-frame stamp forward (tracker copies
the image's header.stamp onto its PointStamped, stereo_sync passes it through
as frame_stamp, triangulator rebuilds a header from it), so for any stage

    lag = arrival wall time - message's frame stamp

is the true age of that data: capture -> this stage -> delivered. Comparing
lag across stages localizes where latency accumulates, and comparing message
counts shows which stage is dropping frames.

    ros2 run mosquito_preference_assay pipeline_monitor

Sample report:

    === pipeline report (5.0 s window) ===
    stage                        msgs   rate Hz    lag ms  med   p90   p99   max
    cam_a position                750     150.0            3.1   4.2   6.0   8.1
    cam_b position                748     149.6            3.3   4.4   6.2   8.5
    stereo pairs                  745     149.0            4.0   5.1   7.0   9.2
    3D positions                  745     149.0            4.3   5.5   7.4   9.6
    yield: pairs/cam_a 99.3%, 3D/pairs 100.0%

IMPORTANT -- clock: lag is wall clock minus stamp, so it is only meaningful
when whatever stamped the frames shares this machine's clock. Same box: fine.
Camera host elsewhere on the network: sync them (PTP/chrony) or the number is
clock offset, not latency.

IMPORTANT -- observer effect: watching the image topics means this node
receives every full frame (1.5 MB each at 1440x1080), which at 200 Hz x2 is
real bandwidth and can slow the very pipeline you are measuring. So
watch_images defaults to FALSE -- the tracking topics are tiny and free to
watch. Turn it on only when you want input-rate accounting and can afford it.

Parameters:
    image_topic_a / image_topic_b   string  /cam_a/image_raw / /cam_b/image_raw
    position_topic_a / position_topic_b  string  /tracking/cam_{a,b}/position
    stereo_topic                    string  /tracking/stereo_track
    position_3d_topic                string  /tracking/position_3d
    watch_images                     bool    False   also measure the image topics
                                                     (costs full-frame bandwidth)
    image_qos                        string  sensor_data   reliable | sensor_data
    report_period_sec                 double  5.0     how often to report (0 = only
                                                      on shutdown)
    publish_report                    bool    True    also publish each report as
                                                      JSON on ~/report, so a bag of a
                                                      run carries its own timings
"""

import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

SCHEMA = "mosquito_preference_assay/pipeline_report/1"


class _Stage:
    """Rolling lag/count bookkeeping for one pipeline stage."""

    def __init__(self, label):
        self.label = label
        self.lags = []
        self.count_window = 0
        self.count_total = 0

    def record(self, stamp_sec, arrival_sec):
        self.count_window += 1
        self.count_total += 1
        if stamp_sec:
            self.lags.append(arrival_sec - stamp_sec)

    def summarize(self, elapsed):
        lags_ms = np.array(self.lags) * 1000.0
        summary = {
            "label": self.label,
            "msgs": self.count_window,
            "total": self.count_total,
            "rate_hz": self.count_window / elapsed if elapsed > 0 else 0.0,
        }
        if len(lags_ms):
            summary.update({
                "lag_ms_median": float(np.median(lags_ms)),
                "lag_ms_p90": float(np.percentile(lags_ms, 90)),
                "lag_ms_p99": float(np.percentile(lags_ms, 99)),
                "lag_ms_max": float(lags_ms.max()),
                "lag_ms_min": float(lags_ms.min()),
            })
        return summary

    def reset(self):
        self.lags = []
        self.count_window = 0


def _stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec / 1e9


class PipelineMonitor(Node):

    def __init__(self):
        super().__init__("pipeline_monitor")

        image_topic_a = str(self.declare_parameter("image_topic_a", "/cam_a/image_raw").value)
        image_topic_b = str(self.declare_parameter("image_topic_b", "/cam_b/image_raw").value)
        position_a = str(self.declare_parameter(
            "position_topic_a", "/tracking/cam_a/position").value)
        position_b = str(self.declare_parameter(
            "position_topic_b", "/tracking/cam_b/position").value)
        stereo_topic = str(self.declare_parameter(
            "stereo_topic", "/tracking/stereo_track").value)
        position_3d_topic = str(self.declare_parameter(
            "position_3d_topic", "/tracking/position_3d").value)
        watch_images = bool(self.declare_parameter("watch_images", False).value)
        image_qos_kind = str(self.declare_parameter("image_qos", "sensor_data").value)
        report_period_sec = float(self.declare_parameter("report_period_sec", 5.0).value)
        self._publish_report = bool(self.declare_parameter("publish_report", True).value)

        self._stages = {}
        image_qos = qos_profile_sensor_data if image_qos_kind == "sensor_data" else 10
        if watch_images:
            self._add(Image, image_topic_a, "cam_a images", image_qos)
            self._add(Image, image_topic_b, "cam_b images", image_qos)
        self._add(PointStamped, position_a, "cam_a position", 50)
        self._add(PointStamped, position_b, "cam_b position", 50)
        self._add(String, stereo_topic, "stereo pairs", 50)
        self._add(PointStamped, position_3d_topic, "3D positions", 50)

        self._report_pub = (
            self.create_publisher(String, "~/report", 10) if self._publish_report else None
        )
        self._window_start = time.monotonic()
        self._run_start = self._window_start
        if report_period_sec > 0:
            self.create_timer(report_period_sec, self._report)

        watching = "images + tracking topics" if watch_images else "tracking topics only"
        self.get_logger().info(
            f"monitoring {watching}; reporting every {report_period_sec}s "
            f"(lag = wall clock - frame stamp, so clocks must be shared)"
        )

    def _add(self, msg_type, topic, label, qos):
        stage = _Stage(label)
        self._stages[topic] = stage
        if msg_type is String:
            def callback(msg, stage=stage):
                arrival = time.time()
                try:
                    stamp = float(json.loads(msg.data).get("frame_stamp") or 0.0)
                except (json.JSONDecodeError, TypeError, ValueError):
                    stamp = 0.0
                stage.record(stamp, arrival)
        else:
            def callback(msg, stage=stage):
                stage.record(_stamp_to_sec(msg.header.stamp), time.time())
        self.create_subscription(msg_type, topic, callback, qos)

    def _report(self):
        now = time.monotonic()
        elapsed = now - self._window_start
        summaries = [stage.summarize(elapsed) for stage in self._stages.values()]

        lines = [f"=== pipeline report ({elapsed:.1f} s window) ===",
                 "%-18s %7s %9s %8s %7s %7s %7s"
                 % ("stage", "msgs", "rate Hz", "lag med", "p90", "p99", "max")]
        for s in summaries:
            if "lag_ms_median" in s:
                lines.append("%-18s %7d %9.1f %8.1f %7.1f %7.1f %7.1f" % (
                    s["label"], s["msgs"], s["rate_hz"], s["lag_ms_median"],
                    s["lag_ms_p90"], s["lag_ms_p99"], s["lag_ms_max"]))
            else:
                lines.append("%-18s %7d %9.1f %8s" % (
                    s["label"], s["msgs"], s["rate_hz"], "-- no msgs --"))

        yields = self._yields()
        if yields:
            lines.append("yield: " + ", ".join(f"{k} {v:.1f}%" for k, v in yields.items()))
        self.get_logger().info("\n".join(lines))

        if self._report_pub is not None:
            msg = String()
            msg.data = json.dumps({
                "schema": SCHEMA,
                "stamp_wall": time.time(),
                "window_sec": elapsed,
                "uptime_sec": now - self._run_start,
                "stages": summaries,
                "yield_pct": yields,
            }, separators=(",", ":"))
            self._report_pub.publish(msg)

        for stage in self._stages.values():
            stage.reset()
        self._window_start = now

    def _yields(self):
        """Stage-to-stage survival rates, over the whole run -- where frames go."""
        by_label = {s.label: s.count_total for s in self._stages.values()}
        out = {}

        def ratio(name, numerator, denominator):
            if by_label.get(denominator):
                out[name] = 100.0 * by_label.get(numerator, 0) / by_label[denominator]

        ratio("cam_a detect/frame", "cam_a position", "cam_a images")
        ratio("cam_b detect/frame", "cam_b position", "cam_b images")
        ratio("pairs/cam_a", "stereo pairs", "cam_a position")
        ratio("3D/pairs", "3D positions", "stereo pairs")
        return out


def main(args=None):
    rclpy.init(args=args)
    node = PipelineMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._report()  # final window, so a short run still reports something
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
