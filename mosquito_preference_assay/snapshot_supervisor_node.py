#!/usr/bin/env python3
"""Make a snapshot-mode rosbag actually write, at the moments worth keeping.

`ros2 bag record --snapshot-mode` keeps messages in a memory buffer and writes
nothing until /rosbag2_recorder/snapshot is called. That is what lets the
recorder be up and DDS-discovered from launch -- no start-up race when the
trigger fires -- while costing nothing on disk during the wait for an animal.

This node calls that service twice per trial:

  * when the trial starts -- flushing the buffer, which still holds the moments
    BEFORE the trigger, so the approach that caused the detection is kept;
  * when the trial completes -- flushing the trial itself.

Verified behavior of snapshot mode: a call writes the buffered messages and
recording continues, so the second call captures traffic from after the first.

The buffer is RAM, bounded by the recorder's --max-cache-size (100 MiB by
default). That is ample for the assay, detection and tracking topics, and far
too small for raw camera feeds -- 15 s of two 1440x1080 feeds is 9.3 GB at
200 fps. Record video with a continuous recorder instead (see
triggered_assay.launch.py's record_mode).

Parameters:
    trial_topic      string  /stimulus_publisher/trial_start   watched for trials
    state_topic      string  /stimulus_publisher/stimulus_state  watched for completion
    service          string  /rosbag2_recorder/snapshot
    wait_for_service_sec  double  10.0   how long to wait for the recorder at startup
"""

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rosbag2_interfaces.srv import Snapshot
from std_msgs.msg import String


class SnapshotSupervisor(Node):

    def __init__(self):
        super().__init__("snapshot_supervisor")

        trial_topic = str(self.declare_parameter(
            "trial_topic", "/stimulus_publisher/trial_start").value)
        state_topic = str(self.declare_parameter(
            "state_topic", "/stimulus_publisher/stimulus_state").value)
        service = str(self.declare_parameter(
            "service", "/rosbag2_recorder/snapshot").value)
        wait_sec = float(self.declare_parameter("wait_for_service_sec", 10.0).value)

        # the assay publishes latched, so subscribe the same way or the
        # already-published armed state is never seen
        latched = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._client = self.create_client(Snapshot, service)
        self._snapshots = 0
        self._completed = False

        if not self._client.wait_for_service(timeout_sec=wait_sec):
            self.get_logger().warn(
                f"no '{service}' after {wait_sec:.0f}s -- is the recorder running "
                f"with --snapshot-mode? Nothing will be written to the bag.")

        self.create_subscription(String, trial_topic, self._on_trial, latched)
        self.create_subscription(String, state_topic, self._on_state, latched)
        self.get_logger().info(
            f"watching '{trial_topic}' and '{state_topic}'; will call '{service}' "
            f"at trial start and at completion")

    def _on_trial(self, msg):
        try:
            event = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        if event.get("phase") != "running":
            return
        self._completed = False
        self._snapshot(f"trial {event.get('trial_id')} started")

    def _on_state(self, msg):
        try:
            event = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        if event.get("phase") == "complete" and not self._completed:
            self._completed = True
            self._snapshot("trial complete")

    def _snapshot(self, why):
        if not self._client.service_is_ready():
            self.get_logger().warn(f"snapshot service not ready ({why}) -- skipped")
            return
        future = self._client.call_async(Snapshot.Request())
        future.add_done_callback(lambda f, why=why: self._done(f, why))
        self._snapshots += 1

    def _done(self, future, why):
        try:
            success = future.result().success
        except Exception as exc:                        # noqa: BLE001
            self.get_logger().warn(f"snapshot call failed ({why}): {exc!r}")
            return
        level = self.get_logger().info if success else self.get_logger().warn
        level(f"snapshot #{self._snapshots} ({why}): "
              f"{'written' if success else 'REPORTED FAILURE'}")


def main(args=None):
    rclpy.init(args=args)
    node = SnapshotSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
