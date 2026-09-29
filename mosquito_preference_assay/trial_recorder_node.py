"""Own both bags, so recording begins at the TRIGGER and ends with the TRIAL.

    trigger    ──> both bags start: the assay topics, and the camera feeds
    trial ends ──> both closed, the assay bag after a short grace so the
                   final `phase: complete` message lands in it

Nothing is written while the rig sits armed waiting for an animal, and nothing
is written after the trial -- so the display can be held up between animals
(`hold_after_trial`) without either bag growing.

Starting at the trigger rather than at launch costs a measured 0.16 s before
the first frame lands, because rosbag2 has to discover and subscribe to topics
that are already publishing. Nothing is lost to that gap, because everything
published before it is LATCHED (TRANSIENT_LOCAL): experiment_info,
stimulus_state, trial_start, and the detection event itself. Verified
directly -- a recorder started after a latched message was published still
records it, where a volatile message published at the same instant is lost.
That is why mosquito_detector publishes its event latched; if that changes,
the message that fired the trial stops appearing in its own bag.

Raw video additionally cannot be treated like the other topics: two 1440x1080
feeds at 200 fps is ~620 MB/s, so recording from launch would cost ~2.2 TB per
hour of waiting, and buffering 15 s in RAM (what `--snapshot-mode` does) would
need ~9.3 GB.

rosbag2 0.15.12 offers no pause/resume service and its keyboard control is
dead under a launch file, so each recorder is a child process stopped with
SIGINT -- never kill, which would leave a bag with no metadata.yaml that
`ros2 bag info` and playback both refuse.
"""

import json
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String


class Recorder:
    """One `ros2 bag record` child process."""

    def __init__(self, node, name, bag_dir, args):
        self.node, self.name, self.bag_dir, self.args = node, name, bag_dir, args
        self.proc = None
        self.started_at = None
        self.closed = False

    def start(self):
        if self.proc is not None or not self.bag_dir:
            return
        os.makedirs(os.path.dirname(os.path.abspath(self.bag_dir)) or ".",
                    exist_ok=True)
        cmd = ["ros2", "bag", "record", "-o", self.bag_dir] + self.args
        self.started_at = time.time()
        # start_new_session: the launch system SIGINTs the whole process
        # group, and these children must be stopped deliberately by us
        # instead, so the stop order and the assay bag's grace are respected.
        self.proc = subprocess.Popen(cmd, start_new_session=True)
        self.node.get_logger().info(
            f"  {self.name} bag -> {self.bag_dir} (pid {self.proc.pid})")

    def stop(self, timeout):
        if self.closed:
            return
        self.closed = True
        if self.proc is None:
            return
        if self.proc.poll() is not None:
            self.node.get_logger().warn(
                f"{self.name} recorder had already exited "
                f"(code {self.proc.returncode})")
            return
        # SIGINT so rosbag2 flushes its cache and writes metadata.yaml.
        self.proc.send_signal(signal.SIGINT)
        try:
            self.proc.wait(timeout=timeout)
            held = time.time() - (self.started_at or time.time())
            self.node.get_logger().info(
                f"  {self.name} bag closed after {held:.1f} s: {self.bag_dir}")
        except subprocess.TimeoutExpired:
            self.node.get_logger().error(
                f"{self.name} recorder did not finish within {timeout} s -- "
                f"killing it; that bag may lack metadata.yaml")
            self.proc.kill()


class TrialRecorderNode(Node):
    def __init__(self):
        super().__init__("trial_recorder")

        self.trigger_topic = str(
            self.declare_parameter("trigger_topic", "/arena/mosquito_present").value)
        msg_type = str(self.declare_parameter("trigger_msg_type", "string").value)
        self.state_topic = str(self.declare_parameter(
            "state_topic", "/stimulus_publisher/stimulus_state").value).strip()

        video_dir = str(self.declare_parameter("bag_dir", "").value).strip()
        assay_dir = str(self.declare_parameter("assay_bag_dir", "").value).strip()
        topics = self.declare_parameter("video_topics", [
            "/cam_sync/cam0/image_raw", "/cam_sync/cam1/image_raw"]).value
        video_topics = [str(t) for t in topics if str(t).strip()]
        # Regex of topics the assay bag must NOT take -- the camera feeds,
        # which the video bag owns.
        exclude = str(self.declare_parameter("assay_exclude", "").value).strip()

        self.stop_timeout = float(self.declare_parameter("stop_timeout", 20.0).value)
        # The final `phase: complete` message is published at the same instant
        # the trial ends. Closing the assay bag immediately would race it out
        # of the recording.
        self.close_grace_sec = float(
            self.declare_parameter("close_grace_sec", 1.5).value)

        self.video = Recorder(self, "video", video_dir, video_topics)
        self.assay = Recorder(self, "assay", assay_dir,
                              ["-a"] + (["-x", exclude] if exclude else []))
        self._grace_timer = None
        # Set the instant completion is seen, NOT when the assay bag
        # actually closes: between those two is close_grace_sec of 10 Hz
        # heartbeats still reporting "complete", and keying off
        # assay.closed would re-enter for every one of them -- spawning a
        # fresh grace timer each time.
        self._closing = False

        kind = Bool if msg_type.lower() == "bool" else String
        self.create_subscription(kind, self.trigger_topic, self._on_trigger, 10)
        if self.state_topic:
            latched = QoSProfile(depth=1,
                                 reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, self.state_topic, self._on_state,
                                     latched)

        self.get_logger().info(
            f"trial_recorder ARMED on '{self.trigger_topic}' ({msg_type}) -- "
            f"nothing is being recorded yet.\n"
            f"  on trigger: assay -> {assay_dir or '(disabled)'}\n"
            f"              video -> {video_dir or '(disabled)'}"
            f"  [{', '.join(video_topics) or 'no topics'}]")

    def _on_trigger(self, msg):
        # A Bool trigger counts only when True; a String event always does.
        if isinstance(msg, Bool) and not msg.data:
            return
        if self.assay.proc is not None or self.video.proc is not None:
            return                                  # one trial, one pair of bags
        self.get_logger().info("TRIGGER -- recording starts now")
        self.assay.start()
        self.video.start()

    def _on_state(self, msg):
        try:
            phase = json.loads(msg.data).get("phase")
        except (ValueError, AttributeError):
            return
        # stimulus_state is a 10 Hz heartbeat that keeps saying "complete" for
        # as long as the display is held, so act once.
        if phase != "complete" or self._closing:
            return
        self._closing = True

        self.get_logger().info(
            "trial complete -- closing both bags. The display may stay up; "
            "nothing further is recorded.")
        self.video.stop(self.stop_timeout)
        # The assay bag gets a grace period so the final state message lands.
        self._grace_timer = self.create_timer(
            self.close_grace_sec, self._close_assay)

    def _close_assay(self):
        if self._grace_timer is not None:
            self._grace_timer.cancel()
            self._grace_timer = None
        self.assay.stop(self.stop_timeout)

    def stop_all(self):
        if self.assay.proc is None and self.video.proc is None:
            self.get_logger().info("no trigger fired -- no bags written")
        self.video.stop(self.stop_timeout)
        self.assay.stop(self.stop_timeout)


def main(args=None):
    rclpy.init(args=args)
    node = TrialRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Idempotent: normally both are already closed at the trial's end.
        node.stop_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
