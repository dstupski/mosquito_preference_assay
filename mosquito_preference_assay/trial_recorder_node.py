"""Own both bags, so recording begins at the TRIGGER and ends with the TRIAL.

    trigger    ──> both bags start: the assay topics, and the camera feeds
    trial ends ──> both closed, the assay bag after a short grace so the
                   final `phase: complete` message lands in it

Nothing is written while the rig sits armed waiting for an animal, and nothing
after the trial -- so the display can be held up between animals
(`hold_after_trial`) without either bag growing.

TWO DIFFERENT RECORDERS, because they have different problems.

*The assay bag is written in-process*, by this node. Spawning `ros2 bag
record` at the trigger costs 170-520 ms before the first message lands:
process start, participant creation, discovery, subscription matching -- all
of it work that could have been done while the rig sat armed doing nothing.
Measured, that lost the first ~0.5 s of a 15 s trial, which is exactly when
the animal is arriving.

So the subscriptions are created at LAUNCH and the callbacks simply drop what
arrives. At the trigger a rosbag2_py writer is opened and assigned, and the
same callbacks begin writing. Nothing to discover, nothing to start:
**measured 2 ms** from trigger to first message written, against 170-520 ms
for the spawned recorder.

Subscribing early means the LATCHED topics need care. experiment_info,
trial_start and stimulus_state are TRANSIENT_LOCAL and are published once,
before the trigger. A spawned recorder receives them on subscribe; this node
subscribed long ago and dropped them. So the latest message on each latched
topic is cached while armed and written first when the bag opens. Get that
wrong and the bag loses the experiment definition, which is far worse than a
slow start.

*The video bag stays a child process.* The same trick would mean subscribing
to ~620 MB/s of camera data for the whole armed period -- however long the
animal takes to arrive -- which is the cost the trigger-start design exists to
avoid. It keeps its 170-520 ms.

rosbag2 0.15.12 offers no pause/resume service and its keyboard control is
dead under a launch file, so the child recorder is stopped with SIGINT --
never kill, which would leave a bag with no metadata.yaml that `ros2 bag info`
and playback both refuse. The in-process writer has no close() either: it
flushes and writes metadata.yaml when the object is destroyed, so it is
dropped and gc'd deliberately.
"""

import gc
import datetime
import json
import os
import re
import signal
import subprocess
import time

import rclpy
import rosbag2_py
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Bool, String

# The topics whose first moments matter, subscribed at launch so the trigger
# costs nothing. (topic, type, latched). The trigger topic is added from the
# node's parameters. Everything else in the graph is picked up at the trigger
# instead, where its start-up latency does not matter.
CRITICAL_TOPICS = [
    ("/stimulus_publisher/experiment_info", "std_msgs/msg/String", True),
    ("/stimulus_publisher/trial_start", "std_msgs/msg/String", True),
    ("/stimulus_publisher/stimulus_state", "std_msgs/msg/String", True),
    ("/stimulus_publisher/stimulus_position", "std_msgs/msg/String", False),
]

_LATCHED_QOS = QoSProfile(depth=10,
                          reliability=QoSReliabilityPolicy.RELIABLE,
                          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
_LIVE_QOS = QoSProfile(depth=100, reliability=QoSReliabilityPolicy.RELIABLE)


class AssayWriter:
    """Pre-subscribed, in-process bag. Subscribe at launch, write from trigger."""

    def __init__(self, node, topics, exclude, enabled=True):
        self.node, self.exclude, self.enabled = node, exclude, enabled
        self.bag_dir = None
        self.writer = None
        self.closed = False
        self.started_at = None
        self.count = 0
        self._known = set()          # topics we have subscribed to
        self._types = {}             # topic -> message type string
        self._latched = {}           # topic -> (raw, stamp_ns), while armed
        self._subs = []

        for topic, type_str, latched in topics:
            self._subscribe(topic, type_str, latched)

    def _subscribe(self, topic, type_str, latched):
        if topic in self._known or not self.enabled:
            return
        try:
            msg_type = get_message(type_str)
        except (ImportError, AttributeError, ValueError) as exc:
            self.node.get_logger().warn(f"cannot record {topic} ({type_str}): {exc}")
            return
        # raw=True hands us the serialized bytes, so nothing is deserialized
        # and re-serialized on the way to disk.
        self._subs.append(self.node.create_subscription(
            msg_type, topic,
            lambda raw, t=topic, lat=latched: self._on_raw(t, raw, lat),
            _LATCHED_QOS if latched else _LIVE_QOS, raw=True))
        self._known.add(topic)
        self._types[topic] = type_str

    def _on_raw(self, topic, raw, latched):
        now = self.node.get_clock().now().nanoseconds
        if self.writer is None:
            # Armed: drop, except that the LAST message on a latched topic has
            # to survive -- it was published once, before the trigger, and will
            # never be sent again.
            if latched:
                self._latched[topic] = (raw, now)
            return
        self.writer.write(topic, raw, now)
        self.count += 1

    def start(self, bag_dir):
        """Open the bag. Runs in the executor thread, so no message can land
        between opening the writer and flushing the latched cache."""
        self.bag_dir = bag_dir
        if self.writer is not None or not self.bag_dir:
            return
        os.makedirs(os.path.dirname(os.path.abspath(self.bag_dir)) or ".",
                    exist_ok=True)
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=self.bag_dir, storage_id="sqlite3"),
                    rosbag2_py.ConverterOptions("cdr", "cdr"))
        for topic in self._known:
            writer.create_topic(rosbag2_py.TopicMetadata(
                name=topic, type=self._types[topic], serialization_format="cdr"))

        # Everything published before the trigger that still matters, in the
        # order it was received.
        for topic, (raw, stamp) in sorted(self._latched.items(),
                                          key=lambda kv: kv[1][1]):
            writer.write(topic, raw, stamp)
            self.count += 1

        self.started_at = time.time()
        self.writer = writer         # single assignment = the switch
        self.node.get_logger().info(
            f"  assay bag -> {self.bag_dir} "
            f"({len(self._latched)} latched messages carried over)")

    def reset(self):
        """Ready for another trial. The SUBSCRIPTIONS stay -- they are the
        whole point of this class, and re-creating them would reintroduce
        the discovery cost at the next trigger. Only the writer state is
        cleared; the latched cache keeps filling from the live
        subscriptions, so the next bag opens with that trial's own
        definition rather than the previous one's."""
        self.writer = None
        self.bag_dir = None
        self.closed = False
        self.started_at = None
        self.count = 0

    def adopt_remaining_topics(self):
        """Pick up everything else in the graph at the trigger. These get the
        start-up latency the critical topics were spared, which is fine --
        /rosout and /parameter_events have no meaningful first 300 ms."""
        if self.writer is None:
            return
        pattern = re.compile(self.exclude) if self.exclude else None
        for topic, types in self.node.get_topic_names_and_types():
            if topic in self._known or not types:
                continue
            if pattern and pattern.search(topic):
                continue             # the camera feeds -- the video bag owns them
            try:
                msg_type = get_message(types[0])
            except (ImportError, AttributeError, ValueError):
                continue
            self.writer.create_topic(rosbag2_py.TopicMetadata(
                name=topic, type=types[0], serialization_format="cdr"))
            self._types[topic] = types[0]
            self._known.add(topic)
            self._subs.append(self.node.create_subscription(
                msg_type, topic,
                lambda raw, t=topic: self._on_raw(t, raw, False),
                _LIVE_QOS, raw=True))

    def stop(self):
        if self.closed:
            return
        self.closed = True
        if self.writer is None:
            return
        held = time.time() - (self.started_at or time.time())
        # No close() on SequentialWriter: it flushes and writes metadata.yaml
        # when destroyed, so drop the reference and force collection.
        self.writer = None
        gc.collect()
        self.node.get_logger().info(
            f"  assay bag closed after {held:.1f} s, {self.count} messages: "
            f"{self.bag_dir}")


class Recorder:
    """One `ros2 bag record` child process -- used for the camera feeds."""

    def __init__(self, node, name, args):
        self.node, self.name, self.args = node, name, args
        self.bag_dir = None
        self.proc = None
        self.started_at = None
        self.closed = False

    def start(self, bag_dir):
        self.bag_dir = bag_dir
        if self.proc is not None or not self.bag_dir:
            return
        os.makedirs(os.path.dirname(os.path.abspath(self.bag_dir)) or ".",
                    exist_ok=True)
        cmd = ["ros2", "bag", "record", "-o", self.bag_dir] + self.args
        self.started_at = time.time()
        # start_new_session: the launch system SIGINTs the whole process
        # group, and this child must be stopped deliberately by us instead.
        self.proc = subprocess.Popen(cmd, start_new_session=True)
        self.node.get_logger().info(
            f"  {self.name} bag -> {self.bag_dir} (pid {self.proc.pid})")

    def reset(self):
        self.proc = None
        self.bag_dir = None
        self.closed = False
        self.started_at = None

    def stop(self, timeout):
        if self.closed:
            return
        self.closed = True
        if self.proc is None or self.proc.poll() is not None:
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

        # One folder PER TRIAL, named at the trigger -- the launch file
        # cannot name them, because it runs once and a session now holds
        # as many trials as you re-arm for. The timestamp is the trigger
        # time, which is also more meaningful than launch time.
        self.save_dir = str(self.declare_parameter("save_dir", "").value).strip()
        self.run_name = str(
            self.declare_parameter("run_name", "trial").value).strip() or "trial"
        self.want_video = bool(self.declare_parameter("record_video", True).value)
        topics = self.declare_parameter("video_topics", [
            "/cam_sync/cam0/image_raw", "/cam_sync/cam1/image_raw"]).value
        video_topics = [str(t) for t in topics if str(t).strip()]
        exclude = str(self.declare_parameter("assay_exclude", "").value).strip()

        self.stop_timeout = float(self.declare_parameter("stop_timeout", 20.0).value)
        # The final `phase: complete` is published at the same instant the
        # trial ends; closing immediately would race it out of the recording.
        self.close_grace_sec = float(
            self.declare_parameter("close_grace_sec", 1.5).value)

        critical = list(CRITICAL_TOPICS)
        if self.trigger_topic:
            # Latched, so that the detection which started the trial lands in
            # the bag it started.
            critical.append((self.trigger_topic, "std_msgs/msg/String", True))

        self.assay = AssayWriter(self, critical, exclude,
                                 enabled=bool(self.save_dir))
        self.video = Recorder(self, "video", video_topics)
        self.trial_no = 0
        self._grace_timer = None
        # Set when completion is first SEEN, not when the bag closes: between
        # them is close_grace_sec of 10 Hz heartbeats still reporting
        # "complete", each of which would otherwise spawn another timer.
        self._closing = False

        kind = Bool if msg_type.lower() == "bool" else String
        self.create_subscription(kind, self.trigger_topic, self._on_trigger, 10)
        if self.state_topic:
            self.create_subscription(String, self.state_topic, self._on_state,
                                     _LATCHED_QOS)

        self.get_logger().info(
            f"trial_recorder ARMED on '{self.trigger_topic}' ({msg_type}) -- "
            f"subscribed, recording nothing.\n"
            f"  each trial -> {self.save_dir or '(recording disabled)'}/"
            f"{self.run_name}_<timestamp>/{{assay,video}}\n"
            f"  video topics: "
            f"{', '.join(video_topics) if self.want_video else '(video off)'}")

    def _on_trigger(self, msg):
        # A Bool trigger counts only when True; a String event always does.
        if isinstance(msg, Bool) and not msg.data:
            return
        if self.assay.writer is not None or self.video.proc is not None:
            return                          # this trial already has its bags
        if not self.save_dir:
            self.get_logger().info("TRIGGER -- recording disabled, nothing written")
            return

        self.trial_no += 1
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = os.path.abspath(os.path.expanduser(
            os.path.join(self.save_dir, f"{self.run_name}_{stamp}")))
        self.get_logger().info(
            f"TRIGGER -- trial {self.trial_no}, recording to {run_dir}")

        # assay first: it is the one that must be fast
        self.assay.start(os.path.join(run_dir, "assay"))
        self.assay.adopt_remaining_topics()
        if self.want_video:
            self.video.start(os.path.join(run_dir, "video"))

    def _on_state(self, msg):
        try:
            phase = json.loads(msg.data).get("phase")
        except (ValueError, AttributeError):
            return
        # Re-armed for the next animal: let the next trigger open new bags.
        # _closing is a one-way latch within a trial, so without clearing it
        # here the SECOND trial would record and never close -- losing it.
        if phase == "armed" and self._closing and self.assay.closed:
            self._closing = False
            self.assay.reset()
            self.video.reset()
            self.get_logger().info("re-armed -- ready to record the next trial")
            return

        if phase != "complete" or self._closing:
            return
        self._closing = True
        self.get_logger().info(
            "trial complete -- closing both bags. The display may stay up; "
            "nothing further is recorded.")
        self.video.stop(self.stop_timeout)
        self._grace_timer = self.create_timer(
            self.close_grace_sec, self._close_assay)

    def _close_assay(self):
        if self._grace_timer is not None:
            self._grace_timer.cancel()
            self._grace_timer = None
        self.assay.stop()

    def stop_all(self):
        if self.assay.writer is None and self.video.proc is None:
            self.get_logger().info("no trigger fired -- no bags written")
        self.video.stop(self.stop_timeout)
        self.assay.stop()


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
