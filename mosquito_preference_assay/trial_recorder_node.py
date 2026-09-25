"""Start recording the camera feeds when the trigger fires, not before.

Raw video cannot be handled the way the assay topics are. Recording it from
launch means writing while the rig sits armed -- two 1440x1080 feeds at 200 fps
is ~620 MB/s, so an hour of waiting for a mosquito costs about 2.2 TB. And it
cannot be buffered in RAM either, which is what `--snapshot-mode` does: 15 s of
those same two feeds is ~9.3 GB.

So the video recorder is spawned at the trigger instead. This node watches the
detection topic and, on the first event, execs

    ros2 bag record -o <bag_dir> <topic> [<topic> ...]

then SIGINTs it at shutdown so rosbag2 flushes its cache and writes
metadata.yaml. The launch file shuts everything down when the trial ends, so
that exit is what closes the video bag.

The cost of this approach is a measured 0.16 s between the trigger and the
first recorded frame -- rosbag2 has to discover and subscribe to topics that
are already publishing. At 200 fps that is roughly the first 30 frames of the
trial. The assay's own topics do NOT have this gap: they are captured by the
continuous recorder that has been up since launch, so the detection event, the
stimulus definitions and the trial timing are all complete. What starts 0.16 s
late is the imagery.

Why a separate bag rather than a second recorder writing into the same one:
two rosbag2 processes cannot share an output directory. Both bags carry the
same timestamps, so they align on playback.
"""

import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class TrialRecorderNode(Node):
    def __init__(self):
        super().__init__("trial_recorder")

        self.trigger_topic = str(
            self.declare_parameter("trigger_topic", "/arena/mosquito_present").value)
        msg_type = str(self.declare_parameter("trigger_msg_type", "string").value)
        self.bag_dir = str(self.declare_parameter("bag_dir", "").value).strip()
        topics = self.declare_parameter("video_topics", [
            "/cam_sync/cam0/image_raw", "/cam_sync/cam1/image_raw"]).value
        self.video_topics = [str(t) for t in topics if str(t).strip()]
        # Seconds to wait for rosbag2 to finalize before giving up on it.
        self.stop_timeout = float(self.declare_parameter("stop_timeout", 20.0).value)

        self.proc = None
        self.started_at = None

        if not self.bag_dir:
            self.get_logger().error("bag_dir is empty -- refusing to record")
        if not self.video_topics:
            self.get_logger().error("video_topics is empty -- nothing to record")

        kind = Bool if msg_type.lower() == "bool" else String
        self.create_subscription(kind, self.trigger_topic, self._on_trigger, 10)
        self.get_logger().info(
            f"trial_recorder armed on '{self.trigger_topic}' ({msg_type}); "
            f"on trigger -> {self.bag_dir}\n  topics: {', '.join(self.video_topics)}")

    def _on_trigger(self, msg):
        # A Bool trigger only counts when it is True; a String event always
        # does. Same convention the sketch uses, so the pseudo-trigger and the
        # real detector both work here.
        if isinstance(msg, Bool) and not msg.data:
            return
        self.start()

    def start(self):
        if self.proc is not None:                # one trial, one video bag
            return
        if not self.bag_dir or not self.video_topics:
            return
        cmd = ["ros2", "bag", "record", "-o", self.bag_dir] + self.video_topics
        os.makedirs(os.path.dirname(os.path.abspath(self.bag_dir)) or ".",
                    exist_ok=True)
        self.started_at = time.time()
        # start_new_session so the launch system's SIGINT to the process group
        # does not race us -- we stop the child deliberately, in stop().
        self.proc = subprocess.Popen(cmd, start_new_session=True)
        self.get_logger().info(
            f"TRIGGER -> recording video to {self.bag_dir} (pid {self.proc.pid})")

    def stop(self):
        if self.proc is None:
            self.get_logger().info("no trigger fired -- no video bag written")
            return
        if self.proc.poll() is not None:
            self.get_logger().warn(
                f"video recorder already exited (code {self.proc.returncode})")
            return
        # SIGINT, not kill: rosbag2 writes remaining cache and metadata.yaml on
        # SIGINT. A killed recorder leaves a bag with no metadata, which
        # `ros2 bag info` and playback both refuse.
        self.proc.send_signal(signal.SIGINT)
        try:
            self.proc.wait(timeout=self.stop_timeout)
            held = time.time() - (self.started_at or time.time())
            self.get_logger().info(
                f"video bag closed after {held:.1f} s: {self.bag_dir}")
        except subprocess.TimeoutExpired:
            self.get_logger().error(
                f"video recorder did not finish within {self.stop_timeout} s -- "
                f"killing it; the bag may lack metadata.yaml")
            self.proc.kill()


def main(args=None):
    rclpy.init(args=args)
    node = TrialRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Runs on the launch system's shutdown, which is what ends the trial.
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
