#!/usr/bin/env python3
"""ROS 2 node: run the py5 preference assay and publish, at all times, a
complete JSON description of which stimulus is on the LEFT and which is on the
RIGHT.

Topics (names are relative to the node, i.e. /stimulus_publisher/...), all
std_msgs/String carrying a JSON object, all latched (transient_local,
keep_last 1):

    experiment_info   published once at startup -- static run metadata:
                      experiment identity/sha1, seeds, pool, durations. Kept
                      out of every stimulus_state message.
                      schema "mosquito_preference_assay/experiment_info/1"

    stimulus_state    the current trial: ids + seed, condition, geometry, and
                      each side's name/type/params. Published on every trial
                      change and at ``heartbeat_hz``. Short (schema/phase/
                      run_id only) while ARMED.
                      schema "mosquito_preference_assay/stimulus_state/3"

    trial_start       the same stimulus_state object, once per new trial (and
                      on phase: complete) -- a convenient downstream trigger.

What is shown -- the stimulus pool, the pairings, the ordering and the timing
-- comes from an experiment YAML (see the experiments/ folder), selected with
the `experiment_file` parameter. The rest of the parameters are operational:

    experiment_file     string  ""    experiment name / path; "" -> built-in default
    start_mode          string  ""    "" -> derive from the experiment's `trigger:` block
                                      (present -> triggered, absent -> auto); or force
                                      "auto" / "triggered"
    trigger_topic       string  ""    "" -> the experiment's `trigger:` topic, else ~/trigger
    trigger_msg_type    string  ""    "" -> the experiment's; else bool. bool: std_msgs/Bool
                                      (true=start, false=abort). string: std_msgs/String, any
                                      message = start (e.g. mosquito_detector's detection event)
    master_seed         int     -1    -1 -> random seed (logged, in experiment_info)
    fullscreen          bool    False  True for the mosquito-facing display
    monitor             string  ""    "" -> primary; "2" -> that display; "span" -> all
    window_pos          string  ""    "x,y" px: place the (windowed) sketch here
    window_w / window_h int  1200/800  ignored when fullscreen
    left_center_px      string  ""    "x,y" px override of display.left_center_px
    right_center_px     string  ""    "x,y" px override of display.right_center_px
    heartbeat_hz        double  10.0   stimulus_state re-publish rate
    show_debug          bool    True   on-screen labels/timer overlay

Run:

    ros2 launch mosquito_preference_assay assay.launch.py
    ros2 run mosquito_preference_assay stimulus_publisher --ros-args \\
        -p experiment_file:=control_vs_grating -p fullscreen:=true
    ros2 bag record /stimulus_publisher/stimulus_state /stimulus_publisher/trial_start
"""

import json
import os
import sys

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool, String

try:
    from . import assay
    from .experiment import Experiment, ExperimentError
except ImportError as exc:  # pragma: no cover - environment guard
    raise ImportError(
        "Could not import the assay module. If this is 'No module named py5', "
        "install py5 into the same Python interpreter ROS uses -- see README.md."
    ) from exc

PACKAGE = "mosquito_preference_assay"


def _center_param(node, name):
    """Read a marker-centre override given as the string "x,y" (pixels).
    Empty -> None (use the experiment's display.*_center_px)."""
    raw = str(node.declare_parameter(name, "").value).strip()
    if not raw:
        return None
    try:
        x, y = (int(float(p)) for p in raw.split(","))
    except ValueError:
        node.get_logger().warn(f"{name}={raw!r} is not 'x,y'; ignoring")
        return None
    return [x, y]


def _resolve_experiment_file(spec):
    """Turn the experiment_file parameter into an absolute path (or None for
    the built-in default). Accepts an absolute path, a path relative to the
    cwd, or a bare name resolved against the installed experiments/ folder."""
    if not spec:
        return None
    if os.path.isabs(spec) and os.path.exists(spec):
        return spec
    if os.path.exists(spec):
        return os.path.abspath(spec)
    try:
        share = get_package_share_directory(PACKAGE)
    except PackageNotFoundError:
        share = None
    if share:
        for cand in (spec, f"{spec}.yaml"):
            path = os.path.join(share, "experiments", cand)
            if os.path.exists(path):
                return path
    raise ExperimentError(
        f"experiment_file {spec!r} not found (looked as a path and in "
        f"{PACKAGE}/experiments/)"
    )


class StimulusPublisher(Node):

    def __init__(self):
        super().__init__("stimulus_publisher")

        experiment_file = self.declare_parameter("experiment_file", "").value
        master_seed = self.declare_parameter("master_seed", -1).value
        fullscreen = self.declare_parameter("fullscreen", False).value
        monitor = str(self.declare_parameter("monitor", "").value).strip() or None
        heartbeat_hz = self.declare_parameter("heartbeat_hz", 10.0).value
        window_w = self.declare_parameter("window_w", 1200).value
        window_h = self.declare_parameter("window_h", 800).value
        show_debug = self.declare_parameter("show_debug", True).value
        left_center_px = _center_param(self, "left_center_px")
        right_center_px = _center_param(self, "right_center_px")
        window_pos = _center_param(self, "window_pos")
        start_mode = str(self.declare_parameter("start_mode", "").value).strip()
        trigger_topic = str(self.declare_parameter("trigger_topic", "").value).strip()
        trigger_msg_type = str(
            self.declare_parameter("trigger_msg_type", "").value).strip().lower()

        # Resolve the seed to a concrete value now, so ~/experiment_info can
        # carry it before the sketch thread runs setup().
        if master_seed is None or master_seed < 0:
            import random
            master_seed = random.SystemRandom().randrange(2 ** 32)
        else:
            master_seed = int(master_seed)

        path = _resolve_experiment_file(experiment_file)
        experiment = Experiment.from_file(path) if path else Experiment.default()
        if experiment.mode == "sample":
            how = f"mode=sample, pool={experiment.pool}"
        else:
            how = f"mode=pairs, {len(experiment.pairs)} pairings"
        self.get_logger().info(
            f"experiment {experiment.name!r} from {experiment.source} "
            f"(sha1 {experiment.sha1}, {how})"
        )

        # start_mode: explicit param wins; otherwise derive from the experiment
        # (a `trigger:` block -> triggered, else auto).
        if start_mode not in ("auto", "triggered"):
            if start_mode:
                self.get_logger().warn(f"start_mode {start_mode!r} invalid; deriving")
            start_mode = "triggered" if experiment.trigger_topic else "auto"

        # trigger_topic: explicit param wins; else the experiment's `trigger:`;
        # else the node-private ~/trigger.
        if not trigger_topic:
            trigger_topic = experiment.trigger_topic or "~/trigger"

        # trigger_msg_type: explicit param wins; else the experiment's; else bool.
        if trigger_msg_type not in ("bool", "string"):
            if trigger_msg_type:
                self.get_logger().warn(f"trigger_msg_type {trigger_msg_type!r} invalid; deriving")
            trigger_msg_type = experiment.trigger_msg_type

        assay.configure(
            experiment=experiment,
            master_seed=master_seed,
            fullscreen=bool(fullscreen),
            monitor=monitor,
            window_pos=window_pos,
            window_w=int(window_w),
            window_h=int(window_h),
            left_center_px=left_center_px,
            right_center_px=right_center_px,
            show_debug=bool(show_debug),
            start_mode=start_mode,
        )

        # Latched so a late subscriber / bag immediately learns the current state.
        latched = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._info_pub = self.create_publisher(String, "~/experiment_info", latched)
        self._state_pub = self.create_publisher(String, "~/stimulus_state", latched)
        self._trial_pub = self.create_publisher(String, "~/trial_start", latched)

        # Static run metadata (experiment, seeds, pool) -- published once here,
        # kept out of every stimulus_state message.
        info = String()
        info.data = assay.experiment_info_json()
        self._info_pub.publish(info)
        self.get_logger().info(f"experiment_info: master_seed={master_seed}")

        assay.set_trial_change_callback(self._on_trial_change)

        hz = heartbeat_hz if heartbeat_hz and heartbeat_hz > 0 else 10.0
        self._timer = self.create_timer(1.0 / hz, self._publish_state)

        self.should_exit = False
        self._sketch_was_running = False
        self._complete_since = None
        self._exit_grace_sec = float(self.declare_parameter("exit_grace_sec", 2.0).value)
        self._watchdog = self.create_timer(0.25, self._check_sketch)

        if start_mode == "triggered":
            if trigger_msg_type == "string":
                self._trigger_sub = self.create_subscription(
                    String, trigger_topic, self._on_trigger_string, 10
                )
                kind = "std_msgs/String (any message = start; e.g. mosquito_detector)"
            else:
                self._trigger_sub = self.create_subscription(
                    Bool, trigger_topic, self._on_trigger_bool, 10
                )
                kind = "std_msgs/Bool (true = start, false = abort)"
            self.get_logger().info(
                f"start_mode=triggered: ARMED, waiting for {kind} on "
                f"'{self._trigger_sub.topic_name}'"
            )

        self.get_logger().info("stimulus_publisher up; starting sketch...")

    def _on_trigger_bool(self, msg):
        """std_msgs/Bool: true -> begin the run, false -> abort to ARMED."""
        if msg.data:
            self.get_logger().info("trigger received -> starting run")
            assay.start_run()
        else:
            self.get_logger().info("trigger false -> aborting run")
            assay.abort_run()

    def _on_trigger_string(self, msg):
        """std_msgs/String: any message = a detection/trigger event = start
        the run. If it's JSON (e.g. mosquito_detector's output), log a short
        summary; the raw message is what actually gets recorded in the bag."""
        summary = msg.data
        try:
            event = json.loads(msg.data)
            summary = f"{event.get('event', '?')} at {event.get('position_px', '?')}"
        except (json.JSONDecodeError, TypeError):
            pass
        self.get_logger().info(f"trigger received ({summary}) -> starting run")
        assay.start_run()

    def _check_sketch(self):
        """Flag the main loop to exit when the run is over -- experiment
        finished (after a short grace so trailing messages reach the bag), or
        the user closed the sketch window."""
        if assay.sketch_running():
            self._sketch_was_running = True
        if self.should_exit:
            return
        if assay.experiment_complete():
            now = self.get_clock().now().nanoseconds / 1e9
            if self._complete_since is None:
                self._complete_since = now
                self.get_logger().info(
                    f"experiment complete; shutting down in {self._exit_grace_sec:.1f}s"
                )
            elif now - self._complete_since >= self._exit_grace_sec:
                self.should_exit = True
        elif self._sketch_was_running and not assay.sketch_running():
            self.get_logger().info("sketch window closed; shutting down")
            self.should_exit = True

    def _on_trial_change(self, state):
        """Called from the py5 sketch thread on each new trial (and on
        run-abort / experiment-complete)."""
        if state is None:
            return
        msg = String()
        msg.data = assay.to_json(state)
        self._state_pub.publish(msg)
        if "trial_id" in state:
            self._trial_pub.publish(msg)
            self.get_logger().info(
                f"[run {state['run_id']}] trial {state['trial_id']} "
                f"({state['phase']}): [{state['condition']['name']}] "
                f"LEFT={state['left']['name']} RIGHT={state['right']['name']} "
                f"{state['trial_duration_sec']:.1f}s (trial_seed={state['trial_seed']})"
            )
        else:
            self.get_logger().info(f"phase={state['phase']} (run {state['run_id']})")

    def _publish_state(self):
        payload = assay.current_state_json()
        if payload is None:
            return  # sketch not through its first trial yet
        msg = String()
        msg.data = payload
        self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = StimulusPublisher()
    except ExperimentError as exc:
        print(f"[stimulus_publisher] bad experiment definition: {exc}", file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(2)

    # py5 must own the main thread, so start the sketch non-blocking and let
    # rclpy.spin() have the main loop. Ctrl-C then unwinds spin() and the
    # finally block closes the sketch window.
    assay.run(block=False)

    exit_code = 0
    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # normal Ctrl-C / window-close / experiment-complete shutdown
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"node error: {exc!r}")
        exit_code = 1
    finally:
        assay.set_trial_change_callback(None)
        assay.request_stop()
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.try_shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
