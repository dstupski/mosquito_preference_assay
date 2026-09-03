#!/usr/bin/env python3
"""ROS 2 node: run the py5 preference assay and publish, at all times, a
complete JSON description of which stimulus is on the LEFT and which is on the
RIGHT.

Topics (names are relative to the node, i.e. /stimulus_publisher/...):

    stimulus_state   std_msgs/String   full state as a JSON object; published
                     on every trial change AND at ``heartbeat_hz``. QoS is
                     transient_local + keep_last(1), so a subscriber (or
                     ``ros2 bag record``) that starts mid-trial still gets the
                     current state immediately.

    trial_start      std_msgs/String   the same JSON object, published exactly
                     once per new trial -- a convenient downstream trigger.

The JSON schema is "mosquito_preference_assay/stimulus_state/2"; see
assay.current_state(). It carries the experiment identity + condition, and
every resolved parameter and seed, so a whole session is reconstructable from
a bag alone.

What is shown -- the stimulus pool, the pairings, the ordering and the timing
-- comes from an experiment YAML (see the experiments/ folder), selected with
the `experiment_file` parameter. The rest of the parameters are operational:

    experiment_file     string  ""    experiment name / path; "" -> built-in default
    start_mode          string  auto  auto -> play immediately; triggered -> wait for a trigger
    trigger_topic       string ~/trigger   std_msgs/Bool: true = start run, false = abort
    master_seed         int     -1    -1 -> random seed (logged, in every message)
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
        -p experiment_file:=grating_speed_sweep -p fullscreen:=true
    ros2 bag record /stimulus_publisher/stimulus_state /stimulus_publisher/trial_start
"""

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
        start_mode = str(self.declare_parameter("start_mode", "auto").value).strip()
        trigger_topic = str(self.declare_parameter("trigger_topic", "~/trigger").value)

        path = _resolve_experiment_file(experiment_file)
        experiment = Experiment.from_file(path) if path else Experiment.default()
        if experiment.mode == "sample":
            how = f"mode=sample, pool={experiment.pool}"
        else:
            how = f"mode=pairs, {len(experiment.conditions)} conditions"
        self.get_logger().info(
            f"experiment {experiment.name!r} from {experiment.source} "
            f"(sha1 {experiment.sha1}, {how})"
        )

        if start_mode not in ("auto", "triggered"):
            self.get_logger().warn(f"start_mode {start_mode!r} invalid; using 'auto'")
            start_mode = "auto"

        assay.configure(
            experiment=experiment,
            master_seed=None if master_seed is None or master_seed < 0 else int(master_seed),
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
        self._state_pub = self.create_publisher(String, "~/stimulus_state", latched)
        self._trial_pub = self.create_publisher(String, "~/trial_start", latched)

        assay.set_trial_change_callback(self._on_trial_change)

        hz = heartbeat_hz if heartbeat_hz and heartbeat_hz > 0 else 10.0
        self._timer = self.create_timer(1.0 / hz, self._publish_state)

        self.should_exit = False
        self._sketch_was_running = False
        self._complete_since = None
        self._exit_grace_sec = float(self.declare_parameter("exit_grace_sec", 2.0).value)
        self._watchdog = self.create_timer(0.25, self._check_sketch)

        if start_mode == "triggered":
            self._trigger_sub = self.create_subscription(
                Bool, trigger_topic, self._on_trigger, 10
            )
            self.get_logger().info(
                f"start_mode=triggered: ARMED, waiting for Bool(true) on "
                f"'{self._trigger_sub.topic_name}'"
            )

        self.get_logger().info("stimulus_publisher up; starting sketch...")

    def _on_trigger(self, msg):
        """std_msgs/Bool on the trigger topic: true -> begin the run,
        false -> abort back to ARMED. Swap the msg type here if your trigger
        source uses something else."""
        if msg.data:
            self.get_logger().info("trigger received -> starting run")
            assay.start_run()
        else:
            self.get_logger().info("trigger false -> aborting run")
            assay.abort_run()

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
                f"[run {state['run_id']}] trial {state['trial_id']}: "
                f"[{state['condition_name']}] LEFT={state['left_name']} "
                f"RIGHT={state['right_name']} {state['trial_duration_sec']:.1f}s "
                f"(phase={state['phase']}, trial_seed={state['trial_seed']})"
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
