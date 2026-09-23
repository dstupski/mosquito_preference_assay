"""One animal, end to end: the display comes up ARMED on the projector with
the detector watching, and the stimuli appear only when a mosquito is found.

    ros2 launch mosquito_preference_assay triggered_assay.launch.py \\
        fullscreen:=true monitor:=2

Order of events:

    launch ──> recorder up (discovered, so nothing is missed later)
          ├──> stimulus_publisher opens on the projector and sits ARMED,
          │      drawing only the background -- the JVM and window cost is
          │      paid HERE, before the animal is introduced
          └──> mosquito_detector starts (after a delay, so it cannot fire at
                 a sketch that is not armed yet) and watches the camera
                        │
       mosquito detected ──> detection_event doubles as the trigger; the two
                        │      stimuli are built and drawn from the next frame
            duration ───> trial completes ──> node exits ──> Shutdown ──>
                              recorder SIGINTed ──> bag finalized

Because the node still exits when its trial ends, bagging is unchanged from
triggered_capture.launch.py: `OnProcessExit -> Shutdown` still SIGINTs the
recorder, so `metadata.yaml` is written and the bag closes cleanly.

Which display: `monitor:=N` picks one, 1-based. Run
`python3 tools/list_displays.py --identify` to see which N is the projector.

Recording (`record_mode`):
    continuous  (default) write from launch. Simple, no discovery race, and
                the only mode that can hold raw camera video, which is far too
                large to buffer in RAM (15 s of two 1440x1080 feeds is 9.3 GB
                at 200 fps, 1.4 GB at 30 fps).
    snapshot    the recorder buffers in memory and writes only when the
                trigger fires, so the waiting period costs nothing on disk.
                Bounded by `--max-cache-size` (100 MiB default), so use it for
                the assay/tracking topics, NOT for raw camera feeds.
    none        no recorder; just the display and the detector.

Arguments
    params_file       config/assay_params.yaml   THIS RIG: screen, circle
                      centres. Point it at your own copy (see the README,
                      "Deploying to another rig").
    experiment_file   single_trigger   experiment name or path (15 s default);
                      pass "" to use whatever params_file says
    fullscreen        ""               unset = leave params_file alone
    monitor           ""               unset = leave params_file alone
    detector_params   config/detector_params.yaml
    image_topic       ""               override the detector's camera topic
    trigger_topic     /arena/mosquito_present   detector output = the trigger
    detector_delay    4.0              seconds to wait before starting the detector
    record_mode       continuous       continuous | snapshot | none
    record_all        true             true -> `-a`; false -> assay + detection only
    bag_dir           <cwd>/mpa_<timestamp>
    max_cache_size    100000000        snapshot-mode buffer, bytes
    master_seed       ""               unset = leave params_file alone
"""

import datetime
import os

from mosquito_preference_assay.config_paths import (
    resolve_config,
    resolve_display_config,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

_DEFAULT_BAG = os.path.join(
    os.getcwd(), "mpa_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
)
# The detector's own message IS the trigger, so whatever fired the trial is
# recorded as data rather than inferred. This launch file owns that wiring: it
# forces both ends onto the same topic rather than relying on detector_params
# and the experiment file happening to agree.
DEFAULT_TRIGGER_TOPIC = "/arena/mosquito_present"


def generate_launch_description():
    default_detector_params = resolve_config("detector_params.yaml")
    default_assay_params = resolve_config("assay_params.yaml")

    args = [
        DeclareLaunchArgument("params_file", default_value=default_assay_params),
        DeclareLaunchArgument("display_config", default_value=""),
        DeclareLaunchArgument("experiment_file", default_value="single_trigger"),
        DeclareLaunchArgument("fullscreen", default_value=""),
        DeclareLaunchArgument("monitor", default_value=""),
        DeclareLaunchArgument("detector_params", default_value=default_detector_params),
        DeclareLaunchArgument("image_topic", default_value=""),
        DeclareLaunchArgument("trigger_topic", default_value=DEFAULT_TRIGGER_TOPIC),
        DeclareLaunchArgument("detector_delay", default_value="4.0"),
        DeclareLaunchArgument("record_mode", default_value="continuous"),
        DeclareLaunchArgument("record_all", default_value="true"),
        DeclareLaunchArgument("bag_dir", default_value=_DEFAULT_BAG),
        DeclareLaunchArgument("max_cache_size", default_value="100000000"),
        DeclareLaunchArgument("master_seed", default_value=""),
    ]

    bag_dir = LaunchConfiguration("bag_dir")
    recording = PythonExpression(
        ["'", LaunchConfiguration("record_mode"), "' != 'none'"])
    record_everything = PythonExpression(
        ["'", LaunchConfiguration("record_all"), "'.lower() == 'true'"])

    def _sketch(context, *_args, **_kwargs):
        """params_file carries the RIG -- which screen, and the circle centres
        display_check wrote. Overrides are applied only when actually given,
        so an unset argument leaves that file authoritative instead of
        silently replacing its value with a launch-file default."""
        calibration = resolve_display_config(
            LaunchConfiguration("display_config").perform(context))
        calibration = [calibration] if calibration else []

        overrides = {
            # forced: this launch file owns the trigger wiring
            "start_mode": "triggered",
            "trigger_topic": LaunchConfiguration("trigger_topic").perform(context),
            "trigger_msg_type": "string",
        }
        experiment = LaunchConfiguration("experiment_file").perform(context).strip()
        if experiment:
            overrides["experiment_file"] = experiment
        fullscreen = LaunchConfiguration("fullscreen").perform(context).strip()
        if fullscreen:
            overrides["fullscreen"] = fullscreen.lower() in ("1", "true", "yes")
        monitor = LaunchConfiguration("monitor").perform(context).strip()
        if monitor:
            overrides["monitor"] = monitor
        seed = LaunchConfiguration("master_seed").perform(context).strip()
        if seed:
            overrides["master_seed"] = int(seed)

        node = Node(
            package="mosquito_preference_assay",
            executable="stimulus_publisher",
            name="stimulus_publisher",
            output="screen",
            parameters=[LaunchConfiguration("params_file"), *calibration, overrides],
        )
        return [node, RegisterEventHandler(OnProcessExit(
            target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason="assay trial finished"))],
        ))]

    # The sketch: opens ARMED and draws only the background until triggered.
    sketch = OpaqueFunction(function=_sketch)

    def _detector(context, *_args, **_kwargs):
        """Built at launch time so an empty image_topic leaves the params file
        authoritative. Passing it unconditionally would override the file with
        an empty string, which rclpy rejects as an invalid topic name."""
        # `topic` is forced, not merely defaulted: the sketch is listening on
        # exactly this, so the detector must publish there whatever its params
        # file says.
        overrides = {"topic": LaunchConfiguration("trigger_topic").perform(context)}
        image_topic = LaunchConfiguration("image_topic").perform(context).strip()
        if image_topic:
            overrides["image_topic"] = image_topic
        parameters = [LaunchConfiguration("detector_params"), overrides]
        return [Node(
            package="mosquito_preference_assay",
            executable="mosquito_detector",
            name="mosquito_detector",
            output="screen",
            parameters=parameters,
        )]

    # Started late ON PURPOSE: a detection that lands before the sketch is
    # armed cannot run a trial, and that animal would be lost. The node now
    # refuses such a trigger loudly rather than stranding state, but the delay
    # is what stops it happening in the first place.
    detector = TimerAction(
        period=LaunchConfiguration("detector_delay"),
        actions=[OpaqueFunction(function=_detector)],
    )

    common = ["ros2", "bag", "record", "-o", bag_dir]
    selected = ["/stimulus_publisher/experiment_info",
                "/stimulus_publisher/stimulus_state",
                "/stimulus_publisher/trial_start",
                LaunchConfiguration("trigger_topic")]

    recorders = [
        ExecuteProcess(
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration("record_mode"), "' == 'continuous' and ",
                 record_everything])),
            cmd=common + ["-a"], output="screen"),
        ExecuteProcess(
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration("record_mode"), "' == 'continuous' and not ",
                 record_everything])),
            cmd=common + selected, output="screen"),
        # snapshot mode buffers in RAM and writes on /rosbag2_recorder/snapshot,
        # so the armed wait costs nothing on disk. Topic list only -- raw camera
        # feeds do not fit in a RAM buffer.
        ExecuteProcess(
            condition=LaunchConfigurationEquals("record_mode", "snapshot"),
            cmd=common + ["--snapshot-mode", "--max-cache-size",
                          LaunchConfiguration("max_cache_size")] + selected,
            output="screen"),
    ]

    snapshot_supervisor = Node(
        package="mosquito_preference_assay",
        executable="snapshot_supervisor",
        name="snapshot_supervisor",
        output="screen",
        condition=LaunchConfigurationEquals("record_mode", "snapshot"),
    )

    # Give the recorder time to finish discovery before anything publishes.
    delayed_sketch = TimerAction(
        period=PythonExpression(["2.0 if ", recording, " else 0.0"]),
        actions=[sketch],
    )

    return LaunchDescription(
        args + recorders + [snapshot_supervisor, delayed_sketch, detector]
    )
