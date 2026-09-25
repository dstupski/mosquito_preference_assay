"""The real run: two synced cameras, one animal, one trial, one run folder.

    ros2 launch mosquito_preference_assay arena_experiment.launch.py \\
        experiment_file:=sippell_retest_experiment \\
        save_dir:=/data/mosquito/2026-09-25

The cameras come from your own stereo package and are expected to be ALREADY
PUBLISHING before this is launched -- nothing here starts them. Check with
`ros2 topic hz /cam_sync/cam0/image_raw` first; a launch against silent
cameras comes up armed and simply never triggers.

    launch ──> recorder up, writing every topic EXCEPT the camera feeds
          ├──> stimulus_publisher opens on the projector and sits ARMED
          └──> mosquito_detector watches the detection camera's trigger zone
                        │
       mosquito detected ──> stimuli appear (same message is the trigger)
                        ├──> trial_recorder starts a SECOND bag for the video
            duration ───> trial ends ──> sketch exits ──> Shutdown ──>
                              both bags SIGINTed and finalized

Two bags, one run folder:

    <save_dir>/<run_name>_<timestamp>/
        assay/   everything but the video -- up from launch, no gap
        video/   the camera feeds -- starts at the trigger

Video is recorded separately and only after the trigger because it cannot be
treated like the other topics: two 1440x1080 feeds at 200 fps is ~620 MB/s, so
recording from launch costs ~2.2 TB per hour of waiting, and buffering 15 s of
it in RAM would need ~9.3 GB. The cost of starting at the trigger is a
measured 0.16 s before the first frame lands (rosbag2 subscribing to an
already-live topic). The assay topics have no such gap.

WHICH CAMERA DETECTS: normally you do not set this here. `trigger_roi` writes
both the zone and the camera it was drawn on into trigger_roi.local.yaml, and
this launch layers that file over detector_params -- so drawing the zone on
cam0 is what points detection at cam0. `detection_camera:=cam1` overrides it
for one run.

Arguments
    save_dir         <cwd>      parent directory for the run folder
    run_name         trial      run folder prefix; a timestamp is appended
    experiment_file  sippell_retest_experiment
    cam0_topic       /cam_sync/cam0/image_raw
    cam1_topic       /cam_sync/cam1/image_raw
    detection_camera ""         "" = whatever trigger_roi.local.yaml says;
                                otherwise cam0 | cam1 | an explicit topic
    trigger_config   ""         "" = config/trigger_roi.local.yaml if present
    params_file      config/assay_params.yaml   (prefers *.local.yaml)
    display_config   ""         "" = config/display_geometry.local.yaml
    detector_params  config/detector_params.yaml
    monitor          ""         unset = leave params_file alone
    fullscreen       ""         unset = leave params_file alone
    master_seed      ""         unset = leave params_file alone
    detector_delay   4.0        seconds before the detector starts watching.
                                A backstop only: the detector additionally
                                holds fire until the sketch reports ARMED,
                                so no detection is wasted if the JVM is slow.
    record_video     true       false = assay topics only
    record           true       false = no bags at all (dry run)
    trigger_topic    /arena/mosquito_present
"""

import datetime
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from mosquito_preference_assay.config_paths import (
    resolve_config,
    resolve_display_config,
    resolve_trigger_config,
)

# The detector's own message IS the trigger, so whatever fired the trial is
# recorded as data rather than inferred. This file owns that wiring: it forces
# both ends onto one topic rather than hoping two config files agree.
DEFAULT_TRIGGER_TOPIC = "/arena/mosquito_present"
DEFAULT_CAM0 = "/cam_sync/cam0/image_raw"
DEFAULT_CAM1 = "/cam_sync/cam1/image_raw"


def _arg(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def generate_launch_description():
    args = [
        DeclareLaunchArgument("save_dir", default_value=os.getcwd()),
        DeclareLaunchArgument("run_name", default_value="trial"),
        DeclareLaunchArgument("experiment_file",
                              default_value="sippell_retest_experiment"),
        DeclareLaunchArgument("cam0_topic", default_value=DEFAULT_CAM0),
        DeclareLaunchArgument("cam1_topic", default_value=DEFAULT_CAM1),
        DeclareLaunchArgument("detection_camera", default_value=""),
        DeclareLaunchArgument("trigger_config", default_value=""),
        DeclareLaunchArgument("params_file",
                              default_value=resolve_config("assay_params.yaml")),
        DeclareLaunchArgument("display_config", default_value=""),
        DeclareLaunchArgument("detector_params",
                              default_value=resolve_config("detector_params.yaml")),
        DeclareLaunchArgument("monitor", default_value=""),
        DeclareLaunchArgument("fullscreen", default_value=""),
        DeclareLaunchArgument("master_seed", default_value=""),
        DeclareLaunchArgument("detector_delay", default_value="4.0"),
        DeclareLaunchArgument("record_video", default_value="true"),
        DeclareLaunchArgument("record", default_value="true"),
        DeclareLaunchArgument("trigger_topic", default_value=DEFAULT_TRIGGER_TOPIC),
    ]

    recording = PythonExpression(
        ["'", LaunchConfiguration("record"), "'.lower() == 'true'"])

    def _setup(context, *_a, **_k):
        """Everything is built here so the run folder carries the launch's own
        timestamp, and so an unset argument leaves the config file
        authoritative rather than overwriting it with a launch default."""
        trigger_topic = _arg(context, "trigger_topic") or DEFAULT_TRIGGER_TOPIC
        cam0, cam1 = _arg(context, "cam0_topic"), _arg(context, "cam1_topic")
        video_topics = [t for t in (cam0, cam1) if t]

        # --- where everything is written ---------------------------------- #
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = os.path.abspath(os.path.expanduser(os.path.join(
            _arg(context, "save_dir") or os.getcwd(),
            f"{_arg(context, 'run_name') or 'trial'}_{stamp}")))
        assay_bag, video_bag = os.path.join(run_dir, "assay"), \
            os.path.join(run_dir, "video")

        # --- the rig's calibrations --------------------------------------- #
        display_cal = resolve_display_config(_arg(context, "display_config"))
        trigger_cal = resolve_trigger_config(_arg(context, "trigger_config"))

        # --- which camera detects ----------------------------------------- #
        # Precedence: explicit argument > trigger_roi file > detector_params.
        # Only set as an override when actually given, so the zone file keeps
        # the camera it was drawn on.
        chosen = _arg(context, "detection_camera")
        detection_topic = {"cam0": cam0, "cam1": cam1}.get(chosen, chosen)
        if chosen and not detection_topic:
            raise RuntimeError(
                f"detection_camera={chosen!r} is not cam0, cam1, or a topic name")

        # Hold fire until the sketch reports ARMED. This is what makes the
        # animal's FIRST approach count: detector_delay alone is a guess at
        # JVM startup time, and when the guess is short that detection is
        # refused and the trial runs on a later, unrelated approach.
        detector_overrides = {
            "topic": trigger_topic,
            "arm_topic": "/stimulus_publisher/stimulus_state",
        }
        if detection_topic:
            detector_overrides["image_topic"] = detection_topic

        # Say out loud what was resolved. Every one of these was a silent
        # failure at some point: a stale config, a zone drawn on the other
        # camera, a bag written somewhere nobody looked.
        notes = [LogInfo(msg=(
            f"run folder      : {run_dir}\n"
            f"  assay bag     : {assay_bag}"
            f"{'' if _arg(context, 'record').lower() == 'true' else '  (disabled)'}\n"
            f"  video bag     : {video_bag}  (starts at the trigger)\n"
            f"display calib   : {display_cal or 'none -- experiment geometry'}\n"
            f"trigger zone    : {trigger_cal or 'none -- detector_params roi'}\n"
            f"detection camera: "
            f"{detection_topic or (trigger_cal and 'from trigger zone file') or cam0}\n"
            f"video topics    : {', '.join(video_topics) or 'none'}"))]

        # --- the sketch: ARMED at launch, stimuli only on detection -------- #
        sketch_overrides = {
            "start_mode": "triggered",
            "trigger_topic": trigger_topic,
            "trigger_msg_type": "string",
        }
        for name, key, cast in (("experiment_file", "experiment_file", str),
                                ("monitor", "monitor", str),
                                ("master_seed", "master_seed", int)):
            value = _arg(context, name)
            if value:
                sketch_overrides[key] = cast(value)
        fullscreen = _arg(context, "fullscreen")
        if fullscreen:
            sketch_overrides["fullscreen"] = fullscreen.lower() in ("1", "true", "yes")

        sketch = Node(
            package="mosquito_preference_assay", executable="stimulus_publisher",
            name="stimulus_publisher", output="screen",
            parameters=[LaunchConfiguration("params_file"),
                        *([display_cal] if display_cal else []),
                        sketch_overrides])

        # --- the detector: started late, on purpose ------------------------ #
        # A detection landing before the sketch is armed cannot run a trial and
        # that animal is lost. The node refuses such a trigger loudly; this
        # delay is what stops it arising.
        detector = TimerAction(
            period=LaunchConfiguration("detector_delay"),
            actions=[Node(
                package="mosquito_preference_assay", executable="mosquito_detector",
                name="mosquito_detector", output="screen",
                parameters=[LaunchConfiguration("detector_params"),
                            *([trigger_cal] if trigger_cal else []),
                            detector_overrides])])

        # --- recording ----------------------------------------------------- #
        # Everything except the video, from launch: cheap while armed, and no
        # discovery race at the trigger.
        exclude = "|".join(t for t in video_topics) or "^$"
        assay_recorder = ExecuteProcess(
            condition=IfCondition(recording),
            cmd=["ros2", "bag", "record", "-o", assay_bag, "-a", "-x", exclude],
            output="screen")

        # The video: spawned by this node when the trigger fires.
        video_recorder = Node(
            package="mosquito_preference_assay", executable="trial_recorder",
            name="trial_recorder", output="screen",
            condition=IfCondition(PythonExpression(
                [recording, " and '", LaunchConfiguration("record_video"),
                 "'.lower() == 'true'"])),
            parameters=[{"trigger_topic": trigger_topic,
                         "trigger_msg_type": "string",
                         "bag_dir": video_bag,
                         "video_topics": video_topics}])

        # The sketch exiting is what ends the run: it closes both bags. Target
        # that process specifically -- an untargeted handler would also fire if
        # a recorder stopped first.
        finish = RegisterEventHandler(OnProcessExit(
            target_action=sketch,
            on_exit=[EmitEvent(event=Shutdown(reason="trial finished"))]))

        # Give the recorder a moment to finish discovery before the sketch
        # publishes its latched experiment_info.
        return notes + [assay_recorder, video_recorder,
                        TimerAction(period=2.0, actions=[sketch, finish]),
                        detector]

    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
