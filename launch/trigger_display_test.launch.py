"""Rehearse one whole trial with no camera and no mosquito.

The point is to check, before an animal is anywhere near the rig, that:

  * the circles land where your config says they should, on the right screen;
  * your experiment file draws the trial you expect;
  * the trigger path works end to end;
  * the bag records and closes properly, with the trial inside it.

    ros2 launch mosquito_preference_assay trigger_display_test.launch.py \\
        params_file:=~/rig/arena1_assay_params.yaml \\
        experiment_file:=ten_stimulus_panel \\
        fullscreen:=true monitor:=2

It brings up `stimulus_publisher` ARMED, waits `delay_sec`, then fires the
trigger itself with `test_trigger` -- standing in for the detector, so no
camera is involved. The trial runs for its `duration_sec`, the node exits, and
that exit closes the bag exactly as it would on a real run.

The two config paths are separate on purpose, because they answer different
questions:

    params_file       this RIG -- which screen, where the circles sit,
                      window size. Defaults to config/assay_params.yaml;
                      point it at your rig's copy.
    experiment_file   the SCIENCE -- which stimuli, how the pair is drawn,
                      how long the trial runs, circle diameter and centres.
                      A bare name resolves against the installed experiments/
                      folder, or give a path.

`fullscreen` / `monitor` override the params file for a one-off, so you can
rehearse on the projector without editing anything.

This launch file OWNS the trigger wiring: it forces the node's trigger topic
and type to match `test_trigger`, rather than depending on whatever the
experiment file's `trigger:` block happens to say. A display rehearsal that
silently never fires would be worse than useless.

Afterwards, check the bag really holds the trial:

    ros2 bag info <bag_dir>
    ros2 topic echo --once /stimulus_publisher/trial_start   # while it runs

Arguments
    params_file      config/assay_params.yaml   this rig's display config
    experiment_file  ""        "" = whatever params_file says; else name or path
    fullscreen       ""        "" = leave params_file alone; true/false to override
    monitor          ""        "" = leave params_file alone; "N" / "span"
    delay_sec        4.0       how long ARMED before the pseudo-trigger fires
    repeat_sec       0.0       >0 fires repeatedly, to watch several trials
    bag_dir          ./display_test_<timestamp>
    record           true      false = no recording, just the display
    record_all       true      true -> `ros2 bag record -a` (everything);
                               false -> the assay topics + the trigger only
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
from launch_ros.parameter_descriptions import ParameterValue

# Fixed here so both ends always agree, whatever the experiment file says.
TRIGGER_TOPIC = "/display_test/trigger"

_DEFAULT_BAG = os.path.join(
    os.getcwd(), "display_test_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
)


def generate_launch_description():
    default_params = resolve_config("assay_params.yaml")

    def _sketch(context, *_args, **_kwargs):
        """Overrides are applied only when given, so an unset argument leaves
        the rig's params file authoritative instead of blanking it."""
        calibration = resolve_display_config(
            LaunchConfiguration("display_config").perform(context))
        note = LogInfo(msg=(f"display calibration: {calibration}" if calibration
                            else "display calibration: none -- using the "
                                 "experiment's own geometry"))
        calibration = [calibration] if calibration else []

        overrides = {
            "start_mode": "triggered",
            "trigger_topic": TRIGGER_TOPIC,
            "trigger_msg_type": "bool",
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

        node = Node(
            package="mosquito_preference_assay",
            executable="stimulus_publisher",
            name="stimulus_publisher",
            output="screen",
            parameters=[LaunchConfiguration("params_file"), *calibration, overrides],
        )
        # registered here, where there is a concrete action to target: the
        # sketch exiting is what closes the bag, and it must be that process
        # specifically -- an untargeted handler would also fire if the
        # recorder or the pseudo-trigger stopped first
        return [note, node, RegisterEventHandler(OnProcessExit(
            target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason="display test finished"))],
        ))]

    sketch = OpaqueFunction(function=_sketch)

    # stands in for the detector: fires once, delay_sec after it starts
    pseudo_trigger = Node(
        package="mosquito_preference_assay",
        executable="test_trigger",
        name="pseudo_trigger",
        output="screen",
        parameters=[{
            "topic": TRIGGER_TOPIC,
            "mode": "timer",
            # value_type=float, or `delay_sec:=30` arrives as an INTEGER and
            # the node rejects it -- the trigger then never fires and the
            # rehearsal silently sits ARMED forever
            "delay_sec": ParameterValue(
                LaunchConfiguration("delay_sec"), value_type=float),
            "repeat_sec": ParameterValue(
                LaunchConfiguration("repeat_sec"), value_type=float),
        }],
    )

    # -a by default: the point of a rehearsal is to prove the recording path,
    # and with no camera in this launch there is nothing large to capture, so
    # recording everything is both cheap and a more faithful dry run of
    # triggered_assay (which also defaults to -a).
    record_everything = PythonExpression(
        ["'", LaunchConfiguration("record_all"), "'.lower() == 'true'"])
    recorder_all = ExecuteProcess(
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration("record"), "'.lower() == 'true' and ",
             record_everything])),
        cmd=["ros2", "bag", "record", "-a", "-o", LaunchConfiguration("bag_dir")],
        output="screen",
    )
    recorder_selected = ExecuteProcess(
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration("record"), "'.lower() == 'true' and not ",
             record_everything])),
        cmd=["ros2", "bag", "record", "-o", LaunchConfiguration("bag_dir"),
             "/stimulus_publisher/experiment_info",
             "/stimulus_publisher/stimulus_state",
             "/stimulus_publisher/trial_start",
             TRIGGER_TOPIC],
        output="screen",
    )

    # the recorder needs a moment to finish discovery before anything publishes,
    # and the trigger must not fire before the sketch is armed
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("display_config", default_value=""),
        DeclareLaunchArgument("experiment_file", default_value=""),
        DeclareLaunchArgument("fullscreen", default_value=""),
        DeclareLaunchArgument("monitor", default_value=""),
        DeclareLaunchArgument("delay_sec", default_value="4.0"),
        DeclareLaunchArgument("repeat_sec", default_value="0.0"),
        DeclareLaunchArgument("bag_dir", default_value=_DEFAULT_BAG),
        DeclareLaunchArgument("record", default_value="true"),
        DeclareLaunchArgument("record_all", default_value="true"),

        recorder_all,
        recorder_selected,
        TimerAction(period=2.0, actions=[sketch]),
        # after the sketch, so the trigger cannot arrive before it is armed
        TimerAction(period=4.0, actions=[pseudo_trigger]),
    ])
