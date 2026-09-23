"""Check the stimulus display, and align it to the arena.

Loads the same `config/assay_params.yaml` the assay itself loads, so the
display it lights up is the display an experiment would use:

    ros2 launch mosquito_preference_assay display_check.launch.py

Point it at your own params file (the one the rig actually runs with):

    ros2 launch mosquito_preference_assay display_check.launch.py \\
        params_file:=/abs/path/to/my_assay_params.yaml

Or override for a one-off, without editing anything:

    ros2 launch mosquito_preference_assay display_check.launch.py \\
        fullscreen:=true monitor:=2

A test pattern appears showing the screen it opened on, corner brackets (a
clipped one means the projector is overscanning), and the two stimulus circles
where the experiment's geometry puts them. Drag the circles to line them up
with the arena and press `s` to write the positions to `out_file`, ready to
paste into `assay_params.yaml`. See display_check_node.py for the full key map.

Arguments
    params_file    config/assay_params.yaml
    fullscreen     ""   set to override the params file
    monitor        ""   set to override the params file
    duration_sec   0.0  0 = stay up until closed
    out_file       ""   "" -> ./<YYYYMMDD>_display_config.yaml; or a dir, or a path
"""


from mosquito_preference_assay.config_paths import (
    resolve_config,
    resolve_display_config,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = resolve_config("assay_params.yaml")

    def _node(context, *_args, **_kwargs):
        """Built at launch time so an unset override leaves the params file
        authoritative, rather than overwriting it with an empty value."""
        calibration = resolve_display_config(
            LaunchConfiguration("display_config").perform(context))
        note = LogInfo(msg=(f"display calibration: {calibration}" if calibration
                            else "display calibration: none -- using the "
                                 "experiment's own geometry"))
        calibration = [calibration] if calibration else []

        overrides = {
            "duration_sec": float(
                LaunchConfiguration("duration_sec").perform(context) or 0.0),
            "out_file": LaunchConfiguration("out_file").perform(context),
        }
        fullscreen = LaunchConfiguration("fullscreen").perform(context).strip()
        if fullscreen:
            overrides["fullscreen"] = fullscreen.lower() in ("1", "true", "yes")
        monitor = LaunchConfiguration("monitor").perform(context).strip()
        if monitor:
            overrides["monitor"] = monitor

        return [note, Node(
            package="mosquito_preference_assay",
            executable="display_check",
            name="display_check",
            output="screen",
            parameters=[LaunchConfiguration("params_file"), *calibration, overrides],
        )]

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("display_config", default_value=""),
        DeclareLaunchArgument("fullscreen", default_value=""),
        DeclareLaunchArgument("monitor", default_value=""),
        DeclareLaunchArgument("duration_sec", default_value="0.0"),
        DeclareLaunchArgument("out_file", default_value=""),
        OpaqueFunction(function=_node),
    ])
