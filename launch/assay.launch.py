"""Launch the preference assay + stimulus_state publisher.

Loads config/assay_params.yaml by default:

    ros2 launch mosquito_preference_assay assay.launch.py

Point at your own params file to change anything:

    ros2 launch mosquito_preference_assay assay.launch.py \\
        params_file:=/abs/path/to/my_params.yaml

(or edit the yaml, or use `ros2 run ... --ros-args -p name:=value` for a
one-off override).
"""


from mosquito_preference_assay.config_paths import (
    resolve_config,
    resolve_display_config,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _node(context, *_args, **_kwargs):
    calibration = resolve_display_config(
        LaunchConfiguration("display_config").perform(context))
    note = LogInfo(msg=(f"display calibration: {calibration}" if calibration
                        else "display calibration: none -- using the "
                             "experiment's own geometry"))
    return [note, Node(
        package="mosquito_preference_assay",
        executable="stimulus_publisher",
        name="stimulus_publisher",
        output="screen",
        parameters=[LaunchConfiguration("params_file"),
                    *([calibration] if calibration else [])],
    )]


def generate_launch_description():
    default_params = resolve_config("assay_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("display_config", default_value=""),
        OpaqueFunction(function=_node),
    ])
