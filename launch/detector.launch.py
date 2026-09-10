"""Launch the mosquito_detector node.

Loads config/detector_params.yaml by default:

    ros2 launch mosquito_preference_assay detector.launch.py

Point at your own params file (copy config/detector_params.yaml, edit for
your rig):

    ros2 launch mosquito_preference_assay detector.launch.py \\
        params_file:=/abs/path/to/my_detector.yaml

(or edit the yaml, or `ros2 run ... --ros-args -p name:=value` for a one-off).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("mosquito_preference_assay"),
        "config", "detector_params.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        Node(
            package="mosquito_preference_assay",
            executable="mosquito_detector",
            name="mosquito_detector",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
