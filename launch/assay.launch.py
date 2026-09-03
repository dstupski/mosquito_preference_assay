"""Launch the preference assay + stimulus_state publisher.

Loads config/assay_params.yaml by default:

    ros2 launch mosquito_preference_assay assay.launch.py

Point at your own params file to change anything:

    ros2 launch mosquito_preference_assay assay.launch.py \\
        params_file:=/abs/path/to/my_params.yaml

(or edit the yaml, or use `ros2 run ... --ros-args -p name:=value` for a
one-off override).
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
        "config", "assay_params.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        Node(
            package="mosquito_preference_assay",
            executable="stimulus_publisher",
            name="stimulus_publisher",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
