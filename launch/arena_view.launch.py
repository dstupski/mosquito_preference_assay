"""A live camera feed with the trigger zone drawn on it.

    ros2 launch mosquito_preference_assay arena_view.launch.py

Read-only. Open it in a spare terminal beside a running experiment and leave
it there. It shows the only region of the frame where a mosquito can start a
trial; everything outside is dimmed.

It layers the same files the detector does -- detector_params then the saved
trigger zone -- so the box on screen is the box that fires trials. Pointing it
somewhere else would make it a decoration rather than a check.

    camera:=cam0       watch the other camera
    display_hz:=5      render less often on a loaded machine
    roi:=""            override the zone (rarely what you want)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mosquito_preference_assay.config_paths import (
    resolve_config,
    resolve_trigger_config,
)

DEFAULT_CAM0 = "/cam_sync/cam0/image_raw"
DEFAULT_CAM1 = "/cam_sync/cam1/image_raw"


def generate_launch_description():
    def _node(context, *_a, **_k):
        def arg(name):
            return LaunchConfiguration(name).perform(context).strip()

        cam0, cam1 = arg("cam0_topic"), arg("cam1_topic")
        chosen = arg("camera")
        image_topic = {"cam0": cam0, "cam1": cam1}.get(chosen, chosen)
        if chosen and not image_topic:
            raise RuntimeError(
                f"camera={chosen!r} is not cam0, cam1, or a topic name")

        zone = resolve_trigger_config(arg("trigger_config"))

        overrides = {
            "image_qos": arg("image_qos") or "sensor_data",
            "display_hz": float(arg("display_hz") or 15.0),
            "max_display_px": int(arg("max_display_px") or 1100),
        }
        # Applied, not merely defaulted: the zone file carries the DETECTION
        # camera's topic, and layering it would otherwise drag this window
        # onto that camera whatever `camera:=` said.
        if image_topic:
            overrides["image_topic"] = image_topic
        if arg("roi"):
            overrides["roi"] = arg("roi")

        return [
            LogInfo(msg=(
                f"arena view\n"
                f"  camera : {image_topic}\n"
                f"  zone   : {zone or 'none -- detector_params roi'}")),
            Node(package="mosquito_preference_assay", executable="arena_view",
                 name="arena_view", output="screen",
                 parameters=[LaunchConfiguration("params_file"),
                             *([zone] if zone else []),
                             overrides]),
        ]

    return LaunchDescription([
        DeclareLaunchArgument("camera", default_value="cam1"),
        DeclareLaunchArgument("cam0_topic", default_value=DEFAULT_CAM0),
        DeclareLaunchArgument("cam1_topic", default_value=DEFAULT_CAM1),
        DeclareLaunchArgument("image_qos", default_value="sensor_data"),
        DeclareLaunchArgument(
            "params_file", default_value=resolve_config("detector_params.yaml")),
        DeclareLaunchArgument("trigger_config", default_value=""),
        DeclareLaunchArgument("roi", default_value=""),
        DeclareLaunchArgument("display_hz", default_value="15.0"),
        DeclareLaunchArgument("max_display_px", default_value="1100"),
        OpaqueFunction(function=_node),
    ])
