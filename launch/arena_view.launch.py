"""A live camera feed with the trigger zone drawn on it.

    ros2 launch mosquito_preference_assay arena_view.launch.py

Read-only. Open it in a spare terminal beside a running experiment and leave
it there. It shows the only region of the frame where a mosquito can start a
trial; everything outside is dimmed.

IT SHOWS WHAT THE DETECTOR SEES. Camera and zone both come from the same files
the detector reads -- detector_params then the saved trigger zone -- so this
is a check on the real configuration rather than a picture of it. Both are
read out of the files directly rather than through ROS parameter delivery,
because a params file keyed by the detector's node name reaches the detector
and not this node.

    camera:=cam0       watch a different camera than the detector's
    display_hz:=5      render less often on a loaded machine
    roi:=""            override the zone (rarely what you want)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mosquito_preference_assay.config_paths import (
    detector_setting,
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
        zone = resolve_trigger_config(arg("trigger_config"))

        chosen = arg("camera")
        if chosen:
            image_topic = {"cam0": cam0, "cam1": cam1}.get(chosen, chosen)
        else:
            # Follow the DETECTOR's camera, so the feed and the zone drawn on
            # it belong to the same device. Showing cam1 with a box measured
            # on cam0 would be worse than showing nothing: it looks correct.
            image_topic = detector_setting(
                "image_topic", arg("params_file"), zone) or cam1

        # Read the zone out of the files rather than relying on ROS to
        # deliver it: this node is not called mosquito_detector, so a params
        # file keyed by that node name would reach the detector and not us --
        # the window would then report no zone while the detector happily had
        # one. Explicit beats a silent mismatch here, since the whole point is
        # to show what the detector is using.
        roi = arg("roi") or detector_setting("roi", arg("params_file"), zone)

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
        if roi:
            overrides["roi"] = str(roi)

        return [
            LogInfo(msg=(
                f"arena view -- the detector's camera and zone\n"
                f"  camera : {image_topic}\n"
                f"  zone   : {roi or 'NONE -- the whole frame can trigger'}\n"
                f"  from   : {zone or arg('params_file')}")),
            Node(package="mosquito_preference_assay", executable="arena_view",
                 name="arena_view", output="screen",
                 parameters=[LaunchConfiguration("params_file"),
                             *([zone] if zone else []),
                             overrides]),
        ]

    return LaunchDescription([
        DeclareLaunchArgument("camera", default_value=""),
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
