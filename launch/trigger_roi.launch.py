"""Draw the detector's trigger zone on the live camera feed.

    ros2 launch mosquito_preference_assay trigger_roi.launch.py

The camera must already be publishing -- this starts no camera. By default it
watches cam0; point it at the other one with `camera:=cam1`.

Drag the box over the part of the arena where the animal should start a trial,
press `s`, and you are done: it writes config/trigger_roi.local.yaml, which
arena_experiment layers over detector_params automatically. The same file
records WHICH camera the zone was drawn on, so choosing the camera here is
what points detection at it -- the box and the camera it belongs to never
travel separately.

The overlay runs the detector's own detection code with the detector's own
parameters, so a blob boxed green here is one that would fire a trial.

    save_dir:=/data/rig   also writes a dated archive there, for the record
    out_file:=none        live file only, no archive

Arguments
    camera           cam0        cam0 | cam1 | an explicit topic name
    cam0_topic       /cam_sync/cam0/image_raw
    cam1_topic       /cam_sync/cam1/image_raw
    image_qos        sensor_data  real camera drivers are best-effort
    params_file      config/detector_params.yaml  (prefers *.local.yaml)
    trigger_config   ""          "" = start from trigger_roi.local.yaml if present
    save_dir         <cwd>       where the dated archive goes
    live_file        ""          "" = config/trigger_roi.local.yaml; none = off
    out_file         ""          "" = <save_dir>/<date>_trigger_roi.yaml
    max_display_px   1100        window width cap; the feed is scaled to fit
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
        chosen = arg("camera") or "cam0"
        image_topic = {"cam0": cam0, "cam1": cam1}.get(chosen, chosen)

        # Start from the zone already saved, so a second session adjusts the
        # existing box instead of silently starting over.
        existing = resolve_trigger_config(arg("trigger_config"))

        overrides = {
            "image_topic": image_topic,
            "image_qos": arg("image_qos") or "sensor_data",
            "save_dir": arg("save_dir"),
            "live_file": arg("live_file"),
            "out_file": arg("out_file"),
            "max_display_px": int(arg("max_display_px") or 1100),
        }

        return [
            LogInfo(msg=(
                f"trigger zone editor\n"
                f"  camera        : {chosen} -> {image_topic}\n"
                f"  starting from : {existing or 'a fresh centred box'}\n"
                f"  detector knobs: {arg('params_file')}")),
            Node(package="mosquito_preference_assay", executable="trigger_roi",
                 name="trigger_roi", output="screen",
                 # detector_params first so the overlay uses the same
                 # thresholds the real detector will, then the saved zone,
                 # then this launch's explicit choices.
                 parameters=[LaunchConfiguration("params_file"),
                             *([existing] if existing else []),
                             overrides]),
        ]

    return LaunchDescription([
        DeclareLaunchArgument("camera", default_value="cam0"),
        DeclareLaunchArgument("cam0_topic", default_value=DEFAULT_CAM0),
        DeclareLaunchArgument("cam1_topic", default_value=DEFAULT_CAM1),
        DeclareLaunchArgument("image_qos", default_value="sensor_data"),
        DeclareLaunchArgument("params_file",
                              default_value=resolve_config("detector_params.yaml")),
        DeclareLaunchArgument("trigger_config", default_value=""),
        DeclareLaunchArgument("save_dir", default_value=""),
        DeclareLaunchArgument("live_file", default_value=""),
        DeclareLaunchArgument("out_file", default_value=""),
        DeclareLaunchArgument("max_display_px", default_value="1100"),
        OpaqueFunction(function=_node),
    ])
