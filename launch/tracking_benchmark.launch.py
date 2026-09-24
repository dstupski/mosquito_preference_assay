"""Bring up the whole stereo-tracking pipeline against recorded footage, with
pipeline_monitor attached -- one command to see, on this machine, what rate
the pipeline holds and how far behind real time it runs.

Run it on a new PC to check the box keeps up before trusting it with live
cameras:

    ros2 launch mosquito_preference_assay tracking_benchmark.launch.py \\
        session:=/path/to/session rate_hz:=200.0 \\
        checkerboard_file:=/path/Checkerboard_<date>.npy \\
        plumbline_file:=/path/Plumbline_<date>.npy

`session` must hold cam_a/ and cam_b/ frame directories (or pass source_a /
source_b directly). Drop the two calibration arguments to benchmark tracking
only -- the triangulator is skipped and stereo pairs are the final stage.

Arguments (defaults in parentheses):
    session                  ""        directory holding cam_a/ and cam_b/
    source_a / source_b      ""        override the two frame sources directly
    rate_hz                  200.0     playback rate to drive the pipeline at
    loop                     False     replay continuously
    params_file   config/tracking_params.yaml (or your .local copy) -- ROIs,
                  topics, detection tuning, sync slop, and the CALIBRATION
                  PATHS. Set them there once instead of passing them every run.
    roi_a / roi_b            "" = leave params_file alone
    image_qos                "" = leave params_file alone. reliable = process
                                  every frame (lag grows under load) |
                                  sensor_data = newest frame, drops when saturated
    max_reprojection_error_px  "" = leave params_file alone
    report_period_sec         "" = leave params_file alone
    watch_images              "" = leave params_file alone
    plot                      False    also open the live 3D trajectory plot
"""

from mosquito_preference_assay.config_paths import resolve_config
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _source(camera):
    """session/<camera> unless source_<letter> was given explicitly."""
    letter = camera[-1]
    explicit = LaunchConfiguration(f"source_{letter}")
    return PythonExpression([
        "'", explicit, "' or ('", LaunchConfiguration("session"), "' and '",
        LaunchConfiguration("session"), "' + '/", camera, "')",
    ])


def _nodes(context, *_args, **_kwargs):
    """Built at launch time so an unset argument leaves the params file
    authoritative. Everything here has a default in config/tracking_params.yaml
    (or your .local copy); the arguments are for one-off overrides."""
    params = LaunchConfiguration("params_file").perform(context)

    def given(name, cast=str):
        raw = LaunchConfiguration(name).perform(context).strip()
        return cast(raw) if raw else None

    def tracker(letter, roi_argument):
        overrides = {}
        roi = given(roi_argument)
        if roi:
            overrides["roi"] = roi
        qos = given("image_qos")
        if qos:
            overrides["image_qos"] = qos
        return Node(
            package="mosquito_preference_assay", executable="tracker",
            name=f"tracker_{letter}", output="screen",
            parameters=[params, overrides],
        )

    calibration = {}
    for name in ("checkerboard_file", "plumbline_file"):
        value = given(name)
        if value:
            calibration[name] = value
    error_px = given("max_reprojection_error_px", float)
    if error_px is not None:
        calibration["max_reprojection_error_px"] = error_px

    monitor = {}
    period = given("report_period_sec", float)
    if period is not None:
        monitor["report_period_sec"] = period
    watch = LaunchConfiguration("watch_images").perform(context).strip()
    if watch:
        monitor["watch_images"] = watch.lower() in ("1", "true", "yes")

    # the triangulator needs a calibration: from the params file, or given here
    import yaml
    from_file = {}
    try:
        with open(params) as handle:
            from_file = (yaml.safe_load(handle) or {}).get(
                "triangulator", {}).get("ros__parameters", {})
    except OSError:
        pass
    have_calibration = bool(
        (calibration.get("checkerboard_file") or from_file.get("checkerboard_file"))
        and (calibration.get("plumbline_file") or from_file.get("plumbline_file")))

    nodes = [
        tracker("a", "roi_a"),
        tracker("b", "roi_b"),
        Node(package="mosquito_preference_assay", executable="stereo_sync",
             name="stereo_sync", output="screen", parameters=[params]),
        Node(package="mosquito_preference_assay", executable="pipeline_monitor",
             name="pipeline_monitor", output="screen", parameters=[params, monitor]),
    ]
    if have_calibration:
        nodes.append(Node(
            package="mosquito_preference_assay", executable="triangulator",
            name="triangulator", output="screen",
            parameters=[params, calibration]))
    else:
        nodes.append(LogInfo(msg=(
            "no calibration (checkerboard_file / plumbline_file unset in "
            f"{params} and not given as arguments) -- running tracking only, "
            "stereo pairs are the final stage")))
    return nodes


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=resolve_config(
            "tracking_params.yaml")),
        DeclareLaunchArgument("session", default_value=""),
        DeclareLaunchArgument("source_a", default_value=""),
        DeclareLaunchArgument("source_b", default_value=""),
        DeclareLaunchArgument("rate_hz", default_value="200.0"),
        DeclareLaunchArgument("loop", default_value="False"),
        DeclareLaunchArgument("roi_a", default_value=""),
        DeclareLaunchArgument("roi_b", default_value=""),
        DeclareLaunchArgument("image_qos", default_value=""),
        DeclareLaunchArgument("checkerboard_file", default_value=""),
        DeclareLaunchArgument("plumbline_file", default_value=""),
        DeclareLaunchArgument("max_reprojection_error_px", default_value=""),
        DeclareLaunchArgument("report_period_sec", default_value=""),
        DeclareLaunchArgument("watch_images", default_value=""),
        DeclareLaunchArgument("plot", default_value="False"),

        OpaqueFunction(function=_nodes),
        Node(
            package="mosquito_preference_assay", executable="trajectory_plotter",
            name="trajectory_plotter", output="screen",
            condition=IfCondition(LaunchConfiguration("plot")),
            parameters=[{"max_points": 800}],
        ),
        # started last: everything above is subscribed before frames flow, so
        # the first frames are not missed while nodes are still coming up
        Node(
            package="mosquito_preference_assay", executable="dual_video_publisher",
            name="dual_video_publisher", output="screen",
            parameters=[{
                "source_a": _source("cam_a"),
                "source_b": _source("cam_b"),
                "rate_hz": LaunchConfiguration("rate_hz"),
                "loop": LaunchConfiguration("loop"),
            }],
        ),
    ])
