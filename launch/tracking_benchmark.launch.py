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
    roi_a / roi_b            the tuned arena ROIs
    image_qos                reliable  reliable = process every frame (lag grows
                                       under load) | sensor_data = always take the
                                       newest frame (drops frames, stays fresh)
    max_reprojection_error_px  0.0     drop triangulations above this (0 = keep all)
    report_period_sec         5.0      monitor reporting interval
    watch_images              False    also measure image-topic rates (costs
                                       full-frame bandwidth -- can skew results)
    plot                      False    also open the live 3D trajectory plot
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

DEFAULT_ROI_A = "340,40,1260,1070"
DEFAULT_ROI_B = "350,20,1370,1070"


def _source(camera):
    """session/<camera> unless source_<letter> was given explicitly."""
    letter = camera[-1]
    explicit = LaunchConfiguration(f"source_{letter}")
    return PythonExpression([
        "'", explicit, "' or ('", LaunchConfiguration("session"), "' and '",
        LaunchConfiguration("session"), "' + '/", camera, "')",
    ])


def generate_launch_description():
    have_calibration = PythonExpression([
        "'", LaunchConfiguration("checkerboard_file"), "' != '' and '",
        LaunchConfiguration("plumbline_file"), "' != ''",
    ])

    def tracker(letter, roi_argument):
        return Node(
            package="mosquito_preference_assay", executable="tracker",
            name=f"tracker_{letter}", output="screen",
            parameters=[{
                "image_topic": f"/cam_{letter}/image_raw",
                "topic": f"/tracking/cam_{letter}/position",
                "frame_id": f"cam_{letter}",
                "roi": LaunchConfiguration(roi_argument),
                "image_qos": LaunchConfiguration("image_qos"),
                "log_every_n": 0,
            }],
        )

    return LaunchDescription([
        DeclareLaunchArgument("session", default_value=""),
        DeclareLaunchArgument("source_a", default_value=""),
        DeclareLaunchArgument("source_b", default_value=""),
        DeclareLaunchArgument("rate_hz", default_value="200.0"),
        DeclareLaunchArgument("loop", default_value="False"),
        DeclareLaunchArgument("roi_a", default_value=DEFAULT_ROI_A),
        DeclareLaunchArgument("roi_b", default_value=DEFAULT_ROI_B),
        DeclareLaunchArgument("image_qos", default_value="reliable"),
        DeclareLaunchArgument("checkerboard_file", default_value=""),
        DeclareLaunchArgument("plumbline_file", default_value=""),
        DeclareLaunchArgument("max_reprojection_error_px", default_value="0.0"),
        DeclareLaunchArgument("report_period_sec", default_value="5.0"),
        DeclareLaunchArgument("watch_images", default_value="False"),
        DeclareLaunchArgument("plot", default_value="False"),

        tracker("a", "roi_a"),
        tracker("b", "roi_b"),

        Node(
            package="mosquito_preference_assay", executable="stereo_sync",
            name="stereo_sync", output="screen",
            parameters=[{"log_every_n": 0}],
        ),
        Node(
            package="mosquito_preference_assay", executable="triangulator",
            name="triangulator", output="screen",
            condition=IfCondition(have_calibration),
            parameters=[{
                "checkerboard_file": LaunchConfiguration("checkerboard_file"),
                "plumbline_file": LaunchConfiguration("plumbline_file"),
                "max_reprojection_error_px": LaunchConfiguration(
                    "max_reprojection_error_px"),
                "log_every_n": 0,
            }],
        ),
        Node(
            package="mosquito_preference_assay", executable="pipeline_monitor",
            name="pipeline_monitor", output="screen",
            parameters=[{
                "report_period_sec": LaunchConfiguration("report_period_sec"),
                "watch_images": LaunchConfiguration("watch_images"),
            }],
        ),
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
