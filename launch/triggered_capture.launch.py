"""Triggered single-run capture.

Brings up `stimulus_publisher` in *triggered* mode next to `ros2 bag record`.
The node opens ARMED (blank screen). When a `std_msgs/Bool` with `data: true`
arrives on the trigger topic it plays one trial (15 s with the default
`single_trigger` experiment), publishes a final `phase: "complete"`
message, then exits. The node exiting emits a launch Shutdown, which SIGINTs
`ros2 bag record` so the bag is finalised (`metadata.yaml` written) and closed.

    ros2 launch mosquito_preference_assay triggered_capture.launch.py

    # fire the trigger from anywhere:
    ros2 topic pub --once /stimulus_publisher/trigger std_msgs/msg/Bool "{data: true}"

Arguments
    experiment_file    single_trigger   experiment name or path
    trigger_topic      ""    override the experiment's `trigger:` topic
    trigger_msg_type   ""    override the experiment's `trigger.msg_type` (bool | string)
    bag_dir            <cwd>/mpa_<timestamp>   output dir (must not already exist)
    record_all       true    true -> `ros2 bag record -a`; false -> assay topics + trigger only
    fullscreen       false
    monitor          ""      "" primary | "2" that display | "span"
    master_seed      -1      -1 -> random (recorded in experiment_info)
"""

import datetime
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_DEFAULT_BAG = os.path.join(
    os.getcwd(), "mpa_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
)


def generate_launch_description():
    args = [
        DeclareLaunchArgument("experiment_file", default_value="single_trigger"),
        # "" -> use the experiment file's `trigger:` topic / msg_type
        DeclareLaunchArgument("trigger_topic", default_value=""),
        DeclareLaunchArgument("trigger_msg_type", default_value=""),
        DeclareLaunchArgument("bag_dir", default_value=_DEFAULT_BAG),
        DeclareLaunchArgument("record_all", default_value="true"),
        DeclareLaunchArgument("fullscreen", default_value="false"),
        DeclareLaunchArgument("monitor", default_value=""),
        DeclareLaunchArgument("master_seed", default_value="-1"),
    ]

    trigger_topic = LaunchConfiguration("trigger_topic")
    bag_dir = LaunchConfiguration("bag_dir")

    node = Node(
        package="mosquito_preference_assay",
        executable="stimulus_publisher",
        name="stimulus_publisher",
        output="screen",
        parameters=[{
            "experiment_file": ParameterValue(
                LaunchConfiguration("experiment_file"), value_type=str),
            "start_mode": "triggered",
            "trigger_topic": ParameterValue(trigger_topic, value_type=str),
            "trigger_msg_type": ParameterValue(
                LaunchConfiguration("trigger_msg_type"), value_type=str),
            "fullscreen": ParameterValue(
                LaunchConfiguration("fullscreen"), value_type=bool),
            "monitor": ParameterValue(LaunchConfiguration("monitor"), value_type=str),
            "master_seed": ParameterValue(
                LaunchConfiguration("master_seed"), value_type=int),
        }],
    )

    bag_all = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("record_all")),
        cmd=["ros2", "bag", "record", "-a", "-o", bag_dir],
        output="screen",
    )
    # record_all:=false -> just the assay topics. The trigger topic comes from
    # the experiment file (unknown to launch here); pass record_all:=true (the
    # default, -a) to capture it and the camera / tracking nodes too.
    bag_selected = ExecuteProcess(
        condition=UnlessCondition(LaunchConfiguration("record_all")),
        cmd=["ros2", "bag", "record", "-o", bag_dir,
             "/stimulus_publisher/experiment_info",
             "/stimulus_publisher/stimulus_state",
             "/stimulus_publisher/trial_start"],
        output="screen",
    )

    # Give the recorder ~2 s to come up and finish discovery before the node
    # starts publishing its ARMED heartbeats.
    delayed_node = TimerAction(period=2.0, actions=[node])

    shutdown_when_node_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason="assay run finished"))],
        )
    )

    return LaunchDescription(
        args + [bag_all, bag_selected, delayed_node, shutdown_when_node_exits]
    )
