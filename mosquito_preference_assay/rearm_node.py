#!/usr/bin/env python3
"""Arm the rig for the next animal, without restarting the experiment.

    ros2 run mosquito_preference_assay rearm

Run it from a second terminal after a trial finishes and you have swapped the
animal. A fresh pairing is drawn, the stimuli start playing it, and the
detector -- which holds fire until the display reports armed -- resumes on its
own. The launch keeps running, so the projector never goes dark and the next
trial writes its own folder.

Refused while a trial is still running: re-arming then would discard a trial
that is still recording, which is never what you meant.
"""

import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

SERVICE = "/stimulus_publisher/rearm"


def main(args=None):
    rclpy.init(args=args)
    node = Node("rearm_client")
    client = node.create_client(Trigger, SERVICE)

    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(
            f"no '{SERVICE}' -- is the experiment running? "
            f"(the launch must still be up; re-arm cannot start a new one)")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    result = future.result()

    code = 0
    if result is None:
        node.get_logger().error("no reply from the experiment")
        code = 1
    elif result.success:
        print(f"\nRE-ARMED -- {result.message}\n")
    else:
        print(f"\nnot armed: {result.message}\n")
        code = 1

    node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
