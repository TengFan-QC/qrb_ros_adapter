# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # The Manager has no parameters; Flows are configured on the HCM side.
    return LaunchDescription([
        Node(
            package="qrb_ros_adapter",
            executable="ros_ipc_bridge",
            output="screen",
        )
    ])
