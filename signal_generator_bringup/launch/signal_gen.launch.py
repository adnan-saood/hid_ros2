# Copyright 2025
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Author: Adnan SAOOD

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command, FindExecutable


def generate_launch_description():
    pkg_share = get_package_share_directory("signal_generator_bringup")

    # Use the GENERATED urdf file (auto-created from schema)
    urdf_file = os.path.join(pkg_share, "urdf", "hid_robot.urdf.xacro")
    # Use the USER-CREATED controller config (for custom controller settings)
    yaml_file = os.path.join(pkg_share, "config", "signal_gen_controllers.yaml")

    robot_description = Command([
        FindExecutable(name="xacro"), " ", urdf_file
    ])

    return LaunchDescription([
        # Static transform from world to signal generator frame
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_signal_gen",
            arguments=["0", "0", "0", "0", "0", "0", "world", "signal_gen_frame"],
            output="screen",
        ),

        # ros2_control node
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": robot_description}, yaml_file],
            output="screen",
        ),

        # Joint state broadcaster (required)
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        ),

        # Generic broadcaster for reading output signal
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["hid_generic_broadcaster", "--controller-manager", "/controller_manager"],
        ),

        # Command controller for sending parameters to device
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["signal_command_controller", "--controller-manager", "/controller_manager"],
        ),

        # Optional: Diagnostic broadcaster
        # Node(
        #     package="controller_manager",
        #     executable="spawner",
        #     arguments=["hid_diagnostic_broadcaster", "--controller-manager", "/controller_manager"],
        # ),
    ])
