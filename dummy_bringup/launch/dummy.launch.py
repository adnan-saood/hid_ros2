# Copyright 2025 Adnan Saood
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


from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command, FindExecutable


def generate_launch_description():
    pkg_share = get_package_share_directory("dummy_bringup")

    urdf_file = os.path.join(pkg_share, "urdf", "hid_robot.urdf.xacro")
    yaml_file = os.path.join(pkg_share, "config", "dummy_controllers.yaml")

    # Load URDF into robot_description
    robot_description = Command([
        FindExecutable(name="xacro"), " ", urdf_file
    ])

    return LaunchDescription([
        # Static transform publisher to link dummy_hid_frame to world frame
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_hid_frame",
            arguments=["0", "0", "0", "0", "0", "0", "world", "dummy_hid_frame"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": robot_description},
                        yaml_file],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        ),
        # Node(
        #     package="controller_manager",
        #     executable="spawner",
        #     arguments=["dummy_pose_broadcaster", "--controller-manager", "/controller_manager"],
        # ),
        # Optional: Uncomment to enable generic HID broadcaster (raw data publisher)
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["hid_generic_broadcaster", "--controller-manager", "/controller_manager"],
        ),
        # Optional: Uncomment to enable diagnostic broadcaster (device health monitoring)
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["hid_diagnostic_broadcaster", "--controller-manager", "/controller_manager"],
        ),
    ])
