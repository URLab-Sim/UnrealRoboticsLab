# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
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

"""Jog a URLab articulation with the standard joint_state_publisher_gui sliders.

This launch file wires only stock ROS 2 nodes. No custom node, no relay:

  joint_state_publisher_gui  builds one slider per non-fixed URDF joint (limits
                             from the URDF) and publishes sensor_msgs/JointState.
                             Its output is remapped off /joint_states onto the
                             command topic /<art>/joint_command, which URLab
                             subscribes to and applies as control in Live mode.

  robot_state_publisher      serves the URDF on /robot_description so rviz's
                             RobotModel display can load it. Its /tf outputs are
                             remapped to dead topics because URLab publishes the
                             authoritative /tf (parent 'world', child
                             '<art>/<body>') from the live sim.

  rviz2                      RobotModel + TF, using the bundled franka.rviz.

The URDF must name its joints to match URLab's canonical joint names (the
JointState.name[] URLab publishes on /<art>/joint_states, e.g. actuator/joint
short names) and its links to match URLab's TF frames '<art>/<body>', with the
fixed frame 'world'. Otherwise the sliders command the wrong joints and rviz
cannot place the links.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_RVIZ = os.path.normpath(os.path.join(_THIS_DIR, "..", "rviz", "franka.rviz"))


def _setup(context, *args, **kwargs):
    art = LaunchConfiguration("art").perform(context)
    urdf_path = LaunchConfiguration("urdf").perform(context)
    rviz_config = LaunchConfiguration("rviz").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context).lower() in ("1", "true", "yes")

    if not urdf_path:
        raise RuntimeError(
            "franka_jog.launch.py requires 'urdf:=<path>' (the URDF exported for "
            "the articulation; its joint/link names must match URLab's)."
        )
    with open(urdf_path, "r", encoding="utf-8") as handle:
        robot_description = handle.read()

    command_topic = f"/{art}/joint_command"

    # Claim control for the ROS source so URLab accepts the jog. URLab gates
    # control writes on ownership; the jog is dropped until '<art>/claim_control'
    # is called (the claim never expires, so once is enough). Requires the sim to
    # be in Live mode with the control source set to the network slot, which the
    # Python bring-up does before this launch.
    claim = ExecuteProcess(
        cmd=["ros2", "service", "call", f"/{art}/claim_control", "std_srvs/srv/Trigger"],
        output="screen",
    )

    nodes = [
        claim,
        # Sliders. Publishes JointState; remapped onto the URLab command topic so
        # it does not collide with URLab's own /<art>/joint_states (real state).
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
            remappings=[("joint_states", command_topic)],
            output="screen",
        ),
        # Poses the URDF for rviz: subscribe URLab's real /<art>/joint_states and
        # publish /tf for the (bare-named) URDF links so RobotModel can place them.
        # (URLab also publishes /tf under '<art>/<body>'; the bare frames here are
        # distinct and are what the bare-link URDF matches.)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
            remappings=[("joint_states", f"/{art}/joint_states")],
            output="screen",
        ),
        # rsp roots the tree at the URDF root link; anchor it under 'world' so the
        # bundled rviz fixed frame resolves.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_root",
            # Flag form, not the deprecated positional "x y z ... frame child":
            # the positional path crashes (access violation) on the Windows build,
            # which severs world->link0 and leaves the RobotModel unable to place
            # any link against the 'world' fixed frame.
            arguments=[
                "--frame-id", "world", "--child-frame-id", "link0",
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
            ],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
    ]

    if use_rviz:
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        )

    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "art",
                default_value="franka",
                description="Articulation name (topic namespace): commands go to /<art>/joint_command.",
            ),
            DeclareLaunchArgument(
                "urdf",
                default_value="",
                description="Path to the URDF exported for the articulation (required).",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value=_DEFAULT_RVIZ,
                description="Path to the rviz2 config.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start rviz2 alongside the sliders.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
