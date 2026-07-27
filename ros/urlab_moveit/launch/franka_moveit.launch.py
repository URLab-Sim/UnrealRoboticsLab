# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
# Licensed under the Apache License, Version 2.0.
"""Bring up MoveIt move_group + rviz MotionPlanning for the URLab Franka, with
trajectory execution streamed to the sim via the urlab trajectory bridge.

Nodes:
  move_group              plans; loads URDF (urdf:=<path>), SRDF, kinematics,
                          joint limits, OMPL, and the FollowJointTrajectory
                          controller mapping. Reads current state from
                          /<art>/joint_states (remapped).
  robot_state_publisher   URDF -> /tf from /<art>/joint_states.
  static world->link0     anchors the (bare-link) URDF under the 'world' frame.
  trajectory_bridge       serves /panda_arm_controller/follow_joint_trajectory
                          and streams points to /<art>/joint_command.
  rviz2                   MotionPlanning display (plan + execute interactively).

Requires MoveIt for the active ROS distro (e.g. `pixi add ros-<distro>-moveit`).
The sim must be up in Live mode (the urlab Python bring-up) before executing.
"""
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_THIS = os.path.dirname(os.path.abspath(__file__))
_CFG = os.path.normpath(os.path.join(_THIS, "..", "config"))
_RVIZ = os.path.normpath(os.path.join(_THIS, "..", "rviz", "moveit.rviz"))


def _load(name):
    with open(os.path.join(_CFG, name), "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _read(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def _setup(context, *args, **kwargs):
    art = LaunchConfiguration("art").perform(context)
    urdf_path = LaunchConfiguration("urdf").perform(context)
    if not urdf_path:
        raise RuntimeError("franka_moveit.launch.py requires 'urdf:=<path>'")

    robot_description = {"robot_description": _read(urdf_path)}
    robot_description_semantic = {
        "robot_description_semantic": _read(os.path.join(_CFG, "franka.srdf"))
    }
    kinematics = _load("kinematics.yaml")
    joint_limits = {"robot_description_planning": _load("joint_limits.yaml")}
    ompl = _load("ompl_planning.yaml")
    controllers = _load("moveit_controllers.yaml")

    planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl,
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 2.0,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.05,
    }
    # MuJoCo's soft joint limits let joints transiently overshoot the URDF hard
    # limits (e.g. the Franka's razor-thin joint4 upper bound), which MoveIt's
    # CheckStartStateBounds rejects. Clamp marginal start-state violations instead
    # of failing the plan; the tight URDF limits still bound the planned motion.
    start_state = {"start_state_max_bounds_error": 0.1}
    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    use_sim_time = {"use_sim_time": True}

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics},
            joint_limits,
            planning_pipeline,
            trajectory_execution,
            start_state,
            controllers,
            planning_scene_monitor,
            use_sim_time,
        ],
        # move_group reads current joint state from 'joint_states'; the sim
        # publishes it namespaced.
        remappings=[("joint_states", f"/{art}/joint_states")],
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, use_sim_time],
        remappings=[("joint_states", f"/{art}/joint_states")],
    )

    world_to_root = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_root",
        arguments=["--frame-id", "world", "--child-frame-id", "link0",
                   "--x", "0", "--y", "0", "--z", "0"],
        parameters=[use_sim_time],
        output="screen",
    )

    bridge = _bridge_process(art)
    gripper = _gripper_process(art)

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", _RVIZ],
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics},
            planning_pipeline,
            use_sim_time,
        ],
    )

    return [move_group, rsp, world_to_root, bridge, gripper, rviz]


def _bridge_process(art):
    # Run the trajectory bridge directly by path (urlab_moveit is a source tree,
    # not an installed package), so `python trajectory_bridge.py` works anywhere.
    from launch.actions import ExecuteProcess
    script = os.path.normpath(os.path.join(_THIS, "..", "scripts", "trajectory_bridge.py"))
    return ExecuteProcess(cmd=["python", script, f"--art={art}"], output="screen")


def _gripper_process(art):
    from launch.actions import ExecuteProcess
    script = os.path.normpath(os.path.join(_THIS, "..", "scripts", "gripper_bridge.py"))
    return ExecuteProcess(cmd=["python", script, f"--art={art}"], output="screen")


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("art", default_value="franka"),
        DeclareLaunchArgument("urdf", default_value=""),
        OpaqueFunction(function=_setup),
    ])
