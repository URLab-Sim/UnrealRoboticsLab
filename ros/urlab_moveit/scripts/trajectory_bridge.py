#!/usr/bin/env python3
# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
# Licensed under the Apache License, Version 2.0.
"""FollowJointTrajectory -> URLab bridge.

MoveIt executes a planned trajectory by sending a FollowJointTrajectory action
to the controller named in moveit_controllers.yaml. This node serves that action
and streams each trajectory point's joint positions onto /<art>/joint_command
(the same topic the jog GUI uses), which URLab applies as position control in
Live mode. URLab maps the JointState.name[] to the actuators that drive those
joints, so no ros2_control stack is needed on the sim side.

Runs on a MultiThreadedExecutor: the execute callback paces the trajectory in
its own thread (using sim time via /clock) while the executor keeps servicing
the clock and action interfaces. It claims control (ros:urlab) up front and
re-asserts the claim per goal so writes are always accepted.
"""
import sys
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class TrajectoryBridge(Node):
    def __init__(self, art: str):
        super().__init__("urlab_trajectory_bridge",
                         parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.art = art
        self._cb = ReentrantCallbackGroup()
        self._cmd = self.create_publisher(JointState, f"/{art}/joint_command", 10)
        self._claim_cli = self.create_client(
            Trigger, f"/{art}/claim_control", callback_group=self._cb)
        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            "/panda_arm_controller/follow_joint_trajectory",
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self._cb,
        )
        self.get_logger().info(
            f"trajectory bridge up: FollowJointTrajectory -> /{art}/joint_command")

    def claim(self, timeout=2.0):
        """Claim control (ros:urlab). Safe to call repeatedly; the claim never
        expires but re-asserting is cheap and covers a lost startup race."""
        if not self._claim_cli.service_is_ready():
            if not self._claim_cli.wait_for_service(timeout_sec=timeout):
                self.get_logger().warn("claim_control service not ready")
                return
        fut = self._claim_cli.call_async(Trigger.Request())
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        if fut.done() and fut.result():
            self.get_logger().info(f"claim_control: {fut.result().message}")

    def _publish(self, names, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(names)
        msg.position = [float(p) for p in positions]
        self._cmd.publish(msg)

    def _execute(self, goal_handle):
        self.claim()  # re-assert ownership before every trajectory
        traj = goal_handle.request.trajectory
        names = list(traj.joint_names)
        points = list(traj.points)
        self.get_logger().info(f"executing trajectory: {len(points)} points")

        start = self.get_clock().now()
        for pt in points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result(
                    error_code=FollowJointTrajectory.Result.SUCCESSFUL)
            target = start + Duration(seconds=pt.time_from_start.sec,
                                      nanoseconds=pt.time_from_start.nanosec)
            # The executor (other threads) advances the clock; just sleep here.
            while rclpy.ok() and self.get_clock().now() < target:
                time.sleep(0.004)
            self._publish(names, pt.positions)

        # Hold the final target briefly so the position servo settles on it.
        if points:
            for _ in range(25):
                self._publish(names, points[-1].positions)
                time.sleep(0.02)

        goal_handle.succeed()
        self.get_logger().info("trajectory complete")
        return FollowJointTrajectory.Result(
            error_code=FollowJointTrajectory.Result.SUCCESSFUL)


def main():
    art = "franka"
    for a in sys.argv[1:]:
        if a.startswith("--art="):
            art = a.split("=", 1)[1]
    rclpy.init()
    node = TrajectoryBridge(art)
    node.claim(timeout=15.0)  # robust startup claim (service may be slow to appear)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
