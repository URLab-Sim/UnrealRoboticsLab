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

It claims control (ros:urlab) on startup so the writes are accepted, and uses
sim time (/clock) so trajectory timing matches the (possibly non-real-time) sim.
"""
import sys

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class TrajectoryBridge(Node):
    def __init__(self, art: str):
        super().__init__("urlab_trajectory_bridge")
        self.art = art
        # use_sim_time so Duration/clock line up with the sim's /clock.
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])

        self._cmd = self.create_publisher(JointState, f"/{art}/joint_command", 10)
        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            f"/panda_arm_controller/follow_joint_trajectory",
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
        )
        self._claim(art)
        self.get_logger().info(
            f"trajectory bridge up: FollowJointTrajectory -> /{art}/joint_command"
        )

    def _claim(self, art: str):
        cli = self.create_client(Trigger, f"/{art}/claim_control")
        if cli.wait_for_service(timeout_sec=5.0):
            fut = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            if fut.result():
                self.get_logger().info(f"claim_control: {fut.result().message}")
        else:
            self.get_logger().warn("claim_control service not available; jog may be dropped")

    def _publish_point(self, names, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(names)
        msg.position = [float(p) for p in positions]
        self._cmd.publish(msg)

    def _execute(self, goal_handle):
        traj = goal_handle.request.trajectory
        names = list(traj.joint_names)
        points = list(traj.points)
        self.get_logger().info(f"executing trajectory: {len(points)} points, joints={names}")

        start = self.get_clock().now()
        for i, pt in enumerate(points):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result
            target = start + Duration(seconds=pt.time_from_start.sec,
                                      nanoseconds=pt.time_from_start.nanosec)
            # Wait (in sim time) until this point is due, then command it.
            while rclpy.ok() and self.get_clock().now() < target:
                rclpy.spin_once(self, timeout_sec=0.005)
            self._publish_point(names, pt.positions)
            fb = FollowJointTrajectory.Feedback()
            fb.joint_names = names
            fb.desired = pt
            goal_handle.publish_feedback(fb)

        # Hold the final target briefly so the servo settles.
        if points:
            for _ in range(20):
                self._publish_point(names, points[-1].positions)
                rclpy.spin_once(self, timeout_sec=0.02)

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info("trajectory complete")
        return result


def main():
    art = "franka"
    for a in sys.argv[1:]:
        if a.startswith("--art="):
            art = a.split("=", 1)[1]
    rclpy.init()
    node = TrajectoryBridge(art)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
