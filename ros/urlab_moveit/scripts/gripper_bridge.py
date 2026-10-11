#!/usr/bin/env python3
# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
# Licensed under the Apache License, Version 2.0.
"""GripperCommand -> URLab gripper bridge.

The Franka gripper is a single tendon actuator (ctrl 0..255) that drives both
fingers symmetrically; there is no per-finger joint actuator. MoveIt controls the
'hand' group through a GripperCommand action (target finger opening, metres).
This node serves that action and maps the opening onto the tendon actuator's
ctrl, published on /<art>/joint_command by the actuator's own name (URLab now
resolves a joint_command entry to an actuator by name, not only by driven joint).

  ctrl = clamp(position, 0, finger_max) / finger_max * ctrl_max

Runs on a MultiThreadedExecutor and claims control (ros:urlab) so writes land.
"""
import sys
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class GripperBridge(Node):
    def __init__(self, art, actuator, finger_max, ctrl_max):
        super().__init__("urlab_gripper_bridge",
                         parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.art = art
        self.actuator = actuator
        self.finger_max = finger_max
        self.ctrl_max = ctrl_max
        self._cb = ReentrantCallbackGroup()
        self._cmd = self.create_publisher(JointState, f"/{art}/joint_command", 10)
        self._claim_cli = self.create_client(
            Trigger, f"/{art}/claim_control", callback_group=self._cb)
        self._server = ActionServer(
            self, GripperCommand, "/hand_controller/gripper_cmd",
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self._cb)
        self.get_logger().info(
            f"gripper bridge up: GripperCommand -> /{art}/joint_command '{actuator}'")

    def claim(self, timeout=2.0):
        if not self._claim_cli.service_is_ready():
            if not self._claim_cli.wait_for_service(timeout_sec=timeout):
                return
        fut = self._claim_cli.call_async(Trigger.Request())
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.02)

    def _publish_ctrl(self, ctrl):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.actuator]
        msg.position = [float(ctrl)]
        self._cmd.publish(msg)

    def _execute(self, goal_handle):
        self.claim()
        pos = float(goal_handle.request.command.position)
        pos = max(0.0, min(self.finger_max, pos))
        ctrl = pos / self.finger_max * self.ctrl_max if self.finger_max > 0 else 0.0
        self.get_logger().info(f"gripper: finger={pos:.4f} -> ctrl={ctrl:.1f}")

        # Hold the command briefly so the tendon servo settles.
        for _ in range(30):
            self._publish_ctrl(ctrl)
            time.sleep(0.02)

        goal_handle.succeed()
        result = GripperCommand.Result()
        result.position = pos
        result.reached_goal = True
        result.stalled = False
        return result


def main():
    art, actuator, finger_max, ctrl_max = "franka", "actuator8", 0.04, 255.0
    for a in sys.argv[1:]:
        if a.startswith("--art="):
            art = a.split("=", 1)[1]
        elif a.startswith("--actuator="):
            actuator = a.split("=", 1)[1]
        elif a.startswith("--finger-max="):
            finger_max = float(a.split("=", 1)[1])
        elif a.startswith("--ctrl-max="):
            ctrl_max = float(a.split("=", 1)[1])
    rclpy.init()
    node = GripperBridge(art, actuator, finger_max, ctrl_max)
    node.claim(timeout=15.0)
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
