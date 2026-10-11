#!/usr/bin/env python3
# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
# Licensed under the Apache License, Version 2.0.
"""Scripted MoveIt pick of a scene object (the coke bottle) on the URLab Franka.

Sequence: open gripper -> plan arm to a pre-grasp pose above the object ->
approach to the grasp pose -> close gripper -> attach the object to the hand
(so the planner carries it) -> lift. The arm goes through move_group (obstacle
aware, using the live planning scene); the gripper goes through the GripperCommand
bridge. The grasp pose is parametric so it can be tuned against the actual object.

Run under the ROS env. The sim must be Live + the MoveIt stack (move_group,
trajectory + gripper bridges) up. Uses the object id the scene provider assigns.
"""
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile

from control_msgs.action import GripperCommand
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject,
                             Constraints, OrientationConstraint, PlanningScene,
                             PositionConstraint, PlanningOptions)
from shape_msgs.msg import SolidPrimitive

ARM = "panda_arm"
HAND_LINK = "hand"
FRAME = "world"

# Object + grasp geometry (world frame). Defaults target the coke bottle; tune
# via CLI. quat is xyzw for a top-down approach (hand z-axis pointing down).
OBJECT_ID = "Geom_0_0_1"
GRASP_XYZ = (0.70, -0.10, 0.12)
PREGRASP_DZ = 0.12          # pre-grasp / lift height above the grasp
GRASP_QUAT = (1.0, 0.0, 0.0, 0.0)  # 180deg about X -> hand points down
OPEN, CLOSED = 0.04, 0.0


class Pick(Node):
    def __init__(self):
        super().__init__("pick_bottle")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.move = ActionClient(self, MoveGroup, "/move_action")
        self.grip = ActionClient(self, GripperCommand, "/hand_controller/gripper_cmd")
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", QoSProfile(depth=1))

    # --- arm ---
    def pose_goal(self, xyz, quat, pos_tol=0.02, ori_tol=0.1):
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = FRAME
        pc.link_name = HAND_LINK
        pc.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[pos_tol]))
        p = Pose(); p.position = Point(x=xyz[0], y=xyz[1], z=xyz[2]); p.orientation.w = 1.0
        pc.constraint_region.primitive_poses.append(p)
        pc.weight = 1.0
        c.position_constraints.append(pc)
        oc = OrientationConstraint()
        oc.header.frame_id = FRAME
        oc.link_name = HAND_LINK
        oc.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        oc.absolute_x_axis_tolerance = ori_tol
        oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        return c

    def move_to(self, xyz, quat, label):
        if not self.move.wait_for_server(timeout_sec=15.0):
            print("move_action unavailable"); return False
        req = MoveGroup.Goal()
        req.request.group_name = ARM
        req.request.num_planning_attempts = 10
        req.request.allowed_planning_time = 8.0
        req.request.max_velocity_scaling_factor = 0.2
        req.request.max_acceleration_scaling_factor = 0.2
        req.request.goal_constraints.append(self.pose_goal(xyz, quat))
        req.planning_options = PlanningOptions()
        req.planning_options.plan_only = False
        fut = self.move.send_goal_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        gh = fut.result()
        if not gh or not gh.accepted:
            print(f"{label}: goal rejected"); return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=40.0)
        code = res.result().result.error_code.val if res.result() else None
        print(f"{label}: error_code={code} ({'OK' if code == 1 else 'FAIL'})")
        return code == 1

    # --- gripper ---
    def gripper(self, position, label):
        if not self.grip.wait_for_server(timeout_sec=10.0):
            print("gripper action unavailable"); return False
        g = GripperCommand.Goal()
        g.command.position = float(position)
        g.command.max_effort = 40.0
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        gh = fut.result()
        if not gh or not gh.accepted:
            print(f"{label}: rejected"); return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=10.0)
        print(f"{label}: done")
        return True

    # --- attach object to the hand so the planner carries it ---
    def attach(self, obj_id):
        ps = PlanningScene(); ps.is_diff = True
        aco = AttachedCollisionObject()
        aco.link_name = HAND_LINK
        aco.object.id = obj_id
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = ["hand", "left_finger", "right_finger"]
        ps.robot_state.attached_collision_objects.append(aco)
        ps.robot_state.is_diff = True
        # Remove the world copy so it is not double-counted.
        rem = CollisionObject(); rem.id = obj_id; rem.operation = CollisionObject.REMOVE
        ps.world.collision_objects.append(rem)
        for _ in range(5):
            self.scene_pub.publish(ps); time.sleep(0.1); rclpy.spin_once(self, timeout_sec=0.05)
        print(f"attached '{obj_id}' to {HAND_LINK}")

    def run(self):
        gx, gy, gz = GRASP_XYZ
        pre = (gx, gy, gz + PREGRASP_DZ)
        print("== PICK ==")
        self.gripper(OPEN, "open")
        if not self.move_to(pre, GRASP_QUAT, "pre-grasp"):
            return
        if not self.move_to((gx, gy, gz), GRASP_QUAT, "approach"):
            return
        self.gripper(CLOSED, "close")
        self.attach(OBJECT_ID)
        self.move_to(pre, GRASP_QUAT, "lift")
        print("== PICK sequence complete ==")


def main():
    rclpy.init()
    node = Pick()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
