#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MoveToTargetNode
----------------
- Subscribes to two topics:
    1. /target_pose  (dsr_cumotion/TargetPose)
    2. /target_joint (dsr_cumotion/TargetJoint)
- Sends goals to MoveIt MoveGroup Action
- Supports both joint and pose-based motion
"""

import math
import threading
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

import numpy as np
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MoveItErrorCodes,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    JointConstraint,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from dsr_msgs2.msg import TargetPose, TargetJoint


# Constants
MOVE_RESULT_WAIT_SEC = 120.0
DEFAULT_SERVER_WAIT_SEC = 5.0
POS_TOL = 0.05
ORI_TOL = 0.1

ORIENTATION_TABLE: Dict[int, Tuple[float, float, float, float]] = {
    1: (0.7071, 0.7071, 0.0, 0.0),
    2: (1.0, 0.0, 0.0, 0.0),
    3: (0.0, 1.0, 0.0, 0.0),
    4: (0.7071, -0.7071, 0.0, 0.0),
    5: (0.0, 0.7071, 0.7071, 0.0),
    6: (-0.7071, 0.0, 0.0, 0.7071),
    7: (0.7071, 0.0, 0.0, 0.7071),
    8: (0.0, 0.7071, -0.7071, 0.0),
    9: (1.0, 1.0, 1.0, 1.0),
    10: (-1.0, 1.0, -1.0, 1.0),
}


def movegroup_success(result: MoveGroup.Result) -> bool:
    return result.error_code.val == MoveItErrorCodes.SUCCESS


def normalize_quat(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / n, y / n, z / n, w / n) if n > 0 else (0.0, 0.0, 0.0, 1.0)


class MoveToTargetNode(Node):
    """Unified node that accepts either pose or joint goals."""

    def __init__(self):
        super().__init__("move_to_target_node")
        self.cb_group = ReentrantCallbackGroup()
        self._move_lock = threading.Lock()

        # Parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("group_name", "manipulator"),
                ("pipeline_id", ""),
                ("planner_id", ""),
                ("base_frame", "base_link"),
                ("eef_link", "link_6"),
                ("tool_frame", "link_6"),
                ("allowed_planning_time", 10.0),
                ("num_planning_attempts", 50),
                ("max_vel_scale", 0.1),
                ("max_acc_scale", 0.1),
            ],
        )

        # MoveGroup Action Client
        self._move_client = ActionClient(self, MoveGroup, "/move_action", callback_group=self.cb_group)

        # Subscriptions
        self.create_subscription(TargetPose, "/target_pose", self._on_target_pose, 10)
        self.create_subscription(TargetJoint, "/target_joint", self._on_target_joint, 10)

        self.get_logger().info("[MoveToTargetNode] Ready — waiting for /target_pose or /target_joint.")

    # ===== Pose Callback =====
    def _on_target_pose(self, msg: TargetPose):
        if not self._move_lock.acquire(blocking=False):
            self.get_logger().warn("Busy — ignoring new pose command.")
            return
        threading.Thread(target=self._move_pose_thread, args=(msg,), daemon=True).start()

    def _move_pose_thread(self, msg: TargetPose):
        try:
            if msg.orientation_id not in ORIENTATION_TABLE:
                self.get_logger().error(f"Invalid orientation ID: {msg.orientation_id}")
                return

            q = normalize_quat(*ORIENTATION_TABLE[msg.orientation_id])
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = msg.x, msg.y, msg.z
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q

            self.get_logger().info(
                f"[Pose] Moving to x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}, id={msg.orientation_id}"
            )
            success = self._move_to_pose(pose)
            self.get_logger().info("[Pose Move] ✅ Success." if success else "[Pose Move] ❌ Failed.")
        finally:
            self._move_lock.release()

    def _on_target_joint(self, msg: TargetJoint):
        if not self._move_lock.acquire(blocking=False):
            self.get_logger().warn("Busy — ignoring new joint command.")
            return
        threading.Thread(target=self._move_joint_thread, args=(msg,), daemon=True).start()

    def _move_joint_thread(self, msg: TargetJoint):
        try:
            if len(msg.joint_deg) != 6:
                self.get_logger().error("Expected 6 joint values.")
                return

            joint_rad = [np.deg2rad(d) for d in msg.joint_deg]
            self.get_logger().info(f"[Joint] Moving to (deg): {msg.joint_deg}")
            success = self._move_to_joint(joint_rad)
            self.get_logger().info("[Joint Move] ✅ Success." if success else "[Joint Move] ❌ Failed.")
        finally:
            self._move_lock.release()

    def _move_to_pose(self, pose: Pose) -> bool:
        if not self._wait_for_server():
            return False
        goal = self._build_pose_goal(pose)
        return self._send_goal(goal)

    def _move_to_joint(self, joints_rad) -> bool:
        if not self._wait_for_server():
            return False
        goal = self._build_joint_goal(joints_rad)
        return self._send_goal(goal)

    def _wait_for_server(self) -> bool:
        if self._move_client.wait_for_server(timeout_sec=DEFAULT_SERVER_WAIT_SEC):
            return True
        self.get_logger().error("[MoveGroup] server not available.")
        return False

    def _send_goal(self, goal: MoveGroup.Goal) -> bool:
        done = threading.Event()
        result_ok = {"v": False}

        def on_goal_response(fut):
            try:
                gh = fut.result()
                if not gh.accepted:
                    self.get_logger().error("Goal rejected by MoveIt")
                    done.set()
                    return

                def on_result(res_future):
                    try:
                        res = res_future.result().result
                        result_ok["v"] = movegroup_success(res)
                    except Exception as e:
                        self.get_logger().error(f"get_result failed: {e}")
                    finally:
                        done.set()

                gh.get_result_async().add_done_callback(on_result)
            except Exception as e:
                self.get_logger().error(f"send_goal failed: {e}")
                done.set()

        self._move_client.send_goal_async(goal).add_done_callback(on_goal_response)
        done.wait(timeout=MOVE_RESULT_WAIT_SEC)
        return result_ok["v"]

    def _build_pose_goal(self, target_pose: Pose) -> MoveGroup.Goal:
        p = self.get_parameter
        goal = MoveGroup.Goal()
        goal.request.group_name = p("group_name").value
        goal.request.pipeline_id = p("pipeline_id").value
        goal.request.planner_id = p("planner_id").value
        goal.request.num_planning_attempts = int(p("num_planning_attempts").value)
        goal.request.allowed_planning_time = float(p("allowed_planning_time").value)
        goal.request.max_velocity_scaling_factor = float(p("max_vel_scale").value)
        goal.request.max_acceleration_scaling_factor = float(p("max_acc_scale").value)

        pos_c = PositionConstraint()
        pos_c.header.frame_id = p("base_frame").value
        pos_c.link_name = p("eef_link").value
        pos_c.weight = 1.0
        prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[POS_TOL] * 3)
        bv = BoundingVolume()
        bv.primitives.append(prim)
        bv.primitive_poses.append(target_pose)
        pos_c.constraint_region = bv

        ori_c = OrientationConstraint()
        ori_c.header.frame_id = p("base_frame").value
        ori_c.link_name = p("tool_frame").value
        ori_c.orientation = target_pose.orientation
        ori_c.absolute_x_axis_tolerance = ORI_TOL
        ori_c.absolute_y_axis_tolerance = ORI_TOL
        ori_c.absolute_z_axis_tolerance = ORI_TOL
        ori_c.weight = 1.0

        cs = Constraints()
        cs.position_constraints.append(pos_c)
        cs.orientation_constraints.append(ori_c)
        goal.request.goal_constraints = [cs]
        return goal

    def _build_joint_goal(self, joints_rad) -> MoveGroup.Goal:
        p = self.get_parameter
        goal = MoveGroup.Goal()
        goal.request.group_name = p("group_name").value
        goal.request.pipeline_id = p("pipeline_id").value
        goal.request.planner_id = p("planner_id").value
        goal.request.num_planning_attempts = int(p("num_planning_attempts").value)
        goal.request.allowed_planning_time = float(p("allowed_planning_time").value)
        goal.request.max_velocity_scaling_factor = float(p("max_vel_scale").value)
        goal.request.max_acceleration_scaling_factor = float(p("max_acc_scale").value)

        constraints = Constraints()
        for i, pos in enumerate(joints_rad):
            jc = JointConstraint()
            jc.joint_name = f"joint_{i+1}"
            jc.position = pos
            jc.tolerance_above = np.deg2rad(1.0)
            jc.tolerance_below = np.deg2rad(1.0)
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints = [constraints]
        return goal


def main(args=None):
    rclpy.init(args=args)
    node = MoveToTargetNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
