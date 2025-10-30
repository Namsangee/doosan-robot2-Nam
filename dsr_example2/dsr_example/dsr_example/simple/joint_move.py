#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Send MoveGroup Action Goal with 6 Joint Constraints (Degree Input)
Using transforms3d + numpy for angle conversion
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
import numpy as np


class JointMoveClient(Node):
    def __init__(self):
        super().__init__('joint_move_client_deg')

        self._action_client = ActionClient(self, MoveGroup, '/move_action')

        # joint 각도 [deg]
        joint_positions_deg = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joint_names = [f'joint_{i+1}' for i in range(6)]

        # degree → rad 변환
        joint_positions_rad = [np.deg2rad(d) for d in joint_positions_deg]

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'manipulator'

        constraints = Constraints()
        for name, pos in zip(joint_names, joint_positions_rad):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = np.deg2rad(1.0)
            jc.tolerance_below = np.deg2rad(1.0)
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)

        self.get_logger().info('Waiting for /move_action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Sending joint goal (deg → rad converted)...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected ❌')
            return
        self.get_logger().info('Goal accepted ✅ waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'MoveIt Result Code: {result.error_code.val}')
        self.get_logger().info('Motion complete ✅')


def main():
    rclpy.init()
    node = JointMoveClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
