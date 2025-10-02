#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from dsr_msgs2.srv import GetToolForce

class ToolForcePublisher(Node):
    def __init__(self):
        super().__init__('tool_force_publisher')

        # Publisher
        self.force_pub = self.create_publisher(WrenchStamped, '/tcp_force', 10)

        # Service Client
        self.cli = self.create_client(GetToolForce, '/aux_control/get_tool_force')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /aux_control/get_tool_force ...')

        # Timer (100Hz -> 0.01s)
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        req = GetToolForce.Request()
        req.ref = 1   # 0=BASE, 1=TOOL, 2=WORLD → TCP
        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return

        # tool_force[6] (Fx, Fy, Fz, Tx, Ty, Tz)
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tcp"

        msg.wrench.force.x = res.tool_force[0]
        msg.wrench.force.y = res.tool_force[1]
        msg.wrench.force.z = res.tool_force[2]
        msg.wrench.torque.x = res.tool_force[3]
        msg.wrench.torque.y = res.tool_force[4]
        msg.wrench.torque.z = res.tool_force[5]

        self.force_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ToolForcePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
