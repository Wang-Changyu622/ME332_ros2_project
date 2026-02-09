#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyToCmdVel(Node):
    def __init__(self):
        super().__init__('joy_to_cmd_vel')
        self.declare_parameter('axis_linear', 1)
        self.declare_parameter('axis_angular', 0)
        self.declare_parameter('scale_linear', 0.22)
        self.declare_parameter('scale_angular', 0.8)
        self.declare_parameter('deadzone', 0.05)

        self.axis_linear = self.get_parameter('axis_linear').value
        self.axis_angular = self.get_parameter('axis_angular').value
        self.scale_linear = self.get_parameter('scale_linear').value
        self.scale_angular = self.get_parameter('scale_angular').value
        self.deadzone = self.get_parameter('deadzone').value

        # 订阅 /joy
        self.sub_joy = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        # 发布 /cmd_vel
        self.pub_cmd = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.get_logger().info('joy_to_cmd_vel node started (no enable button).')

    def joy_callback(self, msg: Joy):
        twist = Twist()

        lin_val = 0.0
        ang_val = 0.0

        if 0 <= self.axis_linear < len(msg.axes):
            lin_val = msg.axes[self.axis_linear]
        if 0 <= self.axis_angular < len(msg.axes):
            ang_val = msg.axes[self.axis_angular]

        if abs(lin_val) < self.deadzone:
            lin_val = 0.0
        if abs(ang_val) < self.deadzone:
            ang_val = 0.0

        twist.linear.x = lin_val * self.scale_linear
        twist.angular.z = ang_val * self.scale_angular

        self.pub_cmd.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
