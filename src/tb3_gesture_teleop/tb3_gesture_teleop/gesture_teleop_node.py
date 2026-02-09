#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from mediapipe_ros2_interfaces.msg import HandGesture  # 就是你刚刚 show 的这个 msg

class Tb3GestureTeleop(Node):
    def __init__(self):
        super().__init__('tb3_gesture_teleop')

        # 订阅手势事件
        self.sub = self.create_subscription(
            HandGesture,
            '/mediapipe/hand/gesture',
            self.gesture_callback,
            10
        )

        # 发布速度
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        #一段时间没手势就停车
        self.last_cmd = Twist()
        self.timeout_sec = 0.8
        self.last_update_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.watchdog_timer)

        self.get_logger().info(
            'Tb3GestureTeleop 已启动：根据 /mediapipe/hand/gesture 控制 /cmd_vel'
        )

    def gesture_callback(self, msg: HandGesture):
        gesture = msg.gesture
        score = msg.score
        self.get_logger().info(f'gesture="{gesture}", score={score:.2f}')

        if score < 0.6:
            return

        cmd = Twist()

        if gesture == 'Thumb_Up':
            cmd.linear.x = 0.3          # 前进
        elif gesture == 'Thumb_Down':
            cmd.linear.x = -0.3       # 后退
        elif gesture == 'Victory':
            cmd.angular.z = 1.0         # 左转
        elif gesture == 'ILoveYou':
            cmd.angular.z = -1.0      # 右转
        elif gesture == 'Open_Palm':
            cmd = Twist()               # 停车
        else:
            cmd = Twist()              
        self.last_cmd = cmd
        self.last_update_time = self.get_clock().now()
        self.cmd_pub.publish(cmd)

    def watchdog_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9

        if dt > self.timeout_sec:
            if self.last_cmd.linear.x != 0.0 or self.last_cmd.angular.z != 0.0:
                self.get_logger().info('超时未检测到手势，自动停车')
            self.last_cmd = Twist()
            self.cmd_pub.publish(self.last_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Tb3GestureTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
