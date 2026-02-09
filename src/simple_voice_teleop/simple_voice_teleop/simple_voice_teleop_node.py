#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import re
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


def chinese_num_to_float(word: str):

    cn_digit = {
        '零': 0, '〇': 0,
        '一': 1, '二': 2, '两': 2, '三': 3, '四': 4,
        '五': 5, '六': 6, '七': 7, '八': 8, '九': 9,
    }

    if word == '半':
        return 0.5

    if len(word) == 1 and word in cn_digit:
        return float(cn_digit[word])

    if word == '十':
        return 10.0

    if len(word) == 2 and word[0] == '十' and word[1] in cn_digit:
        return 10.0 + cn_digit[word[1]]

    if len(word) == 2 and word[1] == '十' and word[0] in cn_digit:
        return float(cn_digit[word[0]] * 10)

    if len(word) == 3 and word[1] == '十' and word[0] in cn_digit and word[2] in cn_digit:
        return float(cn_digit[word[0]] * 10 + cn_digit[word[2]])

    return None


class SimpleVoiceTeleop(Node):
    def __init__(self):
        super().__init__('simple_voice_teleop')

        #订阅语音转文字结果
        self.sub = self.create_subscription(
            String,
            '/voice_cmd_text',      # ASR节点发布到的topic
            self.cmd_callback,
            10
        )

        #发布到底盘控制
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.slow_v = 0.10
        self.normal_v = 0.20
        self.fast_v = 0.30
        self.normal_w = 0.8  

        self.get_logger().info(
            'SimpleVoiceTeleop 已启动：订阅 /voice_cmd_text，解析后发布 /cmd_vel'
        )

    def cmd_callback(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f'收到指令文本: "{text}"')

        if self.handle_shape_command(text):
            return

        try:
            linear_x, angular_z, duration = self.parse_basic_command(text)
        except ValueError as e:
            self.get_logger().warn(f'无法解析指令: {e}')
            return

        self.get_logger().info(
            f'解析结果: linear_x={linear_x:.3f}, '
            f'angular_z={angular_z:.3f}, duration={duration:.2f}s'
        )

        self.execute_motion(linear_x, angular_z, duration)

    #形状类指令处理
    def handle_shape_command(self, text: str) -> bool:
        t = text.replace('，', '').replace('。', '')

        def extract_length_m(default_value: float = 0.5):

            m = re.search(r'([0-9]+(?:\.[0-9]+)?)\s*米', t)
            if m:
                return float(m.group(1))
 
            m_cn = re.search(r'([一二两三四五六七八九十半零〇]+)\s*米', t)
            if m_cn:
                v = chinese_num_to_float(m_cn.group(1))
                if v is not None:
                    return v
            return default_value

        #正方形
        if ('正方形' in t) or ('方形' in t):
            side = extract_length_m(0.5)
            self.get_logger().info(f'识别为：正方形轨迹，边长≈{side:.2f} m')
            self.execute_square(side)
            return True

        #等边三角形
        if '三角形' in t:
            side = extract_length_m(0.5)
            self.get_logger().info(f'识别为：等边三角形轨迹，边长≈{side:.2f} m')
            self.execute_equilateral_triangle(side)
            return True

        #圆
        if ('圆' in t) or ('画个圈' in t) or ('画圈' in t):
            radius = extract_length_m(0.4)   # 默认半径 0.4 m
            self.get_logger().info(f'识别为：圆形轨迹，半径≈{radius:.2f} m')
            self.execute_circle(radius)
            return True

        return False

    #普通指令解析
    def parse_basic_command(self, text: str):

        text = text.replace('，', '').replace('。', '')
        text = text.replace('秒钟', '秒').replace('多秒', '秒')

        # 默认速度与时间
        slow_v = self.slow_v
        normal_v = self.normal_v
        fast_v = self.fast_v
        normal_w = self.normal_w

        default_time_move = 1.5
        default_time_turn = 1.0

        if any(k in text for k in ['停', '停止', '刹车']):
            return 0.0, 0.0, 0.0

        sec = None
        m_sec = re.search(r'([0-9]+(?:\.[0-9]+)?)\s*秒', text)
        if m_sec:
            sec = float(m_sec.group(1))
        else:
            m_sec_cn = re.search(r'([一二两三四五六七八九十半零〇]+)\s*秒', text)
            if m_sec_cn:
                sec = chinese_num_to_float(m_sec_cn.group(1))

        deg = None
        m_deg = re.search(r'([0-9]+(?:\.[0-9]+)?)\s*度', text)
        if m_deg:
            deg = float(m_deg.group(1))
        else:
            m_deg_cn = re.search(r'([一二两三四五六七八九十零〇]+)\s*度', text)
            if m_deg_cn:
                deg = chinese_num_to_float(m_deg_cn.group(1))

        #判断方向
        is_forward = any(k in text for k in ['前进', '往前', '向前', '前面'])
        is_back = any(k in text for k in ['后退', '往后', '向后', '后面'])

        is_left = any(k in text for k in ['左转', '向左', '左边'])
        is_right = any(k in text for k in ['右转', '向右', '右边'])

        #判断速度档位
        if '慢速' in text or '慢一点' in text or '慢点' in text:
            v = slow_v
        elif '快速' in text or '快一点' in text or '快点' in text:
            v = fast_v
        else:
            v = normal_v

        #前后运动
        if is_forward or is_back:
            linear_x = v if is_forward else -v
            angular_z = 0.0
            duration = sec if sec is not None else default_time_move
            return linear_x, angular_z, duration

        #转向运动
        if is_left or is_right:
            linear_x = 0.0
            angular_z = normal_w if ('左' in text) else -normal_w

            if deg is not None:
                rad = deg * math.pi / 180.0
                duration = abs(rad / normal_w)
            else:
                duration = default_time_turn

            return linear_x, angular_z, duration

        raise ValueError(f'无法识别的指令: "{text}"')
    
    def execute_motion(self, linear_x: float, angular_z: float, duration: float):

        rate_hz = 20.0  # 20Hz
        dt = 1.0 / rate_hz

        if duration <= 0.0:
            self.cmd_pub.publish(Twist())
            self.get_logger().info('执行：立即停车')
            return

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        self.get_logger().info(
            f'执行动作: v={linear_x:.2f} m/s, '
            f'w={angular_z:.2f} rad/s, duration={duration:.2f}s'
        )

        t0 = time.time()
        while rclpy.ok():
            now = time.time()
            if now - t0 > duration:
                break
            self.cmd_pub.publish(twist)
            time.sleep(dt)

        # 停车
        self.cmd_pub.publish(Twist())
        self.get_logger().info('动作结束，自动停车')

    #正方形
    def execute_square(self, side_m: float):
        v = self.normal_v
        w = self.normal_w

        if side_m <= 0.05:
            side_m = 0.05

        move_t = side_m / v
        turn_deg = 90.0
        turn_t = math.radians(turn_deg) / w

        self.get_logger().info(
            f'开始正方形轨迹：边长={side_m:.2f}m, v={v:.2f}, w={w:.2f}'
        )

        for i in range(4):
            self.get_logger().info(f'正方形：第 {i+1}/4 条边')
            # 直线
            self.execute_motion(v, 0.0, move_t)
            # 左转 90°
            self.execute_motion(0.0, w, turn_t)

        self.get_logger().info('正方形轨迹完成')

    #等边三角形
    def execute_equilateral_triangle(self, side_m: float):

        v = self.normal_v
        w = self.normal_w

        if side_m <= 0.05:
            side_m = 0.05

        move_t = side_m / v
        turn_deg = 120.0
        turn_t = math.radians(turn_deg) / w

        self.get_logger().info(
            f'开始等边三角形轨迹：边长={side_m:.2f}m, v={v:.2f}, w={w:.2f}'
        )

        for i in range(3):
            self.get_logger().info(f'三角形：第 {i+1}/3 条边')
            self.execute_motion(v, 0.0, move_t)
            self.execute_motion(0.0, w, turn_t)

        self.get_logger().info('等边三角形轨迹完成')

    #圆
    def execute_circle(self, radius_m: float):
        v = self.normal_v

        radius_m = max(radius_m, 0.1)

        # v = r * w => w = v / r
        w = v / radius_m

        # 一整圈的时间：2πr / v
        duration = 2.0 * math.pi * radius_m / v

        self.get_logger().info(
            f'开始圆形轨迹：半径={radius_m:.2f}m, v={v:.2f}, w={w:.2f}, '
            f'duration≈{duration:.2f}s'
        )

        self.execute_motion(v, w, duration)

        self.get_logger().info('圆形轨迹完成')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleVoiceTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
