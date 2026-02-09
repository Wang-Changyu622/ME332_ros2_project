#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from tf2_ros import TransformListener, Buffer
from rclpy.executors import MultiThreadedExecutor
import sys
import time
import threading
import math
import cv2
import mediapipe as mp
# 姿态 1:归位/Home
POSE_1 = [
    math.radians(0),   
    math.radians(0),   
    math.radians(0),  
    math.radians(0),   
    math.radians(0), 
    math.radians(0)   
]

# 姿态 2:准备/Ready
POSE_2 = [
    math.radians(0),   
    math.radians(-30), 
    math.radians(-60), 
    math.radians(0),  
    math.radians(-30), 
    math.radians(0)    
]

# 姿态 3: 放置/Place
POSE_3 = [
    math.radians(175), 
    math.radians(-41), 
    math.radians(-96),
    math.radians(7),   
    math.radians(-44), 
    math.radians(-9)
]

GRIPPER_CLOSE = [-0.015, 0.015] 
GRIPPER_OPEN  = [0.0, 0.0]

PLANNING_GROUP = "arm"
GRIPPER_GROUP = "gripper"
EE_LINK = "arm_link_6"
BASE_LINK = "base_link"

class PoseController(Node):
    def __init__(self):
        super().__init__('pose_controller', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')

        try:
            from linkattacher_msgs.srv import AttachLink, DetachLink
            self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
            self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
            self.has_attacher = True
        except ImportError:
            self.has_attacher = False

    def send_joint_goal(self, group_name, target_joints):
        goal = MoveGroup.Goal()
        goal.request.group_name = group_name
        goal.request.max_velocity_scaling_factor = 1.0
        
        c = Constraints()
        joint_names = [f'arm_joint_{i+1}' for i in range(6)]
        
        for j_name, j_val in zip(joint_names, target_joints):
            jc = JointConstraint()
            jc.joint_name = j_name
            jc.position = j_val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
            
        goal.request.goal_constraints.append(c)
        self.move_group_client.send_goal_async(goal)

    def control_gripper(self, close=True):
        """控制夹爪"""
        goal = MoveGroup.Goal()
        goal.request.group_name = GRIPPER_GROUP
        c = Constraints()
        joints = ['finger_joint_1', 'finger_joint_2']
        values = GRIPPER_CLOSE if close else GRIPPER_OPEN
        
        for j, v in zip(joints, values):
            jc = JointConstraint()
            jc.joint_name = j
            jc.position = v
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
            
        goal.request.goal_constraints.append(c)
        self.move_group_client.send_goal_async(goal)
        
        # 处理吸附
        if self.has_attacher:
            req = AttachLink.Request() if close else DetachLink.Request()
            req.model1_name = 'my_mobile_manipulator'
            req.link1_name = EE_LINK 
            req.model2_name = 'target_cube' 
            req.link2_name = 'link'
            client = self.attach_client if close else self.detach_client
            if client.service_is_ready():
                client.call_async(req)


class GestureApp:
    def __init__(self, ros_node):
        self.node = ros_node
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5)
        self.mp_draw = mp.solutions.drawing_utils
        self.cap = cv2.VideoCapture(0)
        
        self.last_cmd_time = 0
        self.cooldown = 2.0 

    def count_fingers(self, lm):
        fingers = []
        if lm[4].x > lm[3].x: fingers.append(1)
        else: fingers.append(0)
        tips = [8, 12, 16, 20]
        for tip in tips:
            if lm[tip].y < lm[tip - 2].y: fingers.append(1)
            else: fingers.append(0)
        return fingers

    def run(self):
        print("=== 手势控制已启动 ===")
        print("(1指): 姿态1 - 归位")
        print("(2指): 姿态2 - 准备")
        print("(3指): 姿态3 - 放置")
        print("(握拳): 抓取 (Grasp)")
        print("(张手): 松开 (Release)")

        while self.cap.isOpened() and rclpy.ok():
            success, img = self.cap.read()
            if not success: continue

            img = cv2.flip(img, 1)
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            results = self.hands.process(img_rgb)
            
            status_text = "Waiting..."
            current_time = time.time()

            if results.multi_hand_landmarks:
                for hand_lms in results.multi_hand_landmarks:
                    self.mp_draw.draw_landmarks(img, hand_lms, self.mp_hands.HAND_CONNECTIONS)
                    
                    lm_list = hand_lms.landmark
                    fingers = self.count_fingers(lm_list)
                    total_fingers = fingers.count(1)

                    if current_time - self.last_cmd_time > self.cooldown:
                        
                        if total_fingers == 1:
                            print(">>> 执行姿态 1 (Home)")
                            status_text = "POSE 1: HOME"
                            self.node.send_joint_goal(PLANNING_GROUP, POSE_1)
                            self.last_cmd_time = current_time
                            
                        elif total_fingers == 2:
                            print(">>> 执行姿态 2 (Ready)")
                            status_text = "POSE 2: READY"
                            self.node.send_joint_goal(PLANNING_GROUP, POSE_2)
                            self.last_cmd_time = current_time
                            
                        elif total_fingers == 3:
                            print(">>> 执行姿态 3 (Place)")
                            status_text = "POSE 3: PLACE"
                            self.node.send_joint_goal(PLANNING_GROUP, POSE_3)
                            self.last_cmd_time = current_time

                        elif total_fingers == 0: # 握拳
                            print(">>> 抓取")
                            status_text = "GRASP"
                            self.node.control_gripper(close=True)
                            self.last_cmd_time = current_time
                            
                        elif total_fingers == 5: # 张手
                            print(">>> 松开")
                            status_text = "RELEASE"
                            self.node.control_gripper(close=False)
                            self.last_cmd_time = current_time

            cv2.rectangle(img, (0,0), (640, 60), (0,0,0), -1)
            cv2.putText(img, status_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Gesture Pose Control", img)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rclpy.init()
    node = PoseController()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    try:
        app = GestureApp(node)
        app.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
