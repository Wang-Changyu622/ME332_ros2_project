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
import sys, select, termios, tty
import time
import threading
import math

PLACE_JOINTS = [
    math.radians(175),
    math.radians(-41),
    math.radians(-96),
    math.radians(7),
    math.radians(-44),
    math.radians(-9)
]

GRIPPER_CLOSE = [-0.015, 0.015] 
GRIPPER_OPEN  = [0.0, 0.0]

EE_LINK = "arm_link_6"
BASE_LINK = "base_link"
PLANNING_GROUP = "arm"
GRIPPER_GROUP = "gripper"
STEP = 0.015 

try:
    from linkattacher_msgs.srv import AttachLink, DetachLink
except ImportError:
    pass

msg = """
===================================================
    🤖 Grasping Assistant
===================================================
   [W/S] X axis      [I/K] Z axis
   [A/D] Y axis

   [G]   Grasp 
   [P]   Place 
   
   [Q]   Quit
===================================================
"""

class GraspController(Node):
    def __init__(self):
        super().__init__('grasp_controller', 
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        
    def wait_for_initial_tf(self):
        while rclpy.ok():
            try:
                self.tf_buffer.lookup_transform(BASE_LINK, EE_LINK, rclpy.time.Time())
                return
            except Exception:
                time.sleep(1.0)

    def get_current_pose(self):
        try:
            return self.tf_buffer.lookup_transform(BASE_LINK, EE_LINK, rclpy.time.Time())
        except Exception:
            return None

    def send_joint_goal(self, group, joints, values):
        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.max_velocity_scaling_factor = 1.0
        
        c = Constraints()
        for j, v in zip(joints, values):
            jc = JointConstraint()
            jc.joint_name = j
            jc.position = v
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        goal.request.goal_constraints.append(c)
        
        future = self.move_group_client.send_goal_async(goal)
        while not future.done() and rclpy.ok():
            time.sleep(0.05)

    def control_attach(self, attach=True):
        req = AttachLink.Request() if attach else DetachLink.Request()
        req.model1_name = 'my_mobile_manipulator'
        req.link1_name = EE_LINK 
        req.model2_name = 'target_cube' 
        req.link2_name = 'link'
        
        client = self.attach_client if attach else self.detach_client
        if client.service_is_ready():
            client.call_async(req)

    def execute_grasp_sequence(self):
        self.control_attach(True)
        self.send_joint_goal(GRIPPER_GROUP, ['finger_joint_1', 'finger_joint_2'], GRIPPER_CLOSE)

    def execute_place_sequence(self):
        joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 
                       'arm_joint_4', 'arm_joint_5', 'arm_joint_6']
        self.send_joint_goal(PLANNING_GROUP, joint_names, PLACE_JOINTS)
        time.sleep(1.0)
        self.send_joint_goal(GRIPPER_GROUP, ['finger_joint_1', 'finger_joint_2'], GRIPPER_OPEN)
        self.control_attach(False)

    def move_relative(self, dx, dy, dz):
        from moveit_msgs.msg import PositionConstraint, OrientationConstraint
        from geometry_msgs.msg import PoseStamped
        from shape_msgs.msg import SolidPrimitive
        
        current = self.get_current_pose()
        if not current: return

        goal = MoveGroup.Goal()
        goal.request.group_name = PLANNING_GROUP
        
        c = Constraints()
        pcm = PositionConstraint()
        pcm.header.frame_id = BASE_LINK
        pcm.link_name = EE_LINK
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]
        pcm.constraint_region.primitives.append(box)
        
        target = PoseStamped()
        target.header.frame_id = BASE_LINK
        target.pose.position.x = current.transform.translation.x + dx
        target.pose.position.y = current.transform.translation.y + dy
        target.pose.position.z = current.transform.translation.z + dz
        target.pose.orientation = current.transform.rotation
        
        pcm.constraint_region.primitive_poses.append(target.pose)
        pcm.weight = 1.0
        c.position_constraints.append(pcm)
        
        ocm = OrientationConstraint()
        ocm.header.frame_id = BASE_LINK
        ocm.link_name = EE_LINK
        ocm.orientation = target.pose.orientation
        ocm.absolute_x_axis_tolerance = 0.1
        ocm.absolute_y_axis_tolerance = 0.1
        ocm.absolute_z_axis_tolerance = 0.1
        ocm.weight = 1.0
        c.orientation_constraints.append(ocm)
        
        goal.request.goal_constraints.append(c)
        self.move_group_client.send_goal_async(goal)


def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    
    node = GraspController()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    node.wait_for_initial_tf()
    
    print(msg)
    
    try:
        while rclpy.ok():
            key = get_key()
            if key == 'q': break
            
            if key == 'w':   node.move_relative(STEP, 0.0, 0.0)
            elif key == 's': node.move_relative(-STEP, 0.0, 0.0)
            elif key == 'a': node.move_relative(0.0, STEP, 0.0)
            elif key == 'd': node.move_relative(0.0, -STEP, 0.0)
            elif key == 'i': node.move_relative(0.0, 0.0, STEP)
            elif key == 'k': node.move_relative(0.0, 0.0, -STEP)
            elif key == 'g':
                node.execute_grasp_sequence()
            elif key == 'p':
                node.execute_place_sequence()
                
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
