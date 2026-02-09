from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0', 
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }]
    )

    teleop_node = Node(
        package='tb3_joy_teleop',
        executable='joy_to_twist',
        name='tb3_joy_teleop',
        parameters=[{
            'linear_scale': 0.2,
            'angular_scale': 0.8,
        }]
    )

    return LaunchDescription([joy_node, teleop_node])
