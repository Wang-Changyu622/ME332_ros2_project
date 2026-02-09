from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    mp_pkg_share = get_package_share_directory('mediapipe_ros2_py')
    mp_launch = os.path.join(mp_pkg_share, 'launch', 'mp_node.launch.py')

    mp_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mp_launch),
        launch_arguments={
            'model': 'hand',
            'image_topic': '/image_raw',
            'start_rviz': 'true',
        }.items(),
    )

    return LaunchDescription([
        #摄像头节点：发布/image_raw
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            output='screen',
        ),

        #Mediapipe手部关键点节点
        mp_node,

        #手势控制节点 订阅手势话题，发/cmd_vel
        Node(
            package='tb3_gesture_teleop',
            executable='tb3_gesture_teleop',  
            name='tb3_gesture_teleop',
            output='screen',
        ),
    ])
