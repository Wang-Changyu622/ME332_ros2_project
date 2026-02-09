from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        #Vosk 语音识别节点
        Node(
            package='voice_asr_vosk',   
            executable='asr_vosk_node',   
            name='asr_vosk_node',
            output='screen',
        ),

        #simple_voice_teleop 节点 订阅/voice_cmd_text，发布/cmd_vel
            Node(
            package='simple_voice_teleop',
            executable='simple_voice_teleop',  
            name='simple_voice_teleop',
            output='screen',
        ),
    ])
