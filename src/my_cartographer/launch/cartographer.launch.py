import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cartographer_pkg_name = 'my_cartographer' 
    lua_config_name = 'my_robot_2d.lua'

    cartographer_share = get_package_share_directory(cartographer_pkg_name)
    cartographer_config_dir = os.path.join(cartographer_share, 'config')
    #Cartographer节点
    node_cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}], 
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', lua_config_name
        ]
    )

    #栅格地图转换节点
    node_occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': True, 
            'resolution': 0.05
        }]
    )

    #RViz2
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        node_cartographer,
        node_occupancy_grid,
        node_rviz
    ])