#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )

    declare_gui = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Launch Gazebo client (gzclient)",
    )

    pkg_robot = get_package_share_directory("final_assignment_pkg")
    pkg_moveit = get_package_share_directory("final_assignment_moveit_config")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    world_file = os.path.join(pkg_robot, "worlds", "test.sdf")
    xacro_file = os.path.join(pkg_robot, "urdf", "robot.urdf.xacro")

    gazebo_model_path_add = os.path.join(pkg_robot, "worlds")
    existing_gmp = os.environ.get("GAZEBO_MODEL_PATH", "")
    merged_gmp = gazebo_model_path_add if existing_gmp == "" else (gazebo_model_path_add + ":" + existing_gmp)

    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=merged_gmp,
    )

    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_desc,
            "use_sim_time": use_sim_time,
        }],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")),
        launch_arguments={
            "world": world_file,
            "verbose": "true",
            "gui": gui,
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-entity", "my_mobile_manipulator",
            "-z", "0.05",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["arm_controller"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["gripper_controller"],
    )

    after_spawn_load_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    after_jsb_load_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, gripper_controller_spawner],
        )
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_moveit, "launch", "move_group.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_moveit, "launch", "moveit_rviz.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    after_arm_controller_start_moveit = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[move_group_launch, rviz_launch],
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,
        set_gazebo_model_path,

        gazebo,

        robot_state_publisher,
        spawn_entity,

        after_spawn_load_jsb,
        after_jsb_load_controllers,
        after_arm_controller_start_moveit,
    ])
