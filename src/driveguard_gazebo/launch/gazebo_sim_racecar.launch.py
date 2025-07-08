#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    robot_name_in_urdf = 'racecar'
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock if true"
    )
    urdf_path = os.path.join(
        get_package_share_directory("driveguard_gazebo"), "urdf", "racecar/racecar.xacro"
    )
    gazebo_world_path = os.path.join(
        get_package_share_directory("driveguard_gazebo"), "worlds", "room/custom_room.world"
    )
    
    robot_joint_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen"
    )

    robot_joint_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "robot_description": xacro.process_file(urdf_path).toxml(),
            }
        ],
    )

    # Gazebo launch
    # Enter below if you encounter this error "shared_ptr assertion error"
    # source /usr/share/gazebo/setup.sh
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
                "/gazebo.launch.py",
            ]
        ),
        launch_arguments={"world": gazebo_world_path}.items(),
    )

    gazebo_spawn_entity_node = Node(
        name="urdf_spawner",
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "/robot_description", "-entity", robot_name_in_urdf],
        output="screen",
    )


    return LaunchDescription(
        [
            use_sim_time_arg,

            gazebo_node,
            gazebo_spawn_entity_node,

            robot_state_publisher_node,
            # robot_joint_publisher_node,
            # robot_joint_publisher_gui_node
        ]
    )

