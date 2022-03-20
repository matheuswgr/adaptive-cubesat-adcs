#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_cubesat_gazebo = get_package_share_directory('cubesat_gazebo')
    pkg_cubesat_description = get_package_share_directory('cubesat_description')

    # Sart World
    start_world = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_cubesat_gazebo, 'launch', 'world.launch.py'),
        )
    )

    spawn_robot_world = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_cubesat_description, 'launch', 'cubesat_model.launch.py'),
        )
    )    
    
    imu_adapter = launch_ros.actions.Node(
            package='cubesat_sensors',
            node_executable='imu_adapter',
            name='imu_adapter',
            parameters=[{'use_sim_time': True}],
        )

    controller = launch_ros.actions.Node(
            package='cubesat_controllers',
            node_executable='controller_sd',
            name='controller_sd',
            parameters=[{'use_sim_time': True}],
        )

    concept = launch_ros.actions.Node(
            package='concept',
            node_executable='concept',
            name='concept',
            parameters=[{'use_sim_time': True}],
        )

   # bag_node = launch.actions.ExecuteProcess(
   #         cmd=['ros2', 'bag', 'record', '-a'],
   #         output='log'
   #     )

    return LaunchDescription([
        start_world,
        spawn_robot_world,
        imu_adapter,
        #controller,
        #concept,
        #bag_node
    ])