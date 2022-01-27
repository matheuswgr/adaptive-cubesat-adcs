#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
import launch
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

    return LaunchDescription([
        start_world,
        spawn_robot_world
    ])