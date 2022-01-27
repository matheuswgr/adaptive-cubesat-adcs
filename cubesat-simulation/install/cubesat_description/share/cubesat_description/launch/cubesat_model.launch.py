#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    urdf = os.path.join(get_package_share_directory('cubesat_description'), 'models/', 'cubesat.urdf')
    assert os.path.exists(urdf), "cubesat.urdf doesnt exist in "+str(urdf)

    ld = LaunchDescription([
        Node(package='cubesat_description', node_executable='spawn_cubesat.py', arguments=[urdf], output='screen'),
    ])
    return ld