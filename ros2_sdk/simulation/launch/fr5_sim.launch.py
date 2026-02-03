#!/usr/bin/env python3
"""
FR5 PyBullet Simulation Launch File

This launch file starts:
- FR5 PyBullet simulation node with RRT-Connect path planning
- Robot state publisher for TF broadcasting

Usage:
    ros2 launch fr5_pybullet_sim fr5_sim.launch.py
    ros2 launch fr5_pybullet_sim fr5_sim.launch.py gui:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Declare launch arguments
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Enable PyBullet GUI visualization'
    )

    urdf_name_arg = DeclareLaunchArgument(
        'urdf_name',
        default_value='fairino5_v6.urdf',
        description='URDF file name from fairino_description package'
    )

    # FR5 PyBullet Simulation Node
    sim_node = Node(
        package='fr5_pybullet_sim',
        executable='sim_node',
        name='fr5_pybullet_sim',
        parameters=[{
            'gui': LaunchConfiguration('gui'),
            'urdf_name': LaunchConfiguration('urdf_name'),
            'sim_rate': 120.0,
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        gui_arg,
        urdf_name_arg,
        sim_node,
    ])
