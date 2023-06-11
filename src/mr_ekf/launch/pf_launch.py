#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    this_directory = get_package_share_directory('mr_pf')

    remappings = [('/scan', 'base_scan')]

    particle_filter_level_arg = DeclareLaunchArgument(
        'level',
        default_value=TextSubstitution(text='10'),
        description='Level for teachers')

    # ToDo make it work with only the filename independet to the path from where it will be launched!
    # Hint: you have to use an OpaqueFunction
    particle_filter_params_arg = DeclareLaunchArgument(
        'particle_filter_params_file',
        default_value=TextSubstitution(text='particle_filter.yaml'),
        description='ROS2 parameters')

    particle_filter_map_file_arg = DeclareLaunchArgument(
        'particle_filter_map_file',
        default_value=TextSubstitution(text='cave.png'),
        description='map image file')


    return LaunchDescription([
        particle_filter_level_arg,
        particle_filter_map_file_arg,
        particle_filter_params_arg,
        Node(
            package='mr_pf',
            executable='pf_node',
            name='pf',
            remappings=remappings,
            parameters=[LaunchConfiguration('particle_filter_params_file'),
                        {'alevel': LaunchConfiguration('level'),
                        'map_file': LaunchConfiguration('particle_filter_map_file')}]
        )
    ])
