#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    stage_directory = get_package_share_directory('stage_ros2')
    ekf_directory = get_package_share_directory('mr_ekf')
    pf_directory = get_package_share_directory('mr_pf')
    goto_directory = get_package_share_directory('mr_goto')

    world_name = "cave"

    goto_directory_param = goto_directory + "/config/goto.yaml"
    pf_directory_param = pf_directory + "/config/particle_filter.yaml"

    ekf_directory_png = ekf_directory + "/config/maps/" + world_name + ".png"
    ekf_directory_yaml = ekf_directory + "/config/maps/" + world_name + ".yml"
    
    #world_stage_directory = stage_directory + "/world/" + world_name + ".world"

    print(goto_directory_param)

    stage = Node( #stage
            package='stage_ros2',
            executable='stage_ros2',
            name='stage',
            parameters=[
                {"world_file": [LaunchConfiguration('world_file')]}
            ],
    )

    laser = Node( #laser
        package='tuw_laserscan_features',
        executable='composed_node',
        remappings=[
            ('scan', 'base_scan'),
        ],
    )

    ekf = Node( #ekf
        package='mr_ekf',
        executable='ekf_node',
        remappings=[
            ('scan', 'base_scan'),
        ],
        parameters=[
            {"map_file": ekf_directory_png},
            {"map_linesegments_file": ekf_directory_yaml}
        ],
    )
    
    pf = Node( #pf
        package='mr_pf',
        executable='pf_node',
        remappings=[
            ('scan', 'base_scan'),
        ],
        parameters=[
            #pf_directory_param
            LaunchConfiguration('parameter_pf_file')
        ],
    )

    move = Node( #move
        package='mr_move',
        executable='move',
        remappings=[
            ('scan', 'base_scan'),
        ],
        parameters=[
            {"mode": "wanderer"},
        ],
    )

    goto = Node( #goto
        package='mr_goto',
        executable='goto',
        remappings=[
            ('scan', 'base_scan'),
        ],
        parameters=[
            LaunchConfiguration('parameter_goto_file')
            #{"mode": "plan"},
            #{"x": -5.0},
            #{"y": -6.0},
        ],
    )

    # stage
    stage_world_arg = DeclareLaunchArgument(
        'world',
        default_value=TextSubstitution(text='cave'),
        description='World file relative to the project world file, without .world')

    def stage_world_configuration(context):
        file = os.path.join(
            stage_directory,
            'world',
            context.launch_configurations['world'] + '.world')
        return [SetLaunchConfiguration('world_file', file)]
    
    stage_world_configuration_arg = OpaqueFunction(function=stage_world_configuration)

    ld.add_action(stage_world_arg)
    ld.add_action(stage_world_configuration_arg)

    # parameter pf
    parameter_pf_arg = DeclareLaunchArgument(
        'parameter_pf',
        default_value=TextSubstitution(text='particle_filter'),
        description='Parameter file relative to the project file, without .yaml')

    def parameter_pf_configuration(context):
        file = os.path.join(
            pf_directory,
            'config',
            context.launch_configurations['parameter_pf'] + '.yaml')
        #print("loaded", file)
        return [SetLaunchConfiguration('parameter_pf_file', file)]
    
    parameter_pf_configuration_arg = OpaqueFunction(function=parameter_pf_configuration)

    ld.add_action(parameter_pf_arg)
    ld.add_action(parameter_pf_configuration_arg)

    # parameter goto
    parameter_goto_arg = DeclareLaunchArgument(
        'parameter_goto',
        default_value=TextSubstitution(text='goto'),
        description='Parameter file relative to the project file, without .yaml')

    def parameter_goto_configuration(context):
        file = os.path.join(
            goto_directory,
            'config',
            context.launch_configurations['parameter_goto'] + '.yaml')
        #print("loaded", file)
        return [SetLaunchConfiguration('parameter_goto_file', file)]
    
    parameter_goto_configuration_arg = OpaqueFunction(function=parameter_goto_configuration)

    ld.add_action(parameter_goto_arg)
    ld.add_action(parameter_goto_configuration_arg)

    # ekf or pf
    localization = DeclareLaunchArgument(
        name='localization',
        default_value='ekf',
        description='Localization method (ekf or pf)'
    )

    def launch_node(context):
        local = context.launch_configurations['localization']
        if local == 'ekf':
            ld.add_action(ekf)
        elif local == 'pf':
            ld.add_action(pf)

    pf_ekf_chooser_arg = OpaqueFunction(function=launch_node)

    ld.add_action(localization)
    ld.add_action(pf_ekf_chooser_arg)

    # goto or move
    planner = DeclareLaunchArgument(
        name='planner',
        default_value='goto',
        description='Planner method (move or goto)'
    )

    def launch_node(context):
        local = context.launch_configurations['planner']
        if local == 'move':
            ld.add_action(move)
        elif local == 'goto':
            ld.add_action(goto)

    planner_chooser_arg = OpaqueFunction(function=launch_node)

    ld.add_action(planner)
    ld.add_action(planner_chooser_arg)

    # nodes which will start for sure

    ld.add_action(stage)

    ld.add_action(laser)

    return ld

#ros2 run mr_ekf ekf_node --ros-args -r scan:=base_scan -p map_file:=$MR_DIR/ws02/install/mr_ekf/share/mr_ekf/config/maps/line.png -p map_linesegments_file:=$MR_DIR/ws02/install/mr_ekf/share/mr_ekf/config/maps/line.yml