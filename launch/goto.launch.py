from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mr_goto',
            executable='goto',
            #mode='demo',
            remappings=[
            ('scan', 'base_scan'),
            ]
        )
    ])