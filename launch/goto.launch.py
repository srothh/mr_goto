from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    goto_directory = get_package_share_directory('mr_goto')
    goto_directory_param = goto_directory + "/config/goto.yaml"

    return LaunchDescription([
        Node(
            package='mr_goto',
            executable='goto',
            #mode='demo',
            remappings=[
            ('scan', 'base_scan'),
            ],
            parameters=[
            goto_directory_param
            ],
        )
    ])