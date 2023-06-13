from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    # Path to the XML launch file
    xml_launch_file = "/home/rita/talker_launch.xml"

    return LaunchDescription([
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([xml_launch_file])
        )
    ])

if __name__ == '__main__':
    generate_launch_description()

