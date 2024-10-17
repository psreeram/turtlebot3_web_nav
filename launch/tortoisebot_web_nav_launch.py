from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ros2_bridge_node = Node(
        package='turtlebot3_web_nav',
        executable='ros2_bridge',
        name='ros2_bridge'
    )

    return LaunchDescription([
        ros2_bridge_node
    ])