# launch/launch_perception.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = "perception_winter"
    rviz_config_path = os.path.join(
        get_package_share_directory('perception_winter'),
        'rviz_config',
        'config.rviz'
    )
    return LaunchDescription([
        # Launch rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
        # Running the process_lidar node
        Node(
            package=package_name,        # Replace with your actual package name
            executable='process_lidar',        # Replace with your actual executable name
            output='screen',             # Optional: print output to terminal
        )
    ])
