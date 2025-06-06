# launch/launch_perception.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "perception_winter"
    return LaunchDescription([
        # Running the process_lidar node
        Node(
            package=package_name,        # Replace with your actual package name
            executable='process_lidar',        # Replace with your actual executable name
            output='screen',             # Optional: print output to terminal
        )
    ])
