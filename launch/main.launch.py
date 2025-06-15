# launch/launch_perception.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess



def generate_launch_description():
    package_name = "perception_winter"
    package_share_directory = get_package_share_directory('perception_winter')
    rviz_config_path = os.path.join(
        package_share_directory,
        'rviz_config',
        'config.rviz'
    )
    rosbag_path = os.path.join(
    	package_share_directory,
    	'rosbag',
    	'rosbag.db3'
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
            package=package_name,
            executable='process_lidar',
            output='screen',
        ),
        # Playing the rosbag
        ExecuteProcess(
	    cmd=[
		'ros2', 'bag', 'play', rosbag_path,
		'--loop',
	    ],
	    output='screen'
	)
    ])
