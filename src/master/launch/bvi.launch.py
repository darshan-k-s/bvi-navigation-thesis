from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    realsense_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch', 'rs_launch.py'
    )
    return LaunchDescription([

        # RealSense — depth only, 30 FPS
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={
                'depth_module.depth_profile': '640x480x30',
                'enable_color':      'false',
                'enable_infra1':     'false',
                'enable_infra2':     'false',
                'pointcloud.enable': 'false',
            }.items()
        ),

        # IMU node
        Node(
            package='master',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),

	Node(
   	    package='tf2_ros',
    	    executable='static_transform_publisher',
    	    name='imu_tf',
   	    arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
	),


        # Obstacle zone node
        Node(
            package='master',
            executable='obstacle_zone_node',
            name='obstacle_zone_node',
            output='screen'
        ),
    ])

