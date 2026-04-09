from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    realsense_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch', 'rs_launch.py'
    )

    # Declare the audio mode argument — default is hybrid
    audio_mode_arg = DeclareLaunchArgument(
        'audio_mode',
        default_value='hybrid',
        description='Audio feedback mode: tones | speech | hybrid'
    )

    return LaunchDescription([
        audio_mode_arg,

        # RealSense — depth + RGB + aligned depth
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={
                'depth_module.depth_profile':  '640x480x30',
                'rgb_camera.color_profile':    '640x480x30',
                'enable_color':                'true',
                'enable_infra1':               'false',
                'enable_infra2':               'false',
                'pointcloud.enable':           'false',
                'align_depth.enable':          'true',
            }.items()
        ),

        # IMU node
        Node(
            package='master',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),

        # TF: base_link → imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        ),

        # TF: base_link → camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
        ),

        # Obstacle zone node
        Node(
            package='master',
            executable='obstacle_zone_node',
            name='obstacle_zone_node',
            output='screen'
        ),

        # YOLO detection node
        Node(
            package='master',
            executable='yolo_node',
            name='yolo_node',
            output='screen'
        ),

        # Fusion node
        Node(
            package='master',
            executable='fusion_node',
            name='fusion_node',
            output='screen'
        ),

        # Audio feedback node — mode passed as argument
        Node(
            package='master',
            executable='audio_feedback_node',
            name='audio_feedback_node',
            output='screen',
            parameters=[{
                'mode': LaunchConfiguration('audio_mode')
            }]
        ),
    ])
