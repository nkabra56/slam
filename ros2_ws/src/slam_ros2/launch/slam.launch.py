import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('slam_ros2'), 'config', 'params.yaml'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to a params.yaml overriding slam_node defaults',
    )

    slam_node = Node(
        package='slam_ros2',
        executable='slam_node',
        name='slam_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('left/image_raw', '/camera/left/image_raw'),
            ('right/image_raw', '/camera/right/image_raw'),
            ('left/camera_info', '/camera/left/camera_info'),
            ('right/camera_info', '/camera/right/camera_info'),
            ('imu', '/imu/data'),
            ('points', '/velodyne_points'),
        ],
    )

    return LaunchDescription([params_file_arg, slam_node])
