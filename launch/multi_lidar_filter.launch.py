from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('lidar_preprocessor'),
        'config',
        'multi_lidar_filter.yaml'
    )

    return LaunchDescription([
        Node(
            package='lidar_preprocessor',
            executable='multi_lidar_filter_node',
            name='multi_lidar_filter_node',
            output='screen',
            parameters=[config_path]
        )
    ])
