from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = os.path.join(
        get_package_share_directory('fractal_terrain'))
    param_file = os.path.join(pkg_share, 'config', 'fractal_terrain_params.yaml')

    return LaunchDescription([
        Node(
            package='fractal_terrain',
            executable='fractal_terrain',
            name='fractal_terrain',
            parameters=[param_file],
            output='screen'
        )
    ])
