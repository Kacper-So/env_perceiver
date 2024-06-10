from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the directory where the launch file is located
    pkg_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..')

    return LaunchDescription([
        Node(
            package='env_perceiver',
            executable='env_perceiver',
            name='env_perceiver',
            parameters=[os.path.join(pkg_dir, 'config', 'env_perceiver_params.yaml')]
        )
    ])
