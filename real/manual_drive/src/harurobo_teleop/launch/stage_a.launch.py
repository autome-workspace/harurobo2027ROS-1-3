from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = Path(get_package_share_directory('harurobo_teleop')) / 'config' / 'stage_a.yaml'
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(default_config)),
        Node(package='joy', executable='game_controller_node', name='game_controller',
             parameters=[config], output='screen'),
        Node(package='harurobo_teleop', executable='teleop_node', name='harurobo_teleop',
             parameters=[config], output='screen'),
    ])
