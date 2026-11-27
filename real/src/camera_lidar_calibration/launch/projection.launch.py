from pathlib import Path
import os

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    path = Path(os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config')) / 'camera-lidar-calibration/extrinsic.yaml'
    settings_file = path.parent / 'settings.yaml'
    selected = str(path)
    if settings_file.is_file():
        try:
            selected = (yaml.safe_load(settings_file.read_text()) or {}).get('extrinsic_file') or str(path)
        except (OSError, yaml.YAMLError):
            selected = str(path)
    return LaunchDescription([
        DeclareLaunchArgument('extrinsic_file', default_value=selected),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='camera_lidar_calibration', executable='lidar_projection_node', output='screen',
             parameters=[{'extrinsic_file': LaunchConfiguration('extrinsic_file'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
