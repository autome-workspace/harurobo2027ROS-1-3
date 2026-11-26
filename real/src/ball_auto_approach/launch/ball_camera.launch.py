from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('ball_auto_approach')) / 'config/params.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(config)),
        DeclareLaunchArgument('view', default_value='true'),
        Node(package='ball_auto_approach', executable='usb_camera',
             parameters=[LaunchConfiguration('config')], output='screen'),
        Node(package='ball_auto_approach', executable='ball_detector',
             parameters=[LaunchConfiguration('config'), {'debug': True}],
             output='screen'),
        Node(package='rqt_image_view', executable='rqt_image_view',
             arguments=['/ball/debug_image'], condition=IfCondition(LaunchConfiguration('view')),
             output='screen'),
    ])
