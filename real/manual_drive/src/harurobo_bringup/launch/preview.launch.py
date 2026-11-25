"""No CAN nodes or motor bench processes. Preview outputs must not drive hardware."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    teleop_config = str(Path(get_package_share_directory('harurobo_teleop')) / 'config' / 'stage_a.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('imu_port', description='Explicit DM-IMU-L1 by-id path'),
        Node(package='joy', executable='game_controller_node', name='game_controller',
             parameters=[teleop_config], output='screen'),
        Node(package='harurobo_teleop', executable='teleop_node', name='harurobo_teleop',
             parameters=[teleop_config], output='screen'),
        Node(package='harurobo_imu', executable='imu_receiver', name='harurobo_imu',
             parameters=[{'port': LaunchConfiguration('imu_port'), 'frame_id': 'imu_link'}],
             output='screen'),
        Node(package='harurobo_base', executable='heading_preview', name='harurobo_heading_preview',
             parameters=[{'kp': 0.5, 'kd': 0.0, 'max_correction_radps': 0.2}], output='screen'),
    ])
