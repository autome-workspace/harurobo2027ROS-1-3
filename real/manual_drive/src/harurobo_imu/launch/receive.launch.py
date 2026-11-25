from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', description='Explicit DM-IMU-L1 /dev/serial/by-id path'),
        Node(package='harurobo_imu', executable='imu_receiver', name='harurobo_imu',
             parameters=[{'port': LaunchConfiguration('port'), 'frame_id': 'imu_link'}],
             output='screen'),
    ])
