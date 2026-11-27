from pathlib import Path
from uuid import uuid4

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
    IncludeLaunchDescription, SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def sensor_transform(frame, x, z):
    return Node(
        package='tf2_ros', executable='static_transform_publisher',
        name=f'{frame}_transform',
        arguments=['--x', x, '--z', z, '--frame-id', 'base_link', '--child-frame-id', frame],
    )


def generate_launch_description():
    share = Path(get_package_share_directory('harurobo_sim'))
    world = PathJoinSubstitution([str(share / 'worlds'), LaunchConfiguration('field')])
    headless = LaunchConfiguration('headless')
    slam = PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])
    teleop = PathJoinSubstitution([FindPackageShare('harurobo_teleop'), 'launch', 'stage_a.launch.py'])
    return LaunchDescription([
        DeclareLaunchArgument('field', default_value='kanto_2027.sdf',
                              choices=['kanto_2027.sdf', 'training.sdf']),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('teleop', default_value='false'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '87'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(share / 'models')),
        SetEnvironmentVariable('GZ_PARTITION', f'harurobo_sim_{uuid4().hex}'),
        ExecuteProcess(cmd=['gz', 'sim', '-r', world], condition=UnlessCondition(headless)),
        ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', '--headless-rendering', world],
                       condition=IfCondition(headless)),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(share / 'config' / 'bridge.yaml')}]),
        Node(package='harurobo_sim', executable='sim_velocity.py'),
        sensor_transform('laser', '0.19', '0.065'),
        sensor_transform('imu_link', '0', '0.12'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam),
            condition=IfCondition(LaunchConfiguration('slam')),
            launch_arguments={
                'use_sim_time': 'true',
                'slam_params_file': str(share / 'config' / 'slam.yaml'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop),
            condition=IfCondition(LaunchConfiguration('teleop')),
        ),
    ])
