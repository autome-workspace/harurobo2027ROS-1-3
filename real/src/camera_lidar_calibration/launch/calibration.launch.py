from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gui = Node(package='camera_lidar_calibration', executable='calibration_gui', output='screen')
    backend = Node(package='camera_lidar_calibration', executable='calibration_node', output='screen',
                   parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        backend, gui,
        RegisterEventHandler(OnProcessExit(target_action=gui, on_exit=[
            EmitEvent(event=Shutdown(reason='calibration GUI closed'))])),
        RegisterEventHandler(OnProcessExit(target_action=backend, on_exit=[
            EmitEvent(event=Shutdown(reason='calibration backend exited'))])),
    ])
