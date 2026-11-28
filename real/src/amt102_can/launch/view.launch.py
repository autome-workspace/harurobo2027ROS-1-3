from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    config=Path(get_package_share_directory('amt102_can'))/'config/encoders.rviz'
    visualizer=Node(package='amt102_can',executable='encoder_visualizer',output='screen')
    rviz=Node(package='rviz2',executable='rviz2',name='encoders_rviz',arguments=['-d',str(config)],
              additional_env={'QT_QPA_PLATFORM':'xcb','LIBGL_ALWAYS_SOFTWARE':'1'},output='screen')
    stop=RegisterEventHandler(OnProcessExit(on_exit=[EmitEvent(event=Shutdown(reason='エンコーダー表示を終了しました。'))]))
    return LaunchDescription([stop,visualizer,rviz])
