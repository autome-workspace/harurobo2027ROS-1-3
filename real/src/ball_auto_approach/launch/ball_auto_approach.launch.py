from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('ball_auto_approach'))
    config = LaunchConfiguration('config')
    root = LaunchConfiguration('project_root')
    powered = LaunchConfiguration('powered')
    output_topic = PythonExpression(["'/cmd_vel' if '", powered, "'.lower() == 'true' else '/cmd_vel_preview'"])
    declarations = [
        DeclareLaunchArgument(name, default_value=value, description=description)
        for name, value, description in (
            ('config', str(share / 'config/params.yaml'), 'Node parameter YAML'),
            ('project_root', str(Path.home() / 'DM_IMU_ros2'), 'Source checkout for existing LD03 and CAN scripts'),
            ('camera', 'true', 'Start the USB camera; false for external image/CameraInfo'),
            ('detector', 'true', 'Start ball detection; false uses external /ball/detection'),
            ('joy', 'true', 'Start the SDL controller driver'),
            ('lidar', 'false', 'Start the existing LD03 driver; false uses external /scan'),
            ('lidar_port', '/dev/ttyACM0', 'Serial device for the LD03 driver'),
            ('lidar_clockwise', 'false', 'Flip the LD03 scan angles'),
            ('powered', 'false', 'Start the existing CAN driver and watchdog when the operator is ready'),
            ('rviz', 'false', 'Start RViz with the ball and scan displays'),
            ('publish_tf', 'false', 'Publish static transforms using the measured mount arguments'),
            ('laser_xyz', '0.0 0.0 0.05', 'Laser base-relative xyz in metres'),
            ('laser_rpy', '0.0 0.0 0.0', 'Laser base-relative roll pitch yaw in radians'),
            ('camera_xyz', '0.0 0.0 0.10', 'Camera base-relative xyz in metres'),
            ('camera_rpy', '0.0 0.0 0.0', 'Camera body roll pitch yaw in radians'),
        )
    ]

    def mount(child, name):
        arguments = []
        for flag, group, axis in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'),
                                     ('xyz', 'xyz', 'xyz', 'rpy', 'rpy', 'rpy'), (0, 1, 2, 0, 1, 2)):
            arguments.extend(['--' + flag, PythonExpression(
                ["'", LaunchConfiguration(name + '_' + group), f"'.split()[{axis}]"])])
        return Node(package='tf2_ros', executable='static_transform_publisher',
                    condition=IfCondition(LaunchConfiguration('publish_tf')),
                    arguments=[*arguments, '--frame-id', 'base_link', '--child-frame-id', child])

    scripts = '/real/manual_drive/src/harurobo_base/test/'
    watchdog = ExecuteProcess(cmd=['/usr/bin/python3', [root, scripts, 'watchdog_stop.py']],
                              condition=IfCondition(powered), output='screen')
    base = ExecuteProcess(cmd=['/usr/bin/python3', [root, scripts, 'manual_drive.py'],
                               '--execute-powered-robot-estop-ready', '--twist'],
                          condition=IfCondition(powered), output='screen')
    controller = Node(package='ball_auto_approach', executable='approach_controller',
                      parameters=[config], remappings=[('/cmd_vel', output_topic)], output='screen')
    # A dead CAN process must leave the watchdog running to issue its stop bursts.
    # Node-level freshness and the
    # independent watchdog also cover ungraceful exits and callback stalls.
    critical = [RegisterEventHandler(OnProcessExit(
        target_action=action, on_exit=[EmitEvent(event=Shutdown(reason='critical process exited'))]))
        for action in (watchdog, controller)]
    # The watchdog's exclusive UDP bind also prevents a second supervised
    # driver from opening CAN while another session owns the watchdog port.
    start_base = RegisterEventHandler(OnProcessIO(
        target_action=watchdog,
        on_stdout=lambda event: [base] if b'waiting for the first heartbeat' in event.text else []))
    return LaunchDescription([
        *declarations, *critical, start_base,
        Node(package='ball_auto_approach', executable='usb_camera', parameters=[config],
             condition=IfCondition(LaunchConfiguration('camera')), output='screen'),
        Node(package='joy', executable='game_controller_node', name='game_controller', parameters=[config],
             condition=IfCondition(LaunchConfiguration('joy')), output='screen'),
        ExecuteProcess(cmd=['/usr/bin/python3', [root, '/real/lidar/ld03_node.py'], '--ros-args',
                            '-p', ['port:=', LaunchConfiguration('lidar_port')],
                            '-p', ['clockwise:=', LaunchConfiguration('lidar_clockwise')]],
                       condition=IfCondition(LaunchConfiguration('lidar')), output='screen'),
        mount('laser', 'laser'), mount('camera_link', 'camera'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             condition=IfCondition(LaunchConfiguration('publish_tf')),
             arguments=['--roll', '-1.57079632679', '--yaw', '-1.57079632679',
                        '--frame-id', 'camera_link', '--child-frame-id', 'camera_optical_frame']),
        Node(package='ball_auto_approach', executable='ball_detector', parameters=[config],
             condition=IfCondition(LaunchConfiguration('detector')), output='screen'),
        Node(package='ball_auto_approach', executable='ball_localizer',
             parameters=[config], output='screen'),
        controller, watchdog,
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share / 'config/ball.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
