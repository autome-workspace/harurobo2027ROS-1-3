"""Launch the existing approach nodes with this PC's measured geometry."""
import sys
from pathlib import Path

import yaml
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node



def description(path):
    settings = yaml.safe_load(Path(path).read_text())
    geometry = settings['geometry']
    parameters = settings['parameters_file']
    if geometry.get('lidar_orientation_rad') is None:
        raise ValueError('自動接近設定でLiDARの向きを設定してください。')
    nodes = [
        Node(package='ball_auto_approach', executable='ball_localizer',
             parameters=[parameters], output='screen'),
        Node(package='ball_auto_approach', executable='approach_controller',
             parameters=[parameters], remappings=[('/cmd_vel', '/control_center/approach_cmd_vel')],
             output='screen'),
    ]
    def mount(name, parent, child, xyz, rpy):
        args = []
        for flag, value in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), [*xyz, *rpy]):
            args.extend(['--' + flag, str(value)])
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher', name=name,
            arguments=[*args, '--frame-id', parent, '--child-frame-id', child], output='screen'))
    mount('control_center_laser_mount', 'base_link', 'laser',
          geometry['lidar_position_m'], geometry['lidar_orientation_rad'])
    mount('control_center_camera_mount', 'base_link', 'camera_link',
          geometry['camera_position_m'], [0., geometry['camera_pitch_rad'], 0.])
    mount('control_center_optical_mount', 'camera_link', 'camera_optical_frame',
          [0., 0., 0.], [-1.5707963267948966, 0., -1.5707963267948966])
    return LaunchDescription(nodes)


def main():
    service = LaunchService()
    service.include_launch_description(description(sys.argv[1]))
    return service.run()


if __name__ == '__main__':
    sys.exit(main())
