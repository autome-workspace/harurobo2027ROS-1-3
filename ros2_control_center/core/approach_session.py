"""Launch the existing approach nodes with this PC's measured geometry."""
import sys
from pathlib import Path

import yaml
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core.approach_config import apply_calibration



def description(path):
    settings = yaml.safe_load(Path(path).read_text())
    geometry = settings['geometry']
    parameters = settings['parameters_file']
    if geometry.get('lidar_orientation_rad') is None:
        raise ValueError('自動接近設定でLiDARの向きを設定してください。')
    extrinsic = settings.get('extrinsic_file')
    calibrated = yaml.safe_load(Path(parameters).read_text()) if extrinsic else None
    if extrinsic:
        data = apply_calibration(calibrated, geometry, extrinsic)
    detection_remap = [('/ball/detection', '/camera_lidar/ball_detection')] if extrinsic else []
    nodes = [
        Node(package='ball_auto_approach', executable='ball_localizer',
             parameters=[calibrated['ball_localizer']['ros__parameters'] if extrinsic else parameters],
             remappings=detection_remap + ([('/scan', data['settings']['scan_topic'])] if extrinsic else []),
             output='screen'),
        Node(package='ball_auto_approach', executable='approach_controller',
             parameters=[calibrated['approach_controller']['ros__parameters'] if extrinsic else parameters],
             remappings=[('/cmd_vel', '/control_center/approach_cmd_vel'), *detection_remap],
             output='screen'),
    ]
    def mount(name, parent, child, xyz, rpy):
        args = []
        for flag, value in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), [*xyz, *rpy]):
            args.extend(['--' + flag, str(value)])
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher', name=name,
            arguments=[*args, '--frame-id', parent, '--child-frame-id', child], output='screen'))
    mount('control_center_laser_mount', 'base_link', data['lidar_frame'] if extrinsic else 'laser',
          geometry['lidar_position_m'], geometry['lidar_orientation_rad'])
    if extrinsic:
        nodes.append(Node(package='camera_lidar_calibration', executable='approach_detector',
                          parameters=[{'extrinsic_file': str(extrinsic)}], output='screen'))
    else:
        nodes.append(Node(package='ball_auto_approach', executable='ball_detector',
                          parameters=[parameters], output='screen'))
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
