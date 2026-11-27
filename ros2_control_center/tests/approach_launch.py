"""Validate saved geometry and the preview launch with synthetic scans and no motors."""
import os
from pathlib import Path
import sys
import tempfile
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PySide6.QtWidgets import QApplication
import yaml
import rclpy
from sensor_msgs.msg import LaserScan

from core.config_manager import Config
from core.controller import Controller
from ui.approach_settings import ApproachSettingsDialog

app = QApplication([])
with tempfile.TemporaryDirectory() as directory:
    path = Path(directory) / 'local.yaml'
    # Real measured dimensions, with a synthetic orientation for this isolated test.
    path.write_text(yaml.safe_dump({
        'variables': {'CAMERA_CALIBRATION': 'unused-camera-file-for-launch-test'},
        'robot_geometry': {
            'lidar_position_m': [.18, 0., .08], 'lidar_orientation_rad': None,
            'camera_position_m': [.18, 0., .3], 'camera_pitch_rad': .436332313,
            'ball_diameter_m': .06, 'ball_center_height_m': .065,
            'body_length_m': .5, 'body_width_m': .5,
        },
    }))
    config = Config(path)
    config.domains['real'] = 94
    controller = Controller(config)
    controller.mode = 'real'
    errors = []
    controller.log.connect(lambda key, level, line: errors.append((key, level, line)))
    dialog = ApproachSettingsDialog(controller)
    dialog.yaw.setValue(0)
    dialog.save()
    settings = yaml.safe_load(Path(config.variables['APPROACH_SETTINGS']).read_text())
    parameters = yaml.safe_load(Path(settings['parameters_file']).read_text())
    assert parameters['ball_localizer']['ros__parameters']['ball_center_z'] == .065
    assert parameters['approach_controller']['ros__parameters']['half_length'] == .25
    os.environ.update(config.environment('real'))
    rclpy.init(args=[])
    node = rclpy.create_node('approach_launch_test')
    publisher = node.create_publisher(LaserScan, '/scan', 1)
    controller.processes.start('approach', config.command('approach'), config.environment('real'))
    controller.started['approach'] = time.monotonic()
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            msg = LaserScan()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'laser'
            msg.scan_time = .05
            msg.range_min = .05
            msg.range_max = 12.
            msg.angle_increment = .01745
            msg.ranges = [float('inf')] * 360
            publisher.publish(msg)
            app.processEvents()
            rclpy.spin_once(node, timeout_sec=.01)
            if (controller.monitor_available() and
                    '/control_center/approach_cmd_vel' in controller.live and
                    'state' in controller.approach and 'estimate' in controller.approach):
                break
            time.sleep(.02)
        else:
            raise AssertionError(errors)
        assert controller.approach['state'] == 'MANUAL'
        assert controller.approach['estimate']['valid'] is False
        assert controller.status('approach') == ('ボール位置の照合待ち', 'warning')
        print('PASS: saved geometry, static TF launch, state/estimate monitoring, no-target indication')
    finally:
        controller.closing = True
        controller.stop()
        controller.can.close()
        deadline = time.monotonic() + 15
        while (controller.processes.jobs or not controller.can.idle()) and time.monotonic() < deadline:
            app.processEvents()
            time.sleep(.025)
        assert not controller.processes.jobs
        node.destroy_node()
        rclpy.try_shutdown()
