"""Synthetic calibrated approach with no CAN, motor processes or velocity publishers."""
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'ros2_control_center'), str(ROOT/'real/src/camera_lidar_calibration/test')]
os.environ['ROS_DOMAIN_ID'] = '96'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
from core.approach_config import apply_calibration
from camera_lidar_calibration.camera_model import CameraModel
from camera_lidar_calibration.extrinsic_solver import solve, project
from camera_lidar_calibration.settings import defaults
from camera_lidar_calibration.storage import save_bundle, bundle
from camera_lidar_calibration.approach_detector import ApproachDetector
from ball_auto_approach.ball_localizer import BallLocalizer
from ball_auto_approach.approach_controller import ApproachController
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Joy
from test_ros_pipeline import Sensors
import rclpy
from rclpy.executors import SingleThreadedExecutor


with tempfile.TemporaryDirectory() as folder:
    folder = Path(folder)
    camera = CameraModel(960, 720, [[650.,0.,480.],[0.,650.,360.],[0.,0.,1.]], [])
    settings = defaults() | {'center_height_offset': -.015, 'min_points': 3}
    pose = np.array([.2, -.3, .05, -.45, 0., 1.])
    points = np.array([[x,y,-.015] for x in np.linspace(.35,.65,5) for y in np.linspace(-.15,.15,5)])
    pixels, _ = project(points, pose, camera)
    samples = [{'id':i+1,'role':'validation' if i%5==4 else 'train',
                'xyz':xyz.tolist(),'uv':uv.tolist()} for i,(xyz,uv) in enumerate(zip(points,pixels))]
    result = solve(samples, camera, settings)
    assert result['valid'], result['warnings']
    calibration = folder/'extrinsic.yaml'
    save_bundle(calibration, bundle(settings, camera, samples, result))
    geometry = {'lidar_position_m':[.18,0.,.08],'lidar_orientation_rad':[0.,0.,0.],
                'camera_position_m':[.18,0.,.3],'camera_pitch_rad':.4,
                'ball_diameter_m':.06,'ball_center_height_m':.065,
                'body_length_m':.5,'body_width_m':.5,
                'lidar_self_filter_box_m':[-.06,.06,.04,.09]}
    # Verify that GUI saving selects and validates the same calibration.
    os.environ['QT_QPA_PLATFORM'] = 'offscreen'
    os.environ['XDG_CONFIG_HOME'] = str(folder)
    try:
        from PySide6.QtWidgets import QApplication
    except ImportError:
        pass
    else:
        from core.config_manager import Config
        from ui.approach_settings import ApproachSettingsDialog
        app = QApplication.instance() or QApplication([])
        local = folder/'local.yaml'
        local.write_text(yaml.safe_dump({'variables':{'CAMERA_CALIBRATION':'test-intrinsics'},
                                        'robot_geometry':geometry}))
        config = Config(local)
        dialog = ApproachSettingsDialog(SimpleNamespace(config=config, processes=SimpleNamespace(jobs={})))
        dialog.extrinsic.setText(str(calibration))
        dialog.inspect_extrinsic()
        dialog.save()
        saved = yaml.safe_load(Path(config.variables['APPROACH_SETTINGS']).read_text())
        assert saved['extrinsic_file'] == str(calibration)
        saved_params = yaml.safe_load(Path(saved['parameters_file']).read_text())
        assert saved_params['approach_controller']['ros__parameters']['max_vx'] == .03
        dialog.close()

    parameters = yaml.safe_load((ROOT/'real/src/ball_auto_approach/config/params.yaml').read_text())
    apply_calibration(parameters, geometry, calibration)
    assert parameters['ball_localizer']['ros__parameters']['min_points'] == 3
    control = parameters['approach_controller']['ros__parameters']
    control.update(auto_enabled=True, validated_min_range=.1, half_length=.25, half_width=.25)
    parameters['calibrated_approach_detector'] = {'ros__parameters':{'extrinsic_file':str(calibration)}}
    parameter_file = folder/'parameters.yaml'
    parameter_file.write_text(yaml.safe_dump(parameters))
    os.environ['XDG_CONFIG_HOME'] = str(folder)
    rclpy.init(args=['--ros-args','--params-file',str(parameter_file), '-r', '/ball/detection:=/camera_lidar/ball_detection'])
    sensors = Sensors()
    # Own confirmed body reflection is removed; an external return stays visible.
    for xy in ((-.18,.06), (.22,.35)):
        angle = np.arctan2(xy[1],xy[0])
        index = round((angle+np.pi)/(2*np.pi/1440))%1440
        sensors.ranges[index] = float(np.linalg.norm(xy))
    detector = ApproachDetector()
    localizer = BallLocalizer()
    controller = ApproachController()
    transform = TransformStamped()
    transform.transform.rotation.w = 1.
    transform.transform.translation.x = .18
    transform.transform.translation.z = .08
    localizer.tf = SimpleNamespace(lookup_transform=lambda *args:transform)
    commands = []
    controller.mux = SimpleNamespace(publish=lambda manual, auto, active, inhibit:
                                    commands.append((auto if active else manual,inhibit)))
    executor = SingleThreadedExecutor()
    for node in (sensors, detector, localizer, controller):
        executor.add_node(node)
    def spin(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:executor.spin_once(timeout_sec=.01)
    def joy(start=False, deadman=True):
        msg=Joy()
        msg.header.stamp=controller.get_clock().now().to_msg()
        msg.axes=[0.]*6
        msg.buttons=[0]*15
        msg.buttons[9]=int(deadman)
        msg.buttons[3]=int(start)
        controller.on_joy(msg)
    def acquire():
        joy(deadman=False);controller.tick()
        joy();controller.tick()
        joy(start=True);controller.tick()
        end=time.monotonic()+2
        while time.monotonic()<end:
            joy();executor.spin_once(timeout_sec=.01)
            if controller.state=='APPROACHING' and commands[-1][0].linear.x>0:
                break
        assert controller.state=='APPROACHING',controller.reason
        assert localizer.lock is not None
        assert 0<commands[-1][0].linear.x<=.03
    try:
        spin(.8)
        assert controller.estimate is not None and controller.estimate.valid, (detector.compatible, controller.detection, controller.estimate, localizer.p('use_saved_extrinsic'), localizer.p('ball_center_z'), len(localizer.history))
        assert abs(controller.estimate.center.x-.68)<.005
        assert any(abs(p.x-.4)<.005 and abs(p.y-.35)<.005 for p in controller.estimate.obstacles)
        assert not any(-.06<=p.x<=.06 and .04<=p.y<=.09 for p in controller.estimate.obstacles)
        assert abs(controller.estimate.surface_distance-(np.linalg.norm([.5,0.,-.015])-.03))<.005
        # The direct saved transform works without a camera TF publisher.
        acquire()
        joy(deadman=False);controller.tick()
        assert controller.state=='ABORT' and commands[-1][1]
        spin(.1)
        acquire()
        sensors.timer.cancel()
        end=time.monotonic()+.5
        while time.monotonic()<end:
            joy();executor.spin_once(timeout_sec=.01)
        assert not any(command.linear.x>0 and not inhibit for command,inhibit in commands[-3:])
        assert not detector.compatible or controller.detection is None or not controller.detection.detected
        assert not controller.stamped_fresh(controller.estimate.header.stamp, .3)
        print('PASS: saved transform, sphere position/distance, acquisition and track, 30 mm/s cap, L1 and sensor loss stop')
    finally:
        for node in (controller,localizer,detector,sensors):
            executor.remove_node(node)
            node.destroy_node()
        rclpy.try_shutdown()
