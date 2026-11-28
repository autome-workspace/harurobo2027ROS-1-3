import json
import math
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster

from camera_lidar_calibration.calibration_node import CalibrationNode
from camera_lidar_calibration.calibration_tf_publisher import GuardedTF
from camera_lidar_calibration.camera_model import CameraModel
from camera_lidar_calibration.lidar_projection_node import ProjectionNode
from camera_lidar_calibration.storage import load_valid


class Sensors(Node):
    def __init__(self):
        super().__init__('synthetic_calibration_sensors')
        self.camera = CameraModel(960, 720, [[650.,0,480.],[0,650.,360.],[0,0,1.]], [])
        self.rotation = Rotation.from_rotvec([.2,-.3,.05]).as_matrix()
        self.translation = np.array([-.45, 0., 1.0])
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.set_position(.5, 0.)
        self.timer = self.create_timer(.05, self.publish)

    def set_position(self, x, y):
        self.center = np.array([x,y,-.015])
        center = self.rotation @ self.center + self.translation
        radius = .03
        axis = center/np.linalg.norm(center)
        first = np.cross(axis,[0.,1.,0.])
        first /= np.linalg.norm(first)
        second = np.cross(axis,first)
        alpha = math.asin(radius/np.linalg.norm(center))
        theta = np.linspace(0,2*math.pi,160)
        rays = (axis[None]*math.cos(alpha)+math.sin(alpha)*
                (np.cos(theta)[:,None]*first+np.sin(theta)[:,None]*second))
        pixels = self.camera.project(rays)
        frame = np.zeros((720,960,3),np.uint8)
        cv2.fillPoly(frame,[np.rint(pixels).astype(np.int32)],(0,0,255))
        self.frame = frame
        angles = -math.pi + np.arange(1440)*math.tau/1440
        rays = np.column_stack((np.cos(angles),np.sin(angles)))
        b = rays@self.center[:2]
        section = math.sqrt(.03**2-.015**2)
        d = b*b-(self.center[:2]@self.center[:2]-section**2)
        self.ranges = np.where((d>=0)&(b>0),b-np.sqrt(np.maximum(d,0)),np.inf).tolist()

    def publish(self):
        now = self.get_clock().now()
        from rclpy.duration import Duration
        image_stamp = (now-Duration(seconds=.025)).to_msg()
        image = self.bridge.cv2_to_imgmsg(self.frame,'bgr8')
        image.header.frame_id='camera_optical_frame'
        image.header.stamp=image_stamp
        info=CameraInfo()
        info.header=image.header
        info.width=960
        info.height=720
        info.k=self.camera.matrix.ravel().tolist()
        info.d=[]
        info.distortion_model='plumb_bob'
        info.r=np.eye(3).ravel().tolist()
        info.p=self.camera.projection.ravel().tolist()
        scan=LaserScan()
        scan.header.frame_id='laser'
        scan.header.stamp=now.to_msg()
        scan.scan_time=.05
        scan.angle_min=-math.pi
        scan.angle_increment=math.tau/1440
        scan.range_min=.05
        scan.range_max=3.
        scan.ranges=self.ranges
        self.info_pub.publish(info)
        self.image_pub.publish(image)
        self.scan_pub.publish(scan)


def test_complete_ros_pipeline(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_CONFIG_HOME',str(tmp_path))
    monkeypatch.setenv('ROS_DOMAIN_ID','93')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE','LOCALHOST')
    path=tmp_path/'extrinsic.yaml'
    rclpy.init(args=['--ros-args','-p','extrinsic_file:='+str(path)])
    executor=SingleThreadedExecutor()
    sensors=Sensors()
    backend=CalibrationNode()
    executor.add_node(sensors)
    executor.add_node(backend)
    observer=Node('calibration_pipeline_observer')
    executor.add_node(observer)
    statuses=[]
    observer.create_subscription(String,'/camera_lidar/calibration/status',
                                 lambda msg:statuses.append(json.loads(msg.data)),1)
    def spin(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.02)
    def until(condition, timeout=8):
        end=time.monotonic()+timeout
        while not condition():
            if time.monotonic()>end:
                raise AssertionError((backend.message,backend.latest_status.get('sensor_status'),len(backend.collector.samples)))
            executor.spin_once(timeout_sec=.02)
    projection=None
    guard=None
    try:
        settings=backend.settings.copy()
        settings.update(center_height_offset=-.015,processing_hz=10.,stationary_duration=.3,sample_interval=.1)
        backend.command(String(data=json.dumps({'action':'configure','settings':settings})))
        until(lambda:backend.collector.stationary())
        backend.command(String(data=json.dumps({'action':'select','center':[.5,0.,-.015]})))
        until(lambda:backend.collector.stationary())
        assert backend.manual_selection and backend.latest_status['selected']
        backend.command(String(data='{"action":"auto","enabled":true}'))
        until(lambda:len(backend.collector.samples)==1)
        spin(1.1)
        assert len(backend.collector.samples)==1
        backend.command(String(data='{"action":"auto","enabled":false}'))
        backend.command(String(data='{"action":"clear"}'))
        backend.command(String(data='{"action":"select","center":null}'))
        for x in np.linspace(.28,.85,5):
            for y in np.linspace(-.25,.25,5):
                sensors.set_position(float(x),float(y))
                spin(.65)
                until(lambda:backend.collector.stationary())
                before=len(backend.collector.samples)
                backend.command(String(data=json.dumps({'action':'capture'})))
                assert len(backend.collector.samples)==before+1,backend.message
        backend.command(String(data='{"action":"solve"}'))
        assert backend.result and backend.result['valid'],backend.message
        assert backend.result['validation_samples']==5
        backend.command(String(data=json.dumps({'action':'save','path':str(path)})))
        saved,_=load_valid(path)
        assert len(saved['samples'])==25
        assert np.linalg.norm(np.array(saved['result']['translation'])-sensors.translation)<.025
        projection=ProjectionNode()
        executor.add_node(projection)
        positions=[]; detected=[]; distances=[]
        observer.create_subscription(PointStamped,'/ball/position',positions.append,1)
        observer.create_subscription(Bool,'/ball/detected',lambda msg:detected.append(msg.data),1)
        observer.create_subscription(Float32,'/ball/distance',lambda msg:distances.append(msg.data),1)
        until(lambda:positions and detected and detected[-1])
        assert np.linalg.norm(np.array([positions[-1].point.x,positions[-1].point.y,positions[-1].point.z])
                              -sensors.center)<.002
        sensors.timer.cancel()
        spin(1.)
        count=len(positions)
        spin(.4)
        assert len(positions)==count
        assert detected[-1] is False and math.isnan(distances[-1])
        assert not backend.collector.stationary()
        assert not backend.latest_status['camera_detected']
        # The TF lookup in the opposite direction must recover the original OpenCV mapping.
        guard=GuardedTF()
        executor.add_node(guard)
        buffer=Buffer()
        listener=TransformListener(buffer,observer)
        spin(.5)
        assert guard.apply()
        until(lambda:buffer.can_transform('camera_optical_frame','laser',rclpy.time.Time()))
        transform=buffer.lookup_transform('camera_optical_frame','laser',rclpy.time.Time())
        q=transform.transform.rotation
        rotation=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
        t=transform.transform.translation
        assert np.allclose(rotation,saved['result']['rotation'],atol=1e-6)
        assert np.allclose([t.x,t.y,t.z],saved['result']['translation'],atol=1e-6)
        assert not guard.conflict
        other=StaticTransformBroadcaster(sensors)
        conflict=TransformStamped()
        conflict.header.frame_id='conflicting_parent'
        conflict.child_frame_id='camera_optical_frame'
        conflict.transform.rotation.w=1.
        other.sendTransform(conflict)
        until(lambda:guard.conflict)
        # A fresh audit must refuse an already-owned child before creating a publisher.
        executor.remove_node(guard)
        guard.destroy_node()
        guard=GuardedTF()
        executor.add_node(guard)
        spin(.6)
        assert not guard.apply()
        assert not guard.applied
        assert statuses
        for node in (backend,projection,guard):
            topics=observer.get_publisher_names_and_types_by_node(node.get_name(),node.get_namespace())
            assert not any('cmd_vel' in name for name,_ in topics)
    finally:
        for node in (guard,projection,backend,sensors,observer):
            if node:
                executor.remove_node(node)
                node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()


def test_yaml_intrinsics_override_and_mismatch_invalidation(tmp_path, monkeypatch):
    import yaml
    monkeypatch.setenv('XDG_CONFIG_HOME',str(tmp_path))
    monkeypatch.setenv('ROS_DOMAIN_ID','94')
    path=tmp_path/'intrinsic.yaml'
    path.write_text(yaml.safe_dump({
        'image_width':640,'image_height':480,'distortion_model':'plumb_bob',
        'camera_matrix':{'data':[700.,0.,320.,0.,700.,240.,0.,0.,1.]},
        'distortion_coefficients':{'data':[]},
        'rectification_matrix':{'data':np.eye(3).ravel().tolist()},
        'projection_matrix':{'data':[700.,0.,320.,0.,0.,700.,240.,0.,0.,0.,1.,0.]},
    }))
    rclpy.init(args=['--ros-args','-p','intrinsics_file:='+str(path)])
    node=CalibrationNode()
    try:
        info=CameraInfo()
        info.width=640
        info.height=480
        info.header.frame_id='camera_optical_frame'
        # CameraInfo with K=0 must not override the explicitly chosen calibration file.
        node.info_received(info)
        assert node.compatible
        assert node.camera.matrix[0,0]==700.
        node.result={'valid':True,'warnings':[]}
        info.header.frame_id='wrong_frame'
        node.info_received(info)
        assert not node.compatible and not node.result['valid']
        sample = {'id':7,'role':'train','xyz':[.5,0.,0.],'uv':[320.,240.]}
        node.collector.samples = [sample.copy()]
        node.command(String(data=json.dumps({
            'action':'tune_stationarity',
            'values':{'stationary_duration':.4,'stationary_position':.01,'stationary_pixels':4.},
        })))
        assert node.collector.samples == [sample]
        assert node.settings['stationary_duration'] == .4
        assert node.collector.settings['stationary_position'] == .01
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_color_preview_and_selection_without_intrinsics(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_CONFIG_HOME',str(tmp_path))
    monkeypatch.setenv('ROS_DOMAIN_ID','94')
    rclpy.init(args=[])
    node=CalibrationNode()
    try:
        frame=np.zeros((720,1280,3),np.uint8)
        color=cv2.cvtColor(np.uint8([[[168,190,220]]]),cv2.COLOR_HSV2BGR)[0,0].tolist()
        cv2.circle(frame,(600,400),60,color,-1)
        cv2.circle(frame,(600,490),8,color,-1)
        image=CvBridge().cv2_to_imgmsg(frame,'bgr8')
        image.header.frame_id='camera_optical_frame'
        image.header.stamp=node.get_clock().now().to_msg()
        node.image_received(image)
        node.command(String(data=json.dumps({'action':'pick_color','uv':[600,400]})))
        assert node.result is None
        assert node.settings['hsv_lower'][0] <= 168
        node.command(String(data='{"action":"show_mask","enabled":true}'))
        node.tick()
        assert node.latest_status['camera_detected']
        assert not node.latest_status['detection']['geometry_valid']
        assert len(node.latest_status['camera_candidates']) == 2
        node.command(String(data=json.dumps({'action':'select_camera','uv':[600,400]})))
        assert node.camera_tracker.manual
        node.tick()
        assert node.latest_status['camera_detected']
        assert not node.collector.stationary()
        assert not node.collector.samples
        assert (tmp_path/'camera-lidar-calibration/ball_detector.yaml').is_file()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_lidar_preview_survives_missing_camera(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_CONFIG_HOME',str(tmp_path))
    monkeypatch.setenv('ROS_DOMAIN_ID','94')
    rclpy.init(args=[])
    node=CalibrationNode()
    try:
        scan=LaserScan()
        scan.header.frame_id='laser'
        scan.header.stamp=node.get_clock().now().to_msg()
        scan.angle_min=0.
        scan.angle_increment=math.tau/360
        scan.range_min=.05
        scan.range_max=3.
        scan.scan_time=.07
        scan.ranges=[.8]*360
        node.scan_received_callback(scan)
        node.tick()
        assert len(node.latest_status['points'])==360
        assert node.latest_status['lidar_status'].startswith('受信中')
        assert not node.collector.stationary()
        assert not node.collector.samples
        node.scan_received-=1.
        node.tick()
        assert not node.latest_status['points']
        assert node.latest_status['lidar_status']=='データ待ち'
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_sample_preset_preserves_data_and_persists(tmp_path, monkeypatch):
    import yaml
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path))
    monkeypatch.setenv('ROS_DOMAIN_ID', '94')
    rclpy.init(args=[])
    node = CalibrationNode()
    try:
        samples = [{'id': i+1, 'role': 'validation' if i == 4 else 'train',
                    'xyz': [.3+i*.05, .1, 0.], 'uv': [320., 240.]} for i in range(5)]
        node.collector.samples = samples.copy()
        node.collector.auto = True
        node.result = {'valid': True}
        original = node.settings.copy()
        node.command(String(data=json.dumps({'action': 'sample_preset', 'preset': 'five'})))
        assert node.collector.samples == samples
        assert node.collector.settings is node.settings
        assert not node.collector.auto and node.result is None
        assert node.settings['min_train_samples'] == 4
        assert node.settings['min_validation_samples'] == 1
        assert node.settings['max_rms_pixels'] == original['max_rms_pixels']
        assert node.settings['min_hull_area'] == original['min_hull_area']
        stored = yaml.safe_load((tmp_path/'camera-lidar-calibration/settings.yaml').read_text())
        assert stored == node.settings
        node.tf_active = lambda: True
        node.command(String(data=json.dumps({'action': 'sample_preset', 'preset': 'standard'})))
        assert node.settings['min_train_samples'] == 4
        node.tf_active = lambda: False
        node.command(String(data=json.dumps({'action': 'sample_preset', 'preset': 'standard'})))
        assert node.settings['min_train_samples'] == 12
        assert node.settings['min_validation_samples'] == 4
        assert node.collector.samples == samples
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_sector_settings_preserve_samples_and_reset_live_selection(tmp_path, monkeypatch):
    import yaml
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path))
    monkeypatch.setenv('ROS_DOMAIN_ID', '94')
    rclpy.init(args=[])
    node = CalibrationNode()
    try:
        sample = {'id': 1, 'role': 'train', 'xyz': [-.5, 0., 0.], 'uv': [320., 240.]}
        node.collector.samples = [sample]
        node.collector.auto = True
        node.collector.history.append({'example': True})
        node.collector.latest = {'example': True}
        node.manual_selection = True
        node.selected_center = np.array([.5, 0., 0.])
        node.result = {'valid': True}
        changes = {'lidar_sector_enabled': True, 'lidar_sector_center_deg': 180.,
                   'lidar_sector_width_deg': 180.}
        node.command(String(data=json.dumps({'action': 'lidar_sector', 'values': changes})))
        assert node.collector.samples == [sample]
        assert node.collector.latest is None and not node.collector.history
        assert node.result is None and not node.collector.auto
        assert node.selected_center is None and not node.manual_selection
        stored = yaml.safe_load((tmp_path/'camera-lidar-calibration/settings.yaml').read_text())
        assert all(stored[key] == value for key, value in changes.items())
        node.command(String(data=json.dumps({'action': 'lidar_min_points', 'values': {'min_points': 3}})))
        assert node.settings['min_points'] == 3
        assert node.collector.samples == [sample]
        stored = yaml.safe_load((tmp_path/'camera-lidar-calibration/settings.yaml').read_text())
        assert stored['min_points'] == 3
        node.tf_active = lambda: True
        node.command(String(data=json.dumps({'action': 'lidar_sector',
                                            'values': changes | {'lidar_sector_enabled': False}})))
        assert node.settings['lidar_sector_enabled']
        node.tf_active = lambda: False
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_failed_solve_clears_previous_result(tmp_path, monkeypatch):
    import camera_lidar_calibration.calibration_node as module
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path))
    monkeypatch.setenv('ROS_DOMAIN_ID', '94')
    rclpy.init(args=[])
    node = CalibrationNode()
    try:
        node.camera = object()
        node.result = {'valid': True}
        samples = [{'id': 1, 'role': 'train', 'xyz': [-.5, 0., 0.], 'uv': [320., 240.]}]
        node.collector.samples = samples
        def fail(*args):
            raise ValueError('対応点の誤差が上限超過')
        monkeypatch.setattr(module, 'solve', fail)
        node.command(String(data=json.dumps({'action': 'solve'})))
        assert node.result is None
        assert node.collector.samples == samples
        assert '対応点の誤差' in node.message
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
