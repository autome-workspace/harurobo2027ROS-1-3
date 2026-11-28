"""Start real RViz against synthetic LaserScan in an isolated ROS domain."""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy

os.environ['ROS_DOMAIN_ID']='92'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
rclpy.init(args=[])
node=rclpy.create_node('rviz_scan_check')
publisher=node.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
config=Path(__file__).resolve().parents[1]/'config/control_center.rviz'
with tempfile.TemporaryFile(mode='w+') as log:
    env=os.environ|{'QT_QPA_PLATFORM':'xcb','LIBGL_ALWAYS_SOFTWARE':'1'}
    rviz=subprocess.Popen(['rviz2','-d',str(config),'--ros-args','-r','__node:=control_center_rviz_check'],
                          env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        end=time.monotonic()+10
        received_since=None
        while time.monotonic()<end:
            msg=LaserScan()
            msg.header.frame_id='laser';msg.header.stamp=node.get_clock().now().to_msg()
            msg.angle_min=-math.pi;msg.angle_max=math.pi-math.pi/180;msg.angle_increment=math.pi/180
            msg.range_min=.05;msg.range_max=12.;msg.scan_time=.1
            msg.ranges=[1.+.2*math.cos(i*math.pi/90) for i in range(360)]
            publisher.publish(msg)
            rclpy.spin_once(node,timeout_sec=.05)
            endpoints=node.get_subscriptions_info_by_topic('/scan')
            if any('rviz' in e.node_name and e.qos_profile.reliability==ReliabilityPolicy.BEST_EFFORT for e in endpoints):
                received_since=received_since or time.monotonic()
                if time.monotonic()-received_since>1:break
            if rviz.poll() is not None:break
        log.seek(0);output=log.read()
        assert rviz.poll() is None,output
        assert received_since is not None,output
        assert 'Could not load' not in output,output
        print('PASS: real RViz launches with saved config and subscribes directly to /scan with Best Effort; laser needs no map TF')
    finally:
        if rviz.poll() is None:
            os.killpg(rviz.pid,signal.SIGINT)
            try:rviz.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(rviz.pid,signal.SIGKILL);rviz.wait()
node.destroy_node();rclpy.try_shutdown()
