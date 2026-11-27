"""Exercise approach control and drive remapping without any CAN access."""
from pathlib import Path
import runpy
import sys
import time
import types

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'real/src/ball_auto_approach'))
import rclpy
from ball_approach_interfaces.msg import BallDetection, BallEstimate
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point32
from ball_auto_approach.approach_controller import ApproachController

rclpy.init(args=['--ros-args', '-p', 'auto_enabled:=true',
                '-p', 'validated_min_range:=0.05', '-p', 'half_length:=0.25',
                '-p', 'half_width:=0.25', '-p', 'ball_center_z:=0.065'])
node = ApproachController()


class Capture:
    def publish(self, manual, automatic, active, inhibit):
        self.command = automatic if active else manual
        self.inhibit = inhibit


mux = Capture()
node.mux = mux


def stamp():
    return node.get_clock().now().to_msg()


def joy(start=False, deadman=True, cancel=False, x=0.):
    msg = Joy()
    msg.header.stamp = stamp()
    msg.axes = [0.] * 6
    msg.axes[1] = x
    msg.buttons = [0] * 15
    msg.buttons[9] = int(deadman)
    msg.buttons[3] = int(start)
    msg.buttons[1] = int(cancel)
    node.on_joy(msg)


def estimate(tracking, distance=.5):
    msg = BallEstimate()
    msg.valid = True
    msg.header.frame_id = 'base_link'
    msg.header.stamp = stamp()
    msg.image_stamp = node.lock_stamp if tracking else stamp()
    msg.center.x = .18 + distance + .026
    msg.center.z = .065
    msg.surface_distance = distance
    msg.scan_range_min = .05
    msg.ball_radius = .03
    msg.tracking = tracking
    node.on_estimate(msg)


def acquire():
    joy(deadman=False)
    node.tick()
    joy()
    node.tick()
    joy(start=True)
    node.tick()
    assert node.state == 'DETECTING'
    detection = BallDetection()
    detection.header.stamp = stamp()
    detection.detected = True
    detection.ray.z = 1.
    node.on_detection(detection)
    estimate(False)
    node.tick()
    assert node.state == 'APPROACHING'
    estimate(True)
    joy()
    node.tick()
    assert not mux.inhibit
    assert 0 < mux.command.linear.x <= .15


try:
    joy(deadman=False)
    node.tick()
    assert mux.inhibit and mux.command.linear.x == 0
    acquire()
    joy(x=.5)
    node.tick()
    assert node.state == 'ABORT' and mux.inhibit
    acquire()
    node.estimate.obstacles = [Point32(x=.3, y=0., z=.08)]
    node.tick()
    assert node.state == 'ABORT' and mux.inhibit
    acquire()
    joy(cancel=True)
    node.tick()
    assert node.state == 'ABORT' and mux.inhibit
    acquire()
    joy(deadman=False)
    node.tick()
    assert node.state == 'ABORT' and mux.inhibit
    acquire()
    node.joy_at = time.monotonic() - .3
    node.tick()
    assert node.state == 'ABORT' and mux.inhibit
    acquire()
    node.lock_requested = time.monotonic() - 1
    node.estimate.header.stamp.sec -= 1
    node.tick()
    assert node.state == 'ABORT' and mux.inhibit
    acquire()
    estimate(True, .35)
    node.held_since = time.monotonic() - 1
    node.tick()
    assert node.state == 'COMPLETE'
    assert mux.command.linear.x == 0
finally:
    node.destroy_node()
    rclpy.try_shutdown()

calls = []
fake_drive = types.ModuleType('manual_drive')
fake_drive.run = lambda use_twist: calls.append((use_twist, sys.argv.copy()))
sys.modules['manual_drive'] = fake_drive
runpy.run_path(str(Path(__file__).resolve().parents[1] / 'core/approach_drive.py'), run_name='__main__')
assert calls[0][0] is True
assert '/cmd_vel:=/control_center/approach_cmd_vel' in calls[0][1]
print('PASS: acquire, approach, cancel, L1, joy/estimate timeout, completion, isolated drive topic')
