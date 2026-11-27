import json
import fcntl
import hashlib
import os
import time
import signal

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster

from .storage import load_valid
from .settings import user_directory


def inverse_transform(result):
    rotation = np.asarray(result['rotation'])
    translation = np.asarray(result['translation'])
    return rotation.T, -rotation.T @ translation


class GuardedTF(Node):
    def __init__(self):
        super().__init__('camera_lidar_calibration_tf')
        self.declare_parameter('extrinsic_file', '')
        self.data, _ = load_valid(self.get_parameter('extrinsic_file').value)
        self.child = self.data['camera_frame']
        locks = user_directory() / 'locks'
        locks.mkdir(parents=True, exist_ok=True)
        key = hashlib.sha256((os.environ.get('ROS_DOMAIN_ID', '0') + '/' + self.child).encode()).hexdigest()
        self.lock = (locks / key).open('a')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.lock.close()
            raise ValueError('同じフレームの較正TF配信が既に実行されています。')
        self.parent = self.data['lidar_frame']
        self.seen = set()
        self.conflict = False
        self.sent = None
        self.own_seen = False
        self.applied = False
        self.status = self.create_publisher(String, '/camera_lidar/tf_status', 1)
        static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, '/tf_static', self.observe, static_qos)
        self.create_subscription(TFMessage, '/tf', self.observe,
                                 QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))

    def observe(self, message):
        for transform in message.transforms:
            child = transform.child_frame_id.lstrip('/')
            parent = transform.header.frame_id.lstrip('/')
            # Jazzy does not expose a publisher GID in subscription message info.
            # This one-shot broadcaster sends exactly one uniquely timestamped
            # transform. Consume that single local echo, then reject any further
            # message for the same child, even an identical foreign transform.
            if self.applied and not self.own_seen and transform == self.sent:
                self.own_seen = True
                continue
            self.seen.add((parent, child))
            if child == self.child:
                self.conflict = True

    def report(self, valid, message):
        self.status.publish(String(data=json.dumps({'valid': valid, 'message': message})))
        self.get_logger().info(message)

    def apply(self):
        # Publishing parent=lidar, child=camera stores the child's pose in its parent.
        # OpenCV provides p_camera=R*p_lidar+t, so the ROS pose is its inverse.
        if self.conflict or any(child == self.child for _, child in self.seen):
            self.report(False, f'TF競合のため拒否しました。child={self.child} は既に配信されています。')
            return False
        parents = {child: parent for parent, child in self.seen}
        ancestor = self.parent
        visited = set()
        while ancestor in parents and ancestor not in visited:
            visited.add(ancestor)
            ancestor = parents[ancestor]
            if ancestor == self.child:
                self.report(False, 'TFツリーに循環ができるため適用を拒否しました。')
                return False
        rotation, translation = inverse_transform(self.data['result'])
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.parent
        transform.child_frame_id = self.child
        position = transform.transform.translation
        position.x, position.y, position.z = map(float, translation)
        q = Rotation.from_matrix(rotation).as_quat()
        orientation = transform.transform.rotation
        orientation.x, orientation.y, orientation.z, orientation.w = map(float, q)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.sent = transform
        self.applied = True
        self.broadcaster.sendTransform(transform)
        self.report(True, f'TF配信中 {self.parent} → {self.child}')
        return True


    def destroy_node(self):
        self.lock.close()
        return super().destroy_node()


def main():
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = [False]
    def request_stop(signum, frame):
        stopping[0] = True
    for value in (signal.SIGINT, signal.SIGTERM):
        signal.signal(value, request_stop)
    node = None
    try:
        node = GuardedTF()
        deadline = time.monotonic() + 2.5
        while rclpy.ok() and not stopping[0] and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if stopping[0]:
            return 0
        if not node.apply():
            return 1
        while rclpy.ok() and not stopping[0]:
            rclpy.spin_once(node, timeout_sec=.1)
            if node.conflict:
                node.report(False, '配信後に同じchildの別TFを検出しました。配信を終了します。受信側を再起動してください。')
                return 1
    except (ValueError, OSError, KeyError) as error:
        if node:
            node.get_logger().error(str(error))
        else:
            print(str(error))
        return 1
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        return 0
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
