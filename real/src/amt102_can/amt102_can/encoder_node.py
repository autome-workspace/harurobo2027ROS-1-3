"""ROS bridge for up to eight AMT102-CAN channels; read-only diagnostics."""
import json
import math
import signal
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .protocol import BOOT_ID, ChannelTracker, decode_position
from .receiver import CanReceiver
from .diagnostics import FaultReader


class EncoderNode(Node):
    def __init__(self):
        super().__init__('amt102_encoders')
        self.declare_parameters('', [('interface', 'slcan0'), ('expected_devices', 2),
                                    ('stale_timeout', .5), ('publish_hz', 50.)])
        expected = self.get_parameter('expected_devices').value
        stale = self.get_parameter('stale_timeout').value
        hz = self.get_parameter('publish_hz').value
        if not 1<=expected<=8 or not math.isfinite(stale) or not .05<=stale<=10 or not math.isfinite(hz) or not 1<=hz<=50:
            raise ValueError('台数・鮮度上限・配信周期の設定が無効です。')
        self.expected = expected
        self.stream_id = f'{os.getpid()}:{time.monotonic_ns()}'
        self.receiver = CanReceiver(self.get_parameter('interface').value)
        self.tracker = ChannelTracker(stale)
        self.faults = FaultReader()
        self.next_connect = 0.
        self.error = 'CAN受信待ち'
        self.invalid_frames = 0
        self.status = self.create_publisher(String, '/encoders/state', 1)
        self.joints = {channel:self.create_publisher(JointState, f'/encoders/ch{channel}/joint_state', 1)
                       for channel in range(8)}
        self.last_published = {}
        self.create_timer(.005, self.poll)
        self.create_timer(1/hz, self.publish)
        self.get_logger().info('AMT102-CAN読み取り専用。異常詳細の読出しのみ送信し、設定変更・履歴クリア・SYNC・モーター指令は送信しません。')

    def poll(self):
        now = time.monotonic()
        try:
            if self.receiver.socket is None:
                if now < self.next_connect:
                    return
                self.next_connect = now+1.
                self.receiver.connect()
                self.error = ''
            for can_id, data in self.receiver.receive():
                if can_id == BOOT_ID and not data:
                    self.tracker.boot()
                    self.faults.reset()
                    self.last_published.clear()
                elif can_id == 0x488:
                    self.faults.reply(data, time.monotonic())
                else:
                    try:
                        position = decode_position(can_id, data)
                    except ValueError:
                        self.invalid_frames += 1
                        continue
                    self.tracker.update(position, time.monotonic(), self.get_clock().now().nanoseconds)
                    self.faults.observe(position.channel, position.fault)
            active = [row['channel'] for row in self.tracker.snapshot(time.monotonic()) if row['fresh']]
            query = self.faults.request(active, time.monotonic())
            if query is not None:
                try:
                    self.receiver.request_fault(query)
                except OSError:
                    self.faults.failed()
        except OSError as error:
            self.error = str(error)
            self.receiver.close()
            self.faults.reset()

    def publish(self):
        if self.receiver.socket is not None and not self.receiver.is_up():
            self.error = f'{self.receiver.interface}がDOWNまたは切断されました。'
            self.receiver.close()
            self.faults.reset()
        rows = self.tracker.snapshot(time.monotonic())
        connected = self.receiver.socket is not None
        for row in rows:
            row.update(self.faults.describe(row['channel'], row['fault'], time.monotonic()))
            row['fresh'] = row['fresh'] and connected
            if not row['fresh']:
                row['rx_hz'] = 0.
            signature = (row['segment'], row['frames'], row['ros_stamp_ns'])
            if row['fresh'] and self.last_published.get(row['channel']) != signature:
                message = JointState()
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(row['ros_stamp_ns'], 10**9)
                message.name = [f"encoder_ch{row['channel']}"]
                message.position = [row['relative_radians']]
                self.joints[row['channel']].publish(message)
                self.last_published[row['channel']] = signature
        result = {'interface':self.receiver.interface, 'can_connected':connected, 'read_only':True,
                  'stream_id':self.stream_id,
                  'expected_devices':self.expected, 'received_channels':sum(row['fresh'] for row in rows),
                  'error':self.error, 'boot_notifications':self.tracker.boot_notifications,
                  'invalid_frames':self.invalid_frames, 'channels':rows}
        self.status.publish(String(data=json.dumps(result, ensure_ascii=False, allow_nan=False)))

    def destroy_node(self):
        self.receiver.close()
        return super().destroy_node()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = [False]
    for value in (signal.SIGINT, signal.SIGTERM):
        signal.signal(value, lambda *_: stopping.__setitem__(0,True))
    node = None
    try:
        node = EncoderNode()
        while rclpy.ok() and not stopping[0]:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
