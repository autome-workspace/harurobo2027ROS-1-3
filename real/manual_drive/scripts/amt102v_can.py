#!/usr/bin/env python3
"""Publish AMT102V CAN feedback from SocketCAN to ROS 2."""
import math
import socket
import struct

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from tf2_ros import TransformBroadcaster


CAN_FRAME = struct.Struct("=IB3x8s")
CAN_ID = 0x360
POSITION_UNITS_PER_TURN = 65536


class Encoder(Node):
    def __init__(self, receiver):
        super().__init__("amt102v_can")
        self.receiver = receiver
        self.angle = self.create_publisher(Float64, "/amt102v/angle_rad", 10)
        self.fault = self.create_publisher(Bool, "/amt102v/fault", 10)
        self.tf = TransformBroadcaster(self)
        self.create_timer(0.01, self.receive)

    def receive(self):
        latest = None
        for _ in range(128):
            try:
                frame = self.receiver.recv(CAN_FRAME.size)
            except BlockingIOError:
                break
            can_id, length, payload = CAN_FRAME.unpack(frame)
            if can_id == CAN_ID and length == 6:
                latest = payload

        if latest is None:
            return

        position = int.from_bytes(latest[1:4], "big")
        angle = math.tau * (position % POSITION_UNITS_PER_TURN) / POSITION_UNITS_PER_TURN
        self.angle.publish(Float64(data=angle))
        self.fault.publish(Bool(data=bool(latest[0])))

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "encoder_base"
        transform.child_frame_id = "encoder_shaft"
        transform.transform.rotation.z = math.sin(angle / 2)
        transform.transform.rotation.w = math.cos(angle / 2)
        self.tf.sendTransform(transform)


def main():
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as receiver:
        receiver.setsockopt(
            socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
            struct.pack("=II", CAN_ID, socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG | socket.CAN_SFF_MASK),
        )
        receiver.bind(("slcan0",))
        receiver.setblocking(False)
        rclpy.init()
        node = Encoder(receiver)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
