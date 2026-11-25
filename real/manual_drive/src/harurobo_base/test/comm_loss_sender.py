"""Stream +0.2 rad/s to motor 0x01 until killed. Killed externally on purpose."""
import socket
import struct
import time

from probe_dm3520_readonly import CAN_FRAME, verify_link

verify_link()
with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as can:
    can.bind(('can0',))
    can.send(CAN_FRAME.pack(1, 8, b'\xff' * 7 + b'\xfc'))
    while True:
        can.send(CAN_FRAME.pack(0x201, 8, struct.pack('<f', 0.2) + bytes(4)))
        time.sleep(0.02)
