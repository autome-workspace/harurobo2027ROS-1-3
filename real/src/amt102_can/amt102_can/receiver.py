"""SocketCAN position receiver with a narrowly scoped read-only fault query."""
from pathlib import Path
import re
import socket
import struct

from .protocol import CAN_FRAME, BOOT_ID


class CanReceiver:
    def __init__(self, interface='slcan0'):
        if not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}', interface):
            raise ValueError('CANインターフェース名が無効です。')
        self.interface = interface
        self.socket = None

    def is_up(self):
        try:
            root = Path('/sys/class/net') / self.interface
            return int((root/'type').read_text()) == 280 and bool(int((root/'flags').read_text(), 16) & 1)
        except (OSError, ValueError):
            return False

    def connect(self):
        if not self.is_up():
            raise OSError(f'{self.interface}がDOWNまたは未検出です。')
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            # EFF/RTR flags belong in the normal-frame mask. CAN_ERR_FLAG in a
            # SocketCAN mask selects error filters, so errors are rejected below.
            filters = (struct.pack('=II', 0x360, 0xC00007F8) + struct.pack('=II', BOOT_ID, 0xC00007FF)
                       + struct.pack('=II', 0x488, 0xC00007FF))
            sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
            sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
            sock.bind((self.interface,))
            sock.setblocking(False)
        except OSError:
            sock.close()
            raise
        self.socket = sock

    def receive(self, limit=256):
        frames = []
        for _ in range(limit):
            try:
                packet = self.socket.recv(72)
            except BlockingIOError:
                break
            if len(packet) != CAN_FRAME.size:
                continue
            can_id, length, data = CAN_FRAME.unpack(packet)
            if can_id & 0xe0000000 or length > 8:
                continue
            frames.append((can_id, data[:length]))
        return frames

    def request_fault(self, data):
        if (len(data) != 8 or not 0 <= data[0] <= 7 or data[1:7] != bytes([0,5,0,0,0,0])):
            raise ValueError('異常詳細の読み出し以外は送信できません。')
        packet = CAN_FRAME.pack(0x480,8,data)
        if self.socket.send(packet) != len(packet):
            raise OSError('異常詳細の読み出し要求を送信できません。')

    def close(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None
