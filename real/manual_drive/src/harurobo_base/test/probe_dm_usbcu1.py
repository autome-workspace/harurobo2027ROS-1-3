"""DM-USBCU1 V2.0 adapter heartbeat only. Disconnect CAN and motor power first."""
import argparse
from pathlib import Path
import time

import serial

DEVICE = Path('/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00')
HEARTBEAT = bytes.fromhex('55 04 aa 55')
REPLY_LENGTH = 16


def heartbeat_reply(data):
    # CMD=0 means heartbeat; bytes 2..14 have no defined CAN content.
    for first in range(len(data) - REPLY_LENGTH + 1):
        packet = data[first:first + REPLY_LENGTH]
        if packet[:2] == b'\xaa\x00' and packet[-1] == 0x55:
            return packet
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--usb-only', action='store_true', required=True,
                        help='confirm CAN and motor power are disconnected')
    parser.parse_args()
    try:
        if not DEVICE.is_symlink() or not DEVICE.is_char_device():
            parser.exit(1, f'Expected adapter not found: {DEVICE}\n')
        with serial.Serial(str(DEVICE), baudrate=921600, timeout=0.1, write_timeout=0.5,
                           exclusive=True) as port:
            time.sleep(0.3)  # Allow CDC firmware to settle after port opening.
            port.reset_input_buffer()
            port.write(HEARTBEAT)
            data = bytearray()
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                data.extend(port.read(64))
                reply = heartbeat_reply(data)
                if reply is not None:
                    print(f'Heartbeat reply ({len(reply)} bytes): {reply.hex(" ")}')
                    return
            parser.exit(1, f'No V2.0 heartbeat reply; received {bytes(data).hex(" ") or "nothing"}\n')
    except (OSError, serial.SerialException) as error:
        parser.exit(1, f'Cannot probe adapter: {error}\n')


if __name__ == '__main__':
    main()
