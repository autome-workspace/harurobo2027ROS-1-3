"""Read-only inventory, motor IDs 0x01..04 and reply IDs 0x11..14. No enabling."""
import argparse
import json
import socket
import struct
import time

from probe_dm3520_readonly import CAN_FRAME, CAN_RAW_ERR_FILTER, CAN_ERR_FLAG, CAN_ERR_MASK, verify_link

MOTORS = {motor: 0x10 + motor for motor in range(1, 5)}
REGISTERS = ((0x0e, 'sw_ver'), (0x0a, 'CTRL_MODE'), (0x09, 'TIMEOUT'),
             (0x07, 'MST_ID'), (0x23, 'can_br'), (0x16, 'VMAX'), (0x17, 'TMAX'))


def decode_reply(packet, motor, rid):
    cid, dlc, data = CAN_FRAME.unpack(packet)
    if cid & CAN_ERR_FLAG:
        raise RuntimeError(f'CAN error {cid & CAN_ERR_MASK:#010x}; no further requests')
    if dlc != 8 or cid > 0x7ff:
        return None
    destination, operation, register, value = struct.unpack('<HBBI', data)
    if (destination, operation, register) != (motor, 0x33, rid):
        return None
    if cid != MOTORS[motor]:
        raise RuntimeError(f'Motor {motor:#04x}: unexpected reply ID {cid:#05x}')
    return value


def probe():
    verify_link()
    inventory = {}
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as can:
        can.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_ERR_FILTER, struct.pack('=I', CAN_ERR_MASK))
        can.bind(('can0',))
        for motor, reply in MOTORS.items():
            values = {}
            for rid, name in REGISTERS:
                can.send(CAN_FRAME.pack(0x7ff, 8, struct.pack('<HBBI', motor, 0x33, rid, 0)))
                deadline = time.monotonic() + 0.5
                value = None
                while time.monotonic() < deadline:
                    can.settimeout(max(0.001, deadline - time.monotonic()))
                    try:
                        value = decode_reply(can.recv(CAN_FRAME.size), motor, rid)
                    except TimeoutError:
                        break
                    if value is not None:
                        break
                if value is None:
                    raise RuntimeError(f'Motor {motor:#04x}: {name} no reply; no further requests')
                values[name] = struct.unpack('<f', struct.pack('<I', value))[0] if rid in (0x16, 0x17) else value
                print(f'{motor:#04x} -> {reply:#04x} {name}={values[name]} raw={value:#010x}', flush=True)
                time.sleep(0.05)
            if values['MST_ID'] != reply or values['CTRL_MODE'] != 3 or values['can_br'] != 4:
                raise RuntimeError(f'Motor {motor:#04x}: unexpected feedback ID, mode or bitrate')
            inventory[f'{motor:#04x}'] = values
    print(json.dumps(inventory, indent=2, allow_nan=False), flush=True)
    print('PASS: 28 read-only requests; no drive/enable/settings writes', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--powered-fixed-four-motors', action='store_true', required=True)
    parser.parse_args()
    try:
        probe()
    except (OSError, RuntimeError, ValueError) as error:
        parser.exit(1, f'Inventory stopped: {error}\n')


if __name__ == '__main__':
    main()
