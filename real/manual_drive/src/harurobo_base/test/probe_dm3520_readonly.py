"""Read only DM3520-1EC registers on the single-motor bench, CANable in ONE-SHOT."""
import argparse
import json
import socket
import struct
import subprocess
import time

CAN0 = 'can0'
HOST_REQUEST_ID = 0x7ff
MOTOR_ID = 0x01
READ_OPCODE = 0x33
REGISTERS = ((0x0e, 'sw_ver'), (0x0a, 'CTRL_MODE'), (0x09, 'TIMEOUT'))
SETTINGS = (*REGISTERS, (0x0d, 'hw_ver'), (0x24, 'sub_ver'),
            (0x07, 'MST_ID'), (0x23, 'can_br'), (0x06, 'MAX_SPD'),
            (0x04, 'ACC'), (0x05, 'DEC'))
FLOAT_REGISTERS = {0x04, 0x05, 0x06}
CAN_FRAME = struct.Struct('=IB3x8s')
CAN_RAW_ERR_FILTER = 2  # Linux uapi: linux/can/raw.h.
CAN_ERR_FLAG = 0x20000000
CAN_ERR_MASK = 0x1fffffff


def request(rid):
    return CAN_FRAME.pack(HOST_REQUEST_ID, 8, struct.pack('<HBBI', MOTOR_ID, READ_OPCODE, rid, 0))


def decode(packet, rid):
    can_id, length, data = CAN_FRAME.unpack(packet)
    if can_id & CAN_ERR_FLAG:
        return 'error', (can_id & CAN_ERR_MASK, data.hex(' '))
    if can_id > 0x7ff or length != 8:
        return 'unrelated', None
    motor_id, operation, register, value = struct.unpack('<HBBI', data)
    if (motor_id, operation, register) != (MOTOR_ID, READ_OPCODE, rid):
        return 'unrelated', None
    return 'reply', (can_id, value)


def verify_link():
    links = json.loads(subprocess.check_output(['ip', '-j', '-details', 'link', 'show', 'dev', CAN0]))
    link, = links
    config = link.get('linkinfo', {})
    settings = config.get('info_data', {})
    if (config.get('info_kind') != 'can' or 'UP' not in link.get('flags', [])
            or link.get('mtu') != 16 or set(settings.get('ctrlmode', [])) != {'ONE-SHOT'}
            or settings.get('state') != 'ERROR-ACTIVE' or settings.get('restart_ms') != 0
            or settings.get('bittiming', {}).get('bitrate') != 1_000_000):
        raise RuntimeError('Expected Classic CAN UP, ERROR-ACTIVE, 1 Mbit/s, ONE-SHOT only')


def probe(settings=False):
    verify_link()
    registers, cycles = (SETTINGS, 1) if settings else (REGISTERS, 3)
    expected = {}
    error_count = 0
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as can:
        can.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_ERR_FILTER, struct.pack('=I', CAN_ERR_MASK))
        can.bind((CAN0,))
        can.settimeout(0.1)
        for cycle in range(1, cycles + 1):
            for rid, name in registers:
                can.send(request(rid))
                deadline = time.monotonic() + 0.5
                kind = None
                while time.monotonic() < deadline:
                    can.settimeout(max(0.001, deadline - time.monotonic()))
                    try:
                        kind, result = decode(can.recv(CAN_FRAME.size), rid)
                    except TimeoutError:
                        break
                    if kind == 'error':
                        error_count += 1
                        mask, payload = result
                        print(f'{cycle}: {name} CAN error mask={mask:#010x} data={payload}', flush=True)
                        if mask & 0x40:  # CAN_ERR_BUSOFF
                            raise RuntimeError('bus-off; no remaining requests sent')
                    elif kind == 'reply':
                        can_id, value = result
                        display = struct.unpack('<f', struct.pack('<I', value))[0] if rid in FLOAT_REGISTERS else value
                        print(f'{cycle}: {name} reply={can_id:#05x} raw={value:#010x} ({display})', flush=True)
                        if expected.setdefault(name, result) != result:
                            raise RuntimeError(f'{name}: reply differs between runs')
                        break
                else:
                    raise RuntimeError(f'{name}: no matched reply; no remaining requests sent')
                if kind != 'reply':
                    raise RuntimeError(f'{name}: no matched reply; no remaining requests sent')
                time.sleep(0.05)
    print(f'Completed {cycles} read cycle(s), {len(registers) * cycles} requests; '
          f'CAN errors observed: {error_count}', flush=True)
    return 2 if error_count else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--powered-single-motor', action='store_true', required=True,
                        help='confirm only the fixed single-motor bench is connected and powered')
    parser.add_argument('--settings', action='store_true', help='read the extended settings once')
    args = parser.parse_args()
    try:
        raise SystemExit(probe(args.settings))
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        parser.exit(1, f'Probe stopped: {error}\n')


if __name__ == '__main__':
    main()
