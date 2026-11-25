"""Single-motor zero-speed enable test. Default: offline self-test, no CAN access.

No nonzero speed and no parameter writes. A PC-side finally block does not protect
against process/USB/PC failure: keep the independent emergency stop at hand.
"""
import argparse
import socket
import struct
import time

from probe_dm3520_readonly import CAN_FRAME, CAN_RAW_ERR_FILTER, CAN_ERR_FLAG, CAN_ERR_MASK, verify_link

MOTORS = range(1, 5)
ZERO = bytes(8)
DISABLE = b'\xff' * 7 + b'\xfd'
ENABLE = b'\xff' * 7 + b'\xfc'


def feedback_state(packet, motor):
    cid, dlc, data = CAN_FRAME.unpack(packet)
    if cid & CAN_ERR_FLAG:
        raise RuntimeError(f'CAN error {cid & CAN_ERR_MASK:#x}')
    if cid != 0x10 + motor or dlc != 8:
        return None
    # Register replies have a different layout; never interpret them as status.
    if data[0:2] == struct.pack('<H', motor) and data[2] == 0x33:
        return None
    if data[0] & 0x0f != motor:
        raise RuntimeError(f'Wrong motor nibble for feedback ID {cid:#x}')
    return data[0] >> 4


def send(can, motor, enable=False, disable=False):
    if enable and disable:
        raise ValueError('mutually exclusive')
    cid = motor if enable or disable else 0x200 + motor
    payload = ENABLE if enable else DISABLE if disable else ZERO
    can.send(CAN_FRAME.pack(cid, 8, payload))


def expect_state(can, motor, expected, active_target=1):
    deadline = time.monotonic() + 0.1
    while time.monotonic() < deadline:
        can.settimeout(max(0.001, deadline - time.monotonic()))
        try:
            packet = can.recv(CAN_FRAME.size)
        except TimeoutError:
            break
        # Validate other motors' faults, too, if they send status.
        for other in MOTORS:
            state = feedback_state(packet, other)
            if state is not None and state not in (0, 1):
                raise RuntimeError(f'motor {other:#x} reports fault/status {state:#x}')
            if other != active_target and state == 1:
                raise RuntimeError(f'non-target motor {other:#x} is enabled')
            if other == motor and state is not None:
                if state != expected:
                    raise RuntimeError(f'motor {motor:#x} state {state}, expected {expected}')
                return
    raise RuntimeError(f'motor {motor:#x} has no matched status within 100 ms')


def run():
    verify_link()
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as can:
        can.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_ERR_FILTER, struct.pack('=I', CAN_ERR_MASK))
        can.bind(('can0',))
        can.settimeout(0.1)
        try:
            for motor in MOTORS:
                send(can, motor, disable=True)
                expect_state(can, motor, 0)
                print(f'{motor:#04x}: disabled confirmed', flush=True)
            send(can, 1)  # Clear the speed target before enabling.
            expect_state(can, 1, 0)
            send(can, 1, enable=True)
            expect_state(can, 1, 1)
            print('0x01: enabled at zero speed; holding for approximately 0.5 s', flush=True)
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                send(can, 1)
                expect_state(can, 1, 1)
                time.sleep(0.01)
            send(can, 1, disable=True)
            expect_state(can, 1, 0)
            print('PASS: motor 0x01 disabled after zero-speed test', flush=True)
        finally:
            for motor in MOTORS:
                # Both are best effort; human power cutoff is independent.
                for options in ({}, {'disable': True}):
                    try:
                        send(can, motor, **options)
                    except OSError as error:
                        print(f'STOP SEND FAILED {motor:#x}: {error}', flush=True)
            print('Test ended. Turn drive power OFF; final cleanup is not verified.', flush=True)


def self_test():
    packet = lambda cid, data: CAN_FRAME.pack(cid, 8, data)
    assert feedback_state(packet(0x11, bytes.fromhex('1100000000000000')), 1) == 1
    assert feedback_state(packet(0x11, bytes.fromhex('0100000000000000')), 1) == 0
    assert feedback_state(packet(0x11, bytes.fromhex('0100330900000000')), 1) is None
    assert feedback_state(packet(0x12, bytes(8)), 1) is None
    class Fake:
        def __init__(self):
            self.frames = []
        def send(self, data):
            self.frames.append(CAN_FRAME.unpack(data))
    fake = Fake()
    send(fake, 1)
    send(fake, 1, enable=True)
    send(fake, 1, disable=True)
    assert fake.frames == [(0x201, 8, ZERO), (1, 8, ENABLE), (1, 8, DISABLE)]
    print('PASS: offline frame/status tests. No CAN socket opened.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-powered-fixed-wheels-estop-ready', action='store_true')
    args = parser.parse_args()
    try:
        if args.execute_powered_fixed_wheels_estop_ready:
            run()
        else:
            self_test()
    except (OSError, RuntimeError, ValueError) as error:
        parser.exit(1, f'Bench stopped: {error}. Turn drive power OFF.\n')
    except KeyboardInterrupt:
        parser.exit(1, 'Interrupted; turn drive power OFF.\n')


if __name__ == '__main__':
    main()
