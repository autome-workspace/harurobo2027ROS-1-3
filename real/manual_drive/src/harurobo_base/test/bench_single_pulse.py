"""Selected ID: bounded +/-0.2 rad/s for bounded duration, stop and disable. Default offline.
Only for lifted/fixed wheels and an operator at the independent power cutoff.
TIMEOUT=0 is unverified: PC-side cleanup cannot guarantee a stop after PC failure.
"""
import argparse
import math
import socket
import struct
import time

from bench_zero_enable import MOTORS, feedback_state, send, expect_state
from probe_dm3520_readonly import CAN_FRAME, CAN_RAW_ERR_FILTER, CAN_ERR_MASK, verify_link
from probe_four_motor_inventory import decode_reply

SPEED = 0.2


def read_register(can, rid, target=1):
    can.send(CAN_FRAME.pack(0x7ff, 8, struct.pack('<HBBI', target, 0x33, rid, 0)))
    deadline = time.monotonic() + 0.1
    while time.monotonic() < deadline:
        can.settimeout(max(0.001, deadline - time.monotonic()))
        value = decode_reply(can.recv(CAN_FRAME.size), target, rid)
        if value is not None:
            return value
    raise RuntimeError('register response timeout')


def velocity(packet, vmax):
    _, _, data = CAN_FRAME.unpack(packet)
    raw = (data[3] << 4) | (data[4] >> 4)
    return (2.0 * raw / 4095.0 - 1.0) * vmax


def speed_sample(can, vmax, target=1):
    deadline = time.monotonic() + 0.1
    while time.monotonic() < deadline:
        can.settimeout(max(0.001, deadline - time.monotonic()))
        packet = can.recv(CAN_FRAME.size)
        for motor in MOTORS:
            state = feedback_state(packet, motor)
            if state is not None:
                if state != (1 if motor == target else 0):
                    raise RuntimeError(f'unexpected motor {motor} state {state}')
                if motor == target:
                    measured = velocity(packet, vmax)
                    if not math.isfinite(measured) or abs(measured) > 1.0:
                        raise RuntimeError(f'speed exceeds bench bound: {measured}')
                    return measured
    raise RuntimeError('speed response timeout')


def run(target=1, duration=0.5, speed=SPEED):
    if target not in MOTORS or not math.isfinite(duration) or not 0.1 <= duration <= 3.0:
        raise ValueError('target must be 1..4, duration 0.1..3 seconds')
    if not math.isfinite(speed) or not 0 < abs(speed) <= SPEED:
        raise ValueError('speed must be finite, nonzero and within +/-0.2 rad/s')
    verify_link()
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as can:
        can.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_ERR_FILTER, struct.pack('=I', CAN_ERR_MASK))
        can.bind(('can0',))
        can.settimeout(0.1)
        try:
            for motor in MOTORS:
                send(can, motor, disable=True)
                expect_state(can, motor, 0, active_target=target)
            for rid, expected in ((0x0a, 3), (0x07, 0x10 + target), (0x23, 4)):
                if read_register(can, rid, target) != expected:
                    raise RuntimeError('unexpected mode, feedback ID or bitrate')
            vmax = struct.unpack('<f', struct.pack('<I', read_register(can, 0x16, target)))[0]
            if vmax != 200.0:
                raise RuntimeError(f'feedback mapping changed: VMAX={vmax}')
            send(can, target)
            expect_state(can, target, 0, active_target=target)
            send(can, target, enable=True)
            expect_state(can, target, 1, active_target=target)
            # Verify initial rest before issuing any nonzero target.
            send(can, target)
            if abs(speed_sample(can, vmax, target)) > 0.15:
                raise RuntimeError('initial velocity not near zero')
            print(f'BEGIN: ID {target:#04x}, {speed:+.1f} output rad/s, {duration} s nominal', flush=True)
            deadline = time.monotonic() + duration
            samples = []
            while time.monotonic() < deadline:
                can.send(CAN_FRAME.pack(0x200 + target, 8, struct.pack('<f', speed) + bytes(4)))
                samples.append(speed_sample(can, vmax, target))
                time.sleep(0.01)
            print(f'Pulse feedback min/max: {min(samples):.3f}/{max(samples):.3f} rad/s', flush=True)
            deadline = time.monotonic() + 1.0
            settled = 0
            while time.monotonic() < deadline:
                send(can, target)
                measured = speed_sample(can, vmax, target)
                settled = settled + 1 if abs(measured) <= 0.15 else 0
                if settled >= 3:
                    break
                time.sleep(0.01)
            if settled < 3:
                raise RuntimeError('zero target did not settle within 1 s')
            send(can, target, disable=True)
            expect_state(can, target, 0, active_target=target)
            print('PASS: zero-speed feedback and disabled state confirmed', flush=True)
        finally:
            for motor in MOTORS:
                for opts in ({}, {'disable': True}):
                    try:
                        send(can, motor, **opts)
                    except OSError as error:
                        print(f'STOP SEND FAILED {motor}: {error}', flush=True)
            print('Turn drive power OFF. Cleanup does not replace independent cutoff.', flush=True)


def self_test():
    for raw in (0, 2047, 2048, 4095):
        data = bytearray(8)
        data[0] = 0x11
        data[3] = raw >> 4
        data[4] = (raw & 15) << 4
        packet = CAN_FRAME.pack(0x11, 8, data)
        assert abs(velocity(packet, 200) - ((2 * raw / 4095) - 1) * 200) < 1e-12
    assert struct.pack('<f', SPEED).hex() == 'cdcc4c3e'
    print('PASS: offline velocity conversion and bounded command; no CAN socket opened')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute-powered-fixed-wheels-estop-ready', action='store_true')
    args = p.parse_args()
    try:
        if args.execute_powered_fixed_wheels_estop_ready:
            run()
        else:
            self_test()
    except (OSError, RuntimeError, ValueError) as error:
        p.exit(1, f'Bench stopped: {error}; turn drive power OFF.\n')
    except KeyboardInterrupt:
        p.exit(1, 'Interrupted; turn drive power OFF.\n')


if __name__ == '__main__':
    main()
