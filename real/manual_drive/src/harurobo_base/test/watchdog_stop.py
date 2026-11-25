"""Independent heartbeat watchdog: disables all motors if the drive bench dies.

Listens on UDP 127.0.0.1:55901. A heartbeat gap over TIMEOUT_S latches a
paced zero+disable burst, repeated forever until this process is killed.
Receiving 'done' exits cleanly (the bench finished its own shutdown).
This is a PC-side process guard only: total PC death or USB/CAN physical
loss is NOT covered. The human emergency stop remains the final backstop.
"""
import argparse
import socket
import struct
import time

from bench_zero_enable import MOTORS, send
from probe_dm3520_readonly import CAN_FRAME, CAN_RAW_ERR_FILTER, CAN_ERR_MASK, verify_link

ADDRESS = ('127.0.0.1', 55901)
TIMEOUT_S = 0.5
REPEAT_S = 0.5


def expire_test(ages, timeout=TIMEOUT_S):
    """Offline model of the runtime check `now - last_beat > TIMEOUT_S`."""
    if not ages:
        return True
    return ages[-1] is None or ages[-1] > timeout


def offline_tests():
    assert not expire_test([0.49])
    assert not expire_test([0.5])  # strictly-greater, matching the runtime check
    assert expire_test([0.5001])
    assert expire_test([])
    assert expire_test([None])
    print('PASS: watchdog expiry model; no CAN socket opened')


def run():
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.bind(ADDRESS)
    listener.settimeout(.05)
    can = None
    armed_at = None
    last_beat = None
    latched = False
    next_burst = 0.0
    print('WATCHDOG: waiting for the first heartbeat on UDP 55901', flush=True)
    while True:
        try:
            data, _ = listener.recv(64)
        except TimeoutError:
            data = b''
        now = time.monotonic()
        if data == b'done':
            print('WATCHDOG: bench reported clean exit', flush=True)
            if can is not None:
                can.close()
            return
        if data:
            if armed_at is None:
                armed_at = now
                print(f'WATCHDOG: armed at {now:.3f}', flush=True)
            last_beat = now
            if latched:
                continue
        if armed_at is None:
            continue
        if not latched and now - last_beat > TIMEOUT_S:
            latched = True
            print(f'WATCHDOG: HEARTBEAT LOST at +{now-armed_at:.3f}s; latching stop bursts',
                  flush=True)
        if latched and now >= next_burst:
            next_burst = now + REPEAT_S
            if can is None or can.fileno() < 0:
                try:
                    can = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                    can.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
                                   struct.pack('=I', CAN_ERR_MASK))
                    can.bind(('can0',))
                    can.settimeout(.05)
                except OSError as error:
                    print(f'WATCHDOG: cannot open CAN yet: {error}', flush=True)
                    continue
            for index, motor in enumerate(MOTORS):
                if index:
                    time.sleep(.0015)
                try:
                    send(can, motor)
                    send(can, motor, disable=True)
                except OSError as error:
                    print(f'WATCHDOG: stop send failed {motor}: {error}', flush=True)
                    can.close()
                    can = None
                    break
            print(f'WATCHDOG: stop burst sent at +{now-armed_at:.3f}s', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        offline_tests()
    else:
        run()


if __name__ == '__main__':
    main()
