"""Empirical motor-side comm-loss stop test for DM3520 with TIMEOUT=0.

A separate SENDER process streams +0.2 rad/s to motor 0x01 and is SIGKILLed
mid-stream (PC-death simulation). This STOPPER process, started beforehand,
disables all motors automatically at AUTO_STOP_S regardless of the sender.
The physical wheel behavior between sender death and auto-stop is judged by
the human operator; feedback is inquiry-based and cannot be observed passively.
"""
import argparse
import socket
import struct
import time

from bench_zero_enable import MOTORS, send, feedback_state
from bench_single_pulse import velocity
from probe_dm3520_readonly import CAN_FRAME, CAN_RAW_ERR_FILTER, CAN_ERR_MASK, verify_link

AUTO_STOP_S = 8.0
OBSERVE_S = 12.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-powered-fixed-wheels-estop-ready', action='store_true',
                        required=True)
    parser.parse_args()
    verify_link()
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as can:
        can.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_ERR_FILTER, struct.pack('=I', CAN_ERR_MASK))
        can.bind(('can0',))
        can.settimeout(.05)
        start = time.monotonic()
        print('STOPPER ARMED: auto-disable at +8.0 s; start the SENDER process now', flush=True)
        stopped = False
        while time.monotonic() - start < OBSERVE_S:
            now = time.monotonic() - start
            if not stopped and now >= AUTO_STOP_S:
                for index, motor in enumerate(MOTORS):
                    if index:
                        time.sleep(.0015)
                    try:
                        send(can, motor, disable=True)
                    except OSError as error:
                        print(f'AUTO-STOP SEND FAILED {motor}: {error}', flush=True)
                stopped = True
                print('AUTO-STOP: disable frames sent to all motors', flush=True)
            try:
                packet = can.recv(CAN_FRAME.size)
            except TimeoutError:
                continue
            state = feedback_state(packet, 1)
            if state is not None:
                print(f't={now:5.2f}s motor01 state={state} '
                      f'speed={velocity(packet, 200.):+.3f}', flush=True)
        print('OBSERVATION WINDOW COMPLETE. Turn drive power OFF.', flush=True)
        print('Report: did the wheel keep spinning between the sender kill and +8 s?', flush=True)


if __name__ == '__main__':
    main()
