"""ROS /joy -> fixed motor 0x01, +/-0.2 rad/s, lifted bench only.

Default offline tests. Explicit execute runs 60 s maximum; neutral release/press
required, 3 s arming limit, 150 ms Joy timeout. Not a production drive node.
"""
import argparse
import math
import socket
import struct
import time

from bench_zero_enable import MOTORS, send, expect_state
from bench_single_pulse import read_register, speed_sample
from probe_dm3520_readonly import CAN_FRAME, CAN_RAW_ERR_FILTER, CAN_ERR_MASK, verify_link


class InputGate:
    PERMIT = 3.0  # Subclasses override; manual_drive disables expiry entirely.

    def __init__(self):
        self.last = None
        self.ready = False
        self.armed_at = None
        self.velocity = 0.0

    def stop(self):
        self.ready = False
        self.armed_at = None
        self.velocity = 0.0

    def expire(self, now):
        if ((self.last is not None and now - self.last >= 0.150)
                or (self.armed_at is not None and now - self.armed_at >= self.PERMIT)):
            self.stop()

    def update(self, axes, buttons, now):
        self.expire(now)
        if (len(axes) < 3 or len(buttons) <= 9 or buttons[9] not in (0, 1)
                or any(not math.isfinite(v) or abs(v) > 1 for v in axes[:3])):
            self.stop()
            return
        self.last = now
        neutral = all(abs(v) <= 0.12 for v in axes[:3])
        if not buttons[9]:
            self.stop()
            self.ready = neutral
            return
        if self.ready and self.armed_at is None:
            self.ready = False
            if neutral:
                self.armed_at = now
        if self.armed_at is not None:
            axis = axes[1]  # SDL LEFTY as ROS joy publishes it (up positive).
            self.velocity = math.copysign(max(0.0, (abs(axis) - .12) / .88) * .2, axis)


def offline_tests():
    gate = InputGate()
    axes = [0., 0., 0.]
    buttons = [0] * 15
    buttons[9] = 1
    gate.update(axes, buttons, 0.)
    assert gate.armed_at is None
    buttons[9] = 0
    gate.update(axes, buttons, .01)
    buttons[9] = 1
    gate.update(axes, buttons, .02)
    axes[1] = 1.
    gate.update(axes, buttons, .03)
    assert gate.velocity == .2
    gate.expire(.181)
    assert gate.armed_at is None and gate.velocity == 0
    gate.update(axes, buttons, .19)
    assert gate.armed_at is None
    axes[1] = 0
    buttons[9] = 0
    gate.update(axes, buttons, 1.)
    buttons[9] = 1
    gate.update(axes, buttons, 1.01)
    for tick in range(1, 303):
        gate.update(axes, buttons, 1.01 + tick * .01)
    assert gate.armed_at is None
    print('PASS: offline startup/rearm, speed cap, Joy timeout, 3 s limit. No CAN socket opened.')


def run():
    import rclpy
    from sensor_msgs.msg import Joy
    from rclpy.qos import qos_profile_sensor_data

    verify_link()
    rclpy.init()
    node = rclpy.create_node('harurobo_ds4_single_bench')
    gate = InputGate()
    node.create_subscription(Joy, '/joy',
                             lambda msg: gate.update(msg.axes, msg.buttons, time.monotonic()),
                             qos_profile_sensor_data)
    enabled = False
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as can:
        can.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_ERR_FILTER, struct.pack('=I', CAN_ERR_MASK))
        can.bind(('can0',))
        can.settimeout(.1)
        try:
            for motor in MOTORS:
                send(can, motor, disable=True)
                expect_state(can, motor, 0)
            for rid, expected in ((0x0a, 3), (0x07, 0x11), (0x23, 4)):
                if read_register(can, rid) != expected:
                    raise RuntimeError('unexpected mode, reply ID or bitrate')
            vmax = struct.unpack('<f', struct.pack('<I', read_register(can, 0x16)))[0]
            if vmax != 200.0:
                raise RuntimeError('VMAX differs from validated feedback scale')
            deadline = time.monotonic() + 60
            print('READY for 60 s: neutral release L1, press L1, left stick up/down. Motor 01 only.', flush=True)
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.005)
                gate.expire(time.monotonic())
                armed = gate.armed_at is not None
                if not armed and enabled:
                    send(can, 1)
                    # Consume the zero-speed reply before the disable reply.
                    speed_sample(can, vmax)
                    send(can, 1, disable=True)
                    expect_state(can, 1, 0)
                    enabled = False
                    print('DISABLED; neutral L1 release/press required', flush=True)
                elif armed:
                    if not enabled:
                        send(can, 1)
                        expect_state(can, 1, 0)
                        send(can, 1, enable=True)
                        expect_state(can, 1, 1)
                        enabled = True
                        print('ENABLED motor 01, 3 s maximum', flush=True)
                    # Account for elapsed preflight/feedback time before driving.
                    gate.expire(time.monotonic())
                    value = gate.velocity if gate.armed_at is not None else 0.0
                    can.send(CAN_FRAME.pack(0x201, 8, struct.pack('<f', value) + bytes(4)))
                    speed_sample(can, vmax)
                time.sleep(.005)
        finally:
            for motor in MOTORS:
                for opts in ({}, {'disable': True}):
                    try:
                        send(can, motor, **opts)
                    except OSError as error:
                        print(f'STOP SEND FAILED {motor}: {error}', flush=True)
            node.destroy_node()
            rclpy.shutdown()
            print('BENCH ENDED. Turn drive power OFF.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-powered-fixed-wheels-estop-ready', action='store_true')
    args = parser.parse_args()
    try:
        if args.execute_powered_fixed_wheels_estop_ready:
            run()
        else:
            offline_tests()
    except (OSError, RuntimeError, ValueError) as error:
        parser.exit(1, f'Bench stopped: {error}; turn drive power OFF.\n')
    except KeyboardInterrupt:
        parser.exit(1, 'Interrupted; turn drive power OFF.\n')


if __name__ == '__main__':
    main()
