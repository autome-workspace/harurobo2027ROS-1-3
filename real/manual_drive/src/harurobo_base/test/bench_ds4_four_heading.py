"""Lifted four-wheel DS4 bench with IMU heading hold (CAN driving).

Default offline self-test. Explicit execution requires a powered, fixed,
lifted-wheel robot with the independent emergency stop at hand.
Wheel cap 0.2 rad/s, permit 3 s, Joy timeout 150 ms, IMU timeout 100 ms.
PC-side cleanup is not a hardware watchdog; TIMEOUT=0 is still unverified.
"""
import argparse
import math
import socket
import struct
import time

from bench_zero_enable import MOTORS, send, expect_state, feedback_state
from bench_ds4_four import collect, speeds, zero_and_disable, confirm_disabled_rest, mix, drain_pending
from bench_single_pulse import read_register
from probe_dm3520_readonly import CAN_FRAME, CAN_RAW_ERR_FILTER, CAN_ERR_MASK, verify_link

DEADZONE = 0.12
KP, KD, LIMIT = 0.5, 0.0, 0.2
IMU_TIMEOUT = 0.100
PERMIT = 10.0
TAU = 2.0 * math.pi


def shape(value):
    if abs(value) <= DEADZONE:
        return 0.0
    return math.copysign((abs(value) - DEADZONE) / (1 - DEADZONE), value)


class HeadingHoldPy:
    """Mirror of the offline C++ controller: manual priority, settle capture."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.target = None
        self.settled_since = None

    def update(self, permitted, translating, manual_wz, yaw, gyro_z, imu_fresh, now):
        if not permitted:
            self.reset()
            return 0.0, True, False
        if (not imu_fresh or not all(math.isfinite(v) for v in (yaw, gyro_z, manual_wz))):
            self.reset()
            return 0.0, False, False
        if manual_wz != 0.0:
            self.reset()
            return manual_wz, True, False
        if not translating:
            self.reset()
            return 0.0, True, False
        if self.target is None:
            if abs(gyro_z) > 0.05:
                self.settled_since = None
                return 0.0, True, False
            if self.settled_since is None:
                self.settled_since = now
            if now - self.settled_since < 0.200:
                return 0.0, True, False
            self.target = yaw
        error = math.remainder(self.target - yaw, TAU)
        correction = KP * error - KD * gyro_z
        if not math.isfinite(correction):
            self.reset()
            return 0.0, False, False
        return max(-LIMIT, min(LIMIT, correction)), True, True


class Gate:
    """Manual gate plus wheel mixing; arming rules match the earlier benches."""

    def __init__(self):
        self.last = None
        self.ready = False
        self.armed_at = None
        self.forward = self.left = self.manual = 0.0

    def stop(self):
        self.ready = False
        self.armed_at = None
        self.forward = self.left = self.manual = 0.0

    def expire(self, now):
        if ((self.last is not None and now - self.last >= 0.150)
                or (self.armed_at is not None and now - self.armed_at >= PERMIT)):
            self.stop()

    def update(self, axes, buttons, now):
        self.expire(now)
        if (len(axes) < 3 or len(buttons) <= 9 or buttons[9] not in (0, 1)
                or any(not math.isfinite(v) or abs(v) > 1 for v in axes[:3])):
            self.stop()
            return
        self.last = now
        neutral = all(abs(v) <= DEADZONE for v in axes[:3])
        if not buttons[9]:
            self.stop()
            self.ready = neutral
            return
        if self.ready and self.armed_at is None:
            self.ready = False
            if neutral:
                self.armed_at = now
        if self.armed_at is not None:
            self.forward, self.left, self.manual = (shape(axes[1]), shape(axes[0]), shape(axes[2]))

    def wheels(self, correction):
        if self.armed_at is None:
            return (0.0,) * 4
        return mix(self.forward, self.left, self.manual + correction)


def offline_tests():
    gate = Gate()
    hold = HeadingHoldPy()
    axes, buttons = [0.0, 0.0, 0.0], [0] * 15
    buttons[9] = 1
    gate.update(axes, buttons, 0.0)
    buttons[9] = 0
    gate.update(axes, buttons, 0.01)
    buttons[9] = 1
    gate.update(axes, buttons, 0.02)
    axes[1] = 1.0
    gate.update(axes, buttons, 0.03)
    assert gate.wheels(0.0) == (.2, .2, -.2, -.2)
    wz, healthy, holding = hold.update(True, True, 0.0, 0.0, 0.0, True, 0.03)
    assert healthy and not holding and wz == 0.0
    wz, healthy, holding = hold.update(True, True, 0.0, 0.0, 0.0, True, 0.23)
    assert healthy and holding
    target = hold.target
    assert target == 0.0
    wz, healthy, holding = hold.update(True, True, 0.0, 0.8, 0.0, True, 0.23)
    assert wz == -LIMIT and healthy and holding
    wz, healthy, holding = hold.update(True, True, 0.5, 0.8, 0.0, True, 0.24)
    assert wz == 0.5 and not holding
    assert hold.target is None
    hold.update(True, True, 0.0, math.pi - 0.1, 0.0, True, 0.30)
    hold.update(True, True, 0.0, math.pi - 0.1, 0.0, True, 0.50)
    assert hold.target is not None
    wz, healthy, holding = hold.update(True, True, 0.0, -math.pi + 0.1, 0.0, True, 0.51)
    assert abs(wz - KP * math.remainder((math.pi - 0.1) - (-math.pi + 0.1), TAU)) < 1e-12 and healthy and holding
    wz, healthy, holding = hold.update(True, True, 0.0, 0.0, 0.0, False, 0.52)
    assert not healthy and wz == 0.0 and hold.target is None
    axes[1] = 0.0
    gate.update(axes, buttons, 0.53)
    assert gate.wheels(0.0) == (0.0,) * 4
    gate.expire(0.181)
    assert gate.armed_at is None
    print('PASS: offline heading gate/hold; no CAN socket opened.')


def run():
    import rclpy
    from sensor_msgs.msg import Imu, Joy
    from rclpy.qos import qos_profile_sensor_data

    verify_link()
    rclpy.init()
    node = rclpy.create_node('harurobo_ds4_four_heading_bench')
    gate = Gate()
    hold = HeadingHoldPy()
    imu = {'t': None, 'yaw': None, 'gyro_z': None}
    imu_fault = False

    def on_imu(msg):
        q = msg.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if (msg.header.frame_id != 'imu_link' or not math.isfinite(norm)
                or abs(norm - 1.0) > 1e-3
                or not all(math.isfinite(v) for v in (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))):
            return
        imu['t'] = time.monotonic()
        imu['yaw'] = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        imu['gyro_z'] = msg.angular_velocity.z

    node.create_subscription(Imu, '/imu/data', on_imu, qos_profile_sensor_data)
    node.create_subscription(Joy, '/joy',
                             lambda msg: gate.update(msg.axes, msg.buttons, time.monotonic()),
                             qos_profile_sensor_data)

    enabled = False
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as can:
        can.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_ERR_FILTER, struct.pack('=I', CAN_ERR_MASK))
        can.bind(('can0',))
        can.settimeout(.1)
        try:
            for index, motor in enumerate(MOTORS):
                if index:
                    time.sleep(.0015)
                send(can, motor, disable=True)
            collect(can, 0)
            for motor in MOTORS:
                for rid, expected in ((0x0a, 3), (0x07, 0x10 + motor), (0x23, 4)):
                    if read_register(can, rid, motor) != expected:
                        raise RuntimeError(f'motor {motor}: configuration mismatch')
                if struct.unpack('<f', struct.pack('<I', read_register(can, 0x16, motor)))[0] != 200.:
                    raise RuntimeError(f'motor {motor}: VMAX mismatch')
            deadline = time.monotonic() + 5
            while imu['t'] is None or time.monotonic() - imu['t'] > IMU_TIMEOUT:
                if time.monotonic() > deadline:
                    raise RuntimeError('no fresh IMU samples; not enabling anything')
                rclpy.spin_once(node, timeout_sec=.02)
            print('READY: 120 s. Neutral L1 release/press. Wheels stay lifted; estop at hand.', flush=True)
            end = time.monotonic() + 120
            previous = None
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.005)
                now = time.monotonic()
                gate.expire(now)
                imu_fresh = imu['t'] is not None and now - imu['t'] < IMU_TIMEOUT
                armed = gate.armed_at is not None
                translating = math.hypot(gate.forward, gate.left) > 0.0
                if imu_fresh:
                    imu_fault = False
                if armed and (imu_fault or not imu_fresh):
                    zero_and_disable(can)
                    enabled = False
                    imu_fault = not imu_fresh
                    print('IMU FAULT: stopped; fresh IMU and neutral L1 re-press required', flush=True)
                    previous = None
                    continue
                if not armed and enabled:
                    zero_and_disable(can)
                    enabled = False
                    previous = None
                    print('DISABLED: all four confirmed; neutral L1 release/press required', flush=True)
                    continue
                if armed:
                    if not enabled:
                        print('GATE ARMED: starting pre-enable checks', flush=True)
                        try:
                            confirm_disabled_rest(can)
                        except TimeoutError:
                            print('PREARM COLLECT TIMEOUT: in-process diagnostics', flush=True)
                            try:
                              for motor in MOTORS:
                                t0 = time.monotonic()
                                can.send(CAN_FRAME.pack(0x200 + motor, 8, struct.pack('<f', 0.0) + bytes(4)))
                                latency = None
                                end = time.monotonic() + .3
                                while time.monotonic() < end:
                                    can.settimeout(max(.001, end - time.monotonic()))
                                    try:
                                        packet = can.recv(CAN_FRAME.size)
                                    except TimeoutError:
                                        break
                                    if feedback_state(packet, motor) is not None:
                                        latency = (time.monotonic() - t0) * 1000
                                        break
                                print(f'  single motor {motor}: latency_ms={latency}', flush=True)
                            except Exception as error:
                              print(f'  single-send diagnostic incomplete: {error!r}', flush=True)
                            seen = set()
                            t0 = time.monotonic()
                            for index, motor in enumerate(MOTORS):
                                if index:
                                    time.sleep(.0015)
                                can.send(CAN_FRAME.pack(0x200 + motor, 8, struct.pack('<f', 0.0) + bytes(4)))
                            end = t0 + .3
                            while len(seen) < 4 and time.monotonic() < end:
                                can.settimeout(max(.001, end - time.monotonic()))
                                try:
                                    packet = can.recv(CAN_FRAME.size)
                                except TimeoutError:
                                    break
                                for found in MOTORS:
                                    if feedback_state(packet, found) is not None:
                                        seen.add(found)
                            print(f'  immediate paced burst: replies={sorted(seen)}', flush=True)
                            raise
                        print('PREARM REST OK', flush=True)
                        gate.expire(time.monotonic())
                        if gate.armed_at is None:
                            continue
                        drain_pending(can)
                        speeds(can, (0.,) * 4)
                        collect(can, 0)
                        for index, motor in enumerate(MOTORS):
                            if index:
                                time.sleep(.0015)
                            send(can, motor, enable=True)
                        collect(can, 1)
                        enabled = True
                        print('ENABLED: four wheels, 10 s maximum', flush=True)
                    gate.expire(time.monotonic())
                    if gate.armed_at is None:
                        continue
                    wz, healthy, holding = hold.update(
                        True, translating, gate.manual, imu['yaw'], imu['gyro_z'], imu_fresh, now)
                    if not healthy:
                        continue
                    values = gate.wheels(wz)
                    speeds(can, values)
                    collect(can, 1)
                    bucket = (tuple(round(v / .05) for v in values), holding)
                    if bucket != previous:
                        print(f't={now:.3f} wheels={tuple(round(v,3) for v in values)} '
                              f'holding={holding}', flush=True)
                        previous = bucket
                time.sleep(.005)
            if enabled:
                zero_and_disable(can)
        finally:
            frames = [(motor, opts) for motor in MOTORS for opts in ({}, {'disable': True})]
            for index, (motor, opts) in enumerate(frames):
                if index:
                    time.sleep(.0015)
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
        import traceback
        traceback.print_exc()
        parser.exit(1, f'Bench stopped: {error}; turn drive power OFF.\n')
    except KeyboardInterrupt:
        parser.exit(1, 'Interrupted; turn drive power OFF.\n')


if __name__ == '__main__':
    main()
