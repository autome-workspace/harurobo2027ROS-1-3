"""Continuous manual drive (no window, no permit expiry). Safety invariants:

- L1 is a deadman: releasing it stops all wheels immediately.
- Re-arming requires BOTH sticks neutral AND L1 released first.
- The independent watchdog process must run; if this process dies it stops
  all motors within ~0.5 s. Start watchdog_stop.py alongside this node.
- Wheel cap comes from bench_ds4_four.WHEEL_CAP (currently 0.5 rad/s).
- TIMEOUT=0 on the motors means no motor-side comm watchdog (measured).
  Total PC death or USB/CAN physical loss still needs the human e-stop.
Stop the session with SIGINT (clean shutdown) - never with SIGKILL.
"""
import argparse
import math
import socket
import struct
import time

from bench_ds4_four import (WHEEL_CAP, collect, confirm_disabled_rest, drain_pending, mix,
                             speeds, watchdog_done, watchdog_heartbeat, zero_and_disable)
from bench_ds4_single import InputGate
from bench_single_pulse import read_register
from bench_zero_enable import MOTORS, feedback_state, send

def stop_immediately(can):
    """L1 release: cut torque right away instead of ramping down.

    The motor's DEC ramp from 5 rad/s would take ~2.5 s; the deadman must be
    faster, so we send zero targets then disable frames immediately and verify
    the disabled state."""
    drain_pending(can)
    for index, motor in enumerate(MOTORS):
        if index:
            time.sleep(.0015)
        send(can, motor)
    for index, motor in enumerate(MOTORS):
        if index:
            time.sleep(.0015)
        send(can, motor, disable=True)
    # In-flight replies to the zero-speed command still show state 1;
    # wait until every motor has confirmed the DISABLED state.
    deadline = time.monotonic() + .5
    confirmed = set()
    while time.monotonic() < deadline and len(confirmed) < 4:
        can.settimeout(max(.001, deadline - time.monotonic()))
        try:
            packet = can.recv(CAN_FRAME.size)
        except TimeoutError:
            break
        for motor in MOTORS:
            state = feedback_state(packet, motor)
            if state == 0:
                confirmed.add(motor)
    if len(confirmed) < 4:
        raise RuntimeError('motors did not confirm disabled within 0.5 s')
from probe_dm3520_readonly import CAN_FRAME, CAN_RAW_ERR_FILTER, CAN_ERR_MASK, verify_link


class ContinuousGate(InputGate):
    PERMIT = float('inf')  # L1 deadman only; no auto-expiry.


class FourContinuous(ContinuousGate):
    # A single glitched L1-release frame must not stop the drive; a real
    # release is only recognised after RELEASE_DEBOUNCE of continuous release.
    RELEASE_DEBOUNCE = 0.100

    def __init__(self):
        super().__init__()
        self.wheels = (0.,) * 4
        self.released_since = None

    def stop(self):
        super().stop()
        self.wheels = (0.,) * 4
        self.released_since = None

    def update(self, axes, buttons, now):
        self.expire(now)
        if (len(axes) < 3 or len(buttons) <= 9 or buttons[9] not in (0, 1)
                or any(not math.isfinite(v) or abs(v) > 1 for v in axes[:3])):
            self.stop()
            return
        self.last = now
        pressed_raw = buttons[9] == 1
        if pressed_raw:
            self.released_since = None
        elif self.released_since is None:
            self.released_since = now
        # The grace applies only while ARMED: while disarmed, releases must be
        # processed immediately so the neutral re-arm ritual stays responsive.
        grace = (not pressed_raw and self.armed_at is not None
                 and self.released_since is not None
                 and now - self.released_since < self.RELEASE_DEBOUNCE)
        pressed = pressed_raw or grace
        neutral = all(abs(v) <= .12 for v in axes[:3])
        if not pressed:
            self.stop()
            self.ready = neutral
            return
        if self.ready and self.armed_at is None:
            self.ready = False
            if neutral:
                self.armed_at = now
        if self.armed_at is not None:
            def shape(value):
                if abs(value) <= .12:
                    return 0.0
                return math.copysign((abs(value) - .12) / .88, value)
            self.wheels = mix(shape(axes[1]), shape(axes[0]), shape(axes[2]))


# SDL GameController button order: up, down, left, right.
DPAD_DIRECTIONS = ((1, 0), (-1, 0), (0, 1), (0, -1))
FORWARD_METRES_PER_RAD = .110
LEFT_METRES_PER_RAD = .097


class OneMetreMove:
    def __init__(self, direction, yaw, now):
        self.direction = direction
        self.yaw = yaw
        self.started = now
        self.last = now
        self.x = self.y = 0.

    def observe(self, wheels, yaw, now):
        dt = now - self.last
        if not 0 < dt <= .15:
            raise ValueError('wheel feedback interval')
        self.last = now
        a, b, c, d = wheels
        vx = FORWARD_METRES_PER_RAD * (a + b - c - d) / 4
        vy = LEFT_METRES_PER_RAD * (a - b - c + d) / 4
        heading = math.remainder(yaw - self.yaw, math.tau)
        self.x += dt * (math.cos(heading) * vx - math.sin(heading) * vy)
        self.y += dt * (math.sin(heading) * vx + math.cos(heading) * vy)

    def command(self, yaw, gyro, now):
        dx, dy = self.direction
        remaining = 1. - (dx * self.x + dy * self.y)
        cross_track = -dy * self.x + dx * self.y
        heading = math.remainder(yaw - self.yaw, math.tau)
        if remaining <= .01:
            return None, f'complete: estimated {1. - remaining:.3f} m'
        if now - self.started > 22. or abs(heading) > .35 or abs(cross_track) > .20:
            return None, 'aborted: time, heading or cross-track limit'

        along = min(.15, max(.025, 1.2 * remaining))
        lateral = max(-.035, min(.035, -.6 * cross_track))
        vx, vy = along * dx - lateral * dy, along * dy + lateral * dx
        forward = (math.cos(heading) * vx + math.sin(heading) * vy) / FORWARD_METRES_PER_RAD
        left = (-math.sin(heading) * vx + math.cos(heading) * vy) / LEFT_METRES_PER_RAD
        turn = max(-.35, min(.35, -1.5 * heading - .25 * gyro))
        return mix(forward / WHEEL_CAP, left / WHEEL_CAP, turn / WHEEL_CAP), None


class DpadDrive:
    def __init__(self, gate):
        self.gate = gate
        self.keys = ()
        self.request = None
        self.move = None
        self.imu = None
        self.cancelled = False

    def on_joy(self, axes, buttons, now):
        previously_armed = self.gate.armed_at is not None
        self.gate.update(axes, buttons, now)
        keys = tuple(i for i, value in enumerate(buttons[11:15]) if value == 1)
        edge = len(keys) == 1 and not self.keys and len(buttons) >= 15
        self.keys = keys
        neutral = len(axes) >= 3 and all(abs(value) <= .12 for value in axes[:3])
        held = len(buttons) > 9 and buttons[9] == 1
        if not held or not neutral or self.gate.armed_at is None:
            self.request = None
        if self.move is not None and (not held or not neutral or len(keys) > 1):
            self.cancelled = True
            self.gate.stop()
        if previously_armed and self.gate.armed_at is not None and held and neutral and edge:
            self.request = DPAD_DIRECTIONS[keys[0]]

    def on_imu(self, message, now):
        q = message.orientation
        gyro = message.angular_velocity.z
        norm = math.hypot(q.x, q.y, q.z, q.w)
        if (message.header.frame_id != 'imu_link' or not .98 <= norm <= 1.02
                or not math.isfinite(gyro)
                or message.orientation_covariance[0] < 0
                or message.angular_velocity_covariance[0] < 0):
            self.imu = None
            return
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.imu = (now, yaw, gyro)

    def finish(self, reason):
        print(f'DPAD {reason}; release L1 to re-arm', flush=True)
        self.move = self.request = None
        self.cancelled = False
        self.gate.stop()

    def command(self, now):
        if self.cancelled or (self.move is not None and self.gate.armed_at is None):
            self.finish('cancelled by controller')
            return self.gate.wheels
        if self.gate.armed_at is None:
            self.request = None
        if self.move is None and self.request is None:
            return self.gate.wheels
        if self.imu is None or now - self.imu[0] > .15:
            self.finish('aborted: /imu/data missing or stale')
            return self.gate.wheels
        _, yaw, gyro = self.imu
        if self.move is None:
            self.move = OneMetreMove(self.request, yaw, now)
            self.request = None
            print(f'DPAD start: direction={self.move.direction}, target=1.000 m', flush=True)
        values, reason = self.move.command(yaw, gyro, now)
        if reason is not None:
            self.finish(reason)
            return self.gate.wheels
        return values

    def observe(self, wheels, now):
        if self.move is not None:
            try:
                self.move.observe(wheels, self.imu[1], now)
            except ValueError as error:
                self.finish(f'aborted: {error}')


def run():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import Imu, Joy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

    verify_link()
    rclpy.init()
    node = rclpy.create_node('harurobo_manual_drive')
    gate = FourContinuous()
    dpad = DpadDrive(gate)
    node.create_subscription(Joy, '/joy',
                             lambda msg: dpad.on_joy(msg.axes, msg.buttons, time.monotonic()),
                             qos_profile_sensor_data)
    # Poll IMU independently so ready Joy callbacks cannot starve it.
    imu_node = rclpy.create_node('harurobo_manual_drive_imu')
    imu_executor = SingleThreadedExecutor()
    imu_executor.add_node(imu_node)
    imu_node.create_subscription(Imu, '/imu/data', lambda msg: dpad.on_imu(msg, time.monotonic()),
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
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
                vmax = struct.unpack('<f', struct.pack('<I', read_register(can, 0x16, motor)))[0]
                if vmax != 200.:
                    raise RuntimeError(f'motor {motor}: VMAX mismatch')
            print(f'READY: continuous drive. L1=deadman, release to stop. '
                  f'Neutral+release to re-arm. Cap {WHEEL_CAP} rad/s. '
                  f'Neutral L1 release/press to start; D-pad moves 1 m with IMU.', flush=True)
            previous = None
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.005)
                imu_executor.spin_once(timeout_sec=.002)
                watchdog_heartbeat()
                gate.expire(time.monotonic())
                if enabled:
                    values = dpad.command(time.monotonic())
                if gate.armed_at is None and enabled:
                    watchdog_heartbeat()
                    stop_immediately(can)
                    enabled = False
                    previous = None
                    print('DISABLED: all four confirmed; neutral L1 release/press to re-arm',
                          flush=True)
                elif gate.armed_at is not None:
                    if not enabled:
                        confirm_disabled_rest(can, heartbeat=watchdog_heartbeat)
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
                        continue
                        print('ENABLED: continuous; L1 is the deadman', flush=True)
                    watchdog_heartbeat()
                    speeds(can, values)
                    dpad.observe(collect(can, 1), time.monotonic())
                    bucket = tuple(round(v / .05) for v in values)
                    if bucket != previous:
                        print(f't={time.monotonic():.3f} '
                              f'wheels={tuple(round(v, 3) for v in values)}', flush=True)
                        previous = bucket
                time.sleep(.005)
        finally:
            for disable in (False, True):
                for motor in MOTORS:
                    try:
                        send(can, motor, disable=disable)
                    except OSError as error:
                        print(f'STOP SEND FAILED {motor}: {error}', flush=True)
            watchdog_done()
            imu_executor.shutdown()
            imu_node.destroy_node()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            print('MANUAL DRIVE ENDED. Turn drive power OFF.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-powered-robot-estop-ready', action='store_true')
    args = parser.parse_args()
    try:
        if args.execute_powered_robot_estop_ready:
            run()
        else:
            gate = FourContinuous()
            assert gate.PERMIT == float('inf')
            axes, buttons = [0.] * 3, [0] * 15
            gate.update(axes, buttons, 0.)
            buttons[9] = 1
            gate.update(axes, buttons, .01)
            axes[1] = 1.
            gate.update(axes, buttons, .02)
            assert gate.wheels == (WHEEL_CAP,) * 2 + (-WHEEL_CAP,) * 2
            for tick in range(1000):  # 10 s of continuous input must NOT auto-expire.
                gate.update(axes, buttons, .02 + tick * .01)
            assert gate.armed_at is not None
            # One glitched released frame while driving must NOT disarm.
            buttons[9] = 0
            gate.update(axes, buttons, 10.03)
            buttons[9] = 1
            gate.update(axes, buttons, 10.04)
            assert gate.armed_at is not None and gate.wheels[0] == WHEEL_CAP
            # A real ~150 ms release while driving must stop the wheels.
            buttons[9] = 0
            for tick in range(15):
                gate.update(axes, buttons, 10.05 + tick * .01)
            assert gate.armed_at is None and gate.wheels == (0.,) * 4
            print('PASS: continuous gate offline: no expiry, deadman stop. No CAN opened.')
    except (OSError, RuntimeError, ValueError) as error:
        parser.exit(1, f'Manual drive stopped: {error}; turn drive power OFF.\n')
    except KeyboardInterrupt:
        parser.exit(1, 'Interrupted; motors stopped in the finally handler.\n')


if __name__ == '__main__':
    main()
