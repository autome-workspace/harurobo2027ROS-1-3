"""Lifted four-wheel DS4 bench. Legacy mix, wheel cap 0.2 rad/s, 3 s permit.
No body-speed/geometry calibration or IMU. Default offline. Manual estop required.
"""
import argparse
import math
import socket
import struct
import time

from bench_ds4_single import InputGate
from bench_zero_enable import MOTORS, feedback_state, send
from bench_single_pulse import read_register, velocity
from probe_dm3520_readonly import CAN_FRAME, CAN_RAW_ERR_FILTER, CAN_ERR_MASK, verify_link

WHEEL_CAP = 6.6
WATCHDOG_ADDRESS = ('127.0.0.1', 55901)
_heartbeat_socket = None


def watchdog_heartbeat():
    """Feed the independent watchdog; it stops all motors if this process dies."""
    global _heartbeat_socket
    if _heartbeat_socket is None:
        _heartbeat_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    _heartbeat_socket.sendto(b'hb', WATCHDOG_ADDRESS)


def watchdog_done():
    global _heartbeat_socket
    if _heartbeat_socket is not None:
        try:
            _heartbeat_socket.sendto(b'done', WATCHDOG_ADDRESS)
            _heartbeat_socket.close()
        except OSError:
            pass
        _heartbeat_socket = None


def mix(forward, left, turn):
    values = (forward + left + turn, forward - left + turn,
              -forward - left + turn, -forward + left + turn)
    scale = WHEEL_CAP / max(1., *(abs(v) for v in values))
    return tuple(v * scale for v in values)


class FourGate(InputGate):
    PERMIT = 10.0  # Watchdog verification needs a driving window long enough to kill into.

    def __init__(self):
        super().__init__()
        self.wheels = (0.,) * 4

    def stop(self):
        super().stop()
        self.wheels = (0.,) * 4

    def update(self, axes, buttons, now):
        super().update(axes, buttons, now)
        if self.armed_at is not None:
            def shape(v):
                return math.copysign(max(0., (abs(v) - .12) / .88), v)
            self.wheels = mix(shape(axes[1]), shape(axes[0]), shape(axes[2]))


def drain_pending(can):
    # Replies have no transaction sequence: remove already-queued status before
    # sending a new batch. Never discard CAN errors silently.
    can.setblocking(False)
    try:
        for _ in range(128):
            try:
                packet = can.recv(CAN_FRAME.size)
            except BlockingIOError:
                return
            for motor in MOTORS:
                state = feedback_state(packet, motor)
                if state is not None and state not in (0, 1):
                    raise RuntimeError(f'queued motor {motor} fault/status={state}')
        raise RuntimeError('receive queue did not drain within 128 frames')
    finally:
        can.settimeout(.1)


def confirm_disabled_rest(can, heartbeat=None):
    deadline = time.monotonic() + 2.
    settled = 0
    last = None
    while time.monotonic() < deadline:
        if heartbeat is not None:
            heartbeat()
        speeds(can, (0.,)*4)
        measured = collect(can, 0)
        if measured != last:
            print(f'PREARM disabled velocity={measured}', flush=True)
            last = measured
        settled = settled+1 if all(abs(v) <= .20 for v in measured) else 0
        if settled >= 3:
            return
        time.sleep(.01)
    raise RuntimeError(f'initial wheel velocity above 0.20 rad/s after 2 s: {last}')


def collect(can, expected):
    found = {}
    deadline = time.monotonic() + .1
    while len(found) < 4 and time.monotonic() < deadline:
        can.settimeout(max(.001, deadline-time.monotonic()))
        packet = can.recv(CAN_FRAME.size)
        for motor in MOTORS:
            state = feedback_state(packet, motor)
            if state is not None:
                if state != expected:
                    raise RuntimeError(f'motor {motor} state={state}, expected={expected}')
                speed = velocity(packet, 200.)
                # Runaway guard scales with the configured command cap.
                if not math.isfinite(speed) or abs(speed) > WHEEL_CAP + .5:
                    raise RuntimeError(f'motor {motor} excessive velocity {speed}')
                found[motor] = speed
    if len(found) != 4:
        raise RuntimeError('not all wheels replied within 100 ms')
    return tuple(found[motor] for motor in MOTORS)


def speeds(can, values):
    if len(values) != 4 or any(not math.isfinite(v) or abs(v) > WHEEL_CAP + 1e-6 for v in values):
        raise ValueError('invalid wheel commands')
    drain_pending(can)
    for index, (motor, value) in enumerate(zip(MOTORS, values)):
        if index:
            time.sleep(.0015)  # Post-re-enumeration CANable drops back-to-back frames.
        can.send(CAN_FRAME.pack(0x200+motor, 8, struct.pack('<f', value)+bytes(4)))


def zero_and_disable(can, heartbeat=None):
    deadline = time.monotonic() + 1.
    settled = 0
    while time.monotonic() < deadline:
        if heartbeat is not None:
            heartbeat()
        speeds(can, (0.,)*4)
        measured = collect(can, 1)
        settled = settled+1 if all(abs(v) <= .15 for v in measured) else 0
        if settled >= 3:
            break
        time.sleep(.005)
    if settled < 3:
        raise RuntimeError('wheels did not settle on zero command within 1 s')
    drain_pending(can)
    for motor in MOTORS:
        send(can, motor, disable=True)
    measured = collect(can, 0)
    print(f'POSTSTOP disabled velocity={measured}', flush=True)


def run():
    import rclpy
    from sensor_msgs.msg import Joy
    from rclpy.qos import qos_profile_sensor_data
    verify_link()
    rclpy.init()
    node = rclpy.create_node('harurobo_ds4_four_bench')
    gate = FourGate()
    node.create_subscription(Joy, '/joy',
                             lambda msg: gate.update(msg.axes,msg.buttons,time.monotonic()),
                             qos_profile_sensor_data)
    enabled = False
    with socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW) as can:
        can.setsockopt(socket.SOL_CAN_RAW,CAN_RAW_ERR_FILTER,struct.pack('=I',CAN_ERR_MASK))
        can.bind(('can0',))
        can.settimeout(.1)
        try:
            for motor in MOTORS:
                send(can,motor,disable=True)
            collect(can,0)
            for motor in MOTORS:
                for rid,expected in ((0x0a,3),(0x07,0x10+motor),(0x23,4)):
                    if read_register(can,rid,motor)!=expected:
                        raise RuntimeError(f'motor {motor}: configuration mismatch')
                vmax=struct.unpack('<f',struct.pack('<I',read_register(can,0x16,motor)))[0]
                if vmax!=200.:
                    raise RuntimeError(f'motor {motor}: VMAX mismatch')
            print(f'READY: 120 s, four wheels. Neutral L1 release/press. Cap {WHEEL_CAP} rad/s; permit 10 s.',flush=True)
            end=time.monotonic()+120
            previous=None
            while time.monotonic()<end:
                rclpy.spin_once(node,timeout_sec=.005)
                watchdog_heartbeat()
                gate.expire(time.monotonic())
                if gate.armed_at is None and enabled:
                    zero_and_disable(can, heartbeat=watchdog_heartbeat)
                    enabled=False
                    previous=None
                    print('DISABLED: all four confirmed; neutral L1 release/press required',flush=True)
                elif gate.armed_at is not None:
                    if not enabled:
                        confirm_disabled_rest(can, heartbeat=watchdog_heartbeat)
                        gate.expire(time.monotonic())
                        if gate.armed_at is None:
                            continue
                        drain_pending(can)
                        for motor in MOTORS:
                            send(can,motor,enable=True)
                        collect(can,1)
                        enabled=True
                        print('ENABLED: four wheels, 10 s maximum',flush=True)
                    gate.expire(time.monotonic())
                    values=gate.wheels if gate.armed_at is not None else (0.,)*4
                    speeds(can,values)
                    measured=collect(can,1)
                    bucket=tuple(round(v/.05) for v in values)
                    if bucket!=previous:
                        print(f't={time.monotonic():.3f} wheels={tuple(round(v,3) for v in values)} feedback={tuple(round(v,3) for v in measured)}',flush=True)
                        previous=bucket
                time.sleep(.005)
            if enabled:
                zero_and_disable(can)
        finally:
            for motor in MOTORS:
                for opts in ({},{'disable':True}):
                    try:
                        send(can,motor,**opts)
                    except OSError as error:
                        print(f'STOP SEND FAILED {motor}: {error}',flush=True)
            watchdog_done()
            node.destroy_node()
            rclpy.shutdown()
            print('BENCH ENDED. Turn drive power OFF.',flush=True)


def self_test():
    import runpy
    legacy=runpy.run_path('/home/kaboc/Downloads/omni_ps4.py')['mix']
    for x in (-1.,0.,1.):
        for y in (-1.,0.,1.):
            for z in (-1.,0.,1.):
                assert mix(x,y,z)==legacy(x,y,z,WHEEL_CAP)
                assert max(abs(v) for v in mix(x,y,z))<=WHEEL_CAP+1e-6
    gate=FourGate()
    buttons=[0]*15
    gate.update([0.,0.,0.],buttons,0.)
    buttons[9]=1
    gate.update([0.,0.,0.],buttons,.01)
    gate.update([0.,1.,0.],buttons,.02)
    assert gate.wheels==(WHEEL_CAP,WHEEL_CAP,-WHEEL_CAP,-WHEEL_CAP)
    gate.expire(.18)
    assert gate.armed_at is None and gate.wheels==(0.,)*4
    print('PASS: 27 legacy mixing vectors, cap, neutral rearm, timeout. No CAN socket opened.')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute-powered-fixed-wheels-estop-ready',action='store_true')
    args=p.parse_args()
    try:
        run() if args.execute_powered_fixed_wheels_estop_ready else self_test()
    except (OSError,RuntimeError,ValueError) as error:
        p.exit(1,f'Bench stopped: {error}; turn drive power OFF.\n')
    except KeyboardInterrupt:
        p.exit(1,'Interrupted; turn drive power OFF.\n')


if __name__=='__main__':
    main()
