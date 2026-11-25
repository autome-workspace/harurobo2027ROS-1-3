#!/usr/bin/env python3
"""Record DM IMU and four DM3520 feedback streams without sending CAN frames."""

import argparse
import csv
import math
from pathlib import Path
import socket
import struct
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--can", default="can0")
    parser.add_argument("--duration", type=float, default=180)
    parser.add_argument(
        "--drive-repo", type=Path,
        default=Path.home() / "ros2-poc-manual-drive",
    )
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be positive and finite")
    sys.path.insert(0, str(args.drive_repo / "src/harurobo_base/test"))
    from bench_zero_enable import MOTORS, feedback_state
    from bench_single_pulse import velocity
    from probe_dm3520_readonly import CAN_FRAME

    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as can:
        filters = b"".join(struct.pack("=II", 0x10 + motor, 0xC00007FF) for motor in MOTORS)
        can.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
        can.bind((args.can,))
        can.setblocking(False)
        with args.output.open("x", newline="", buffering=65536) as stream:
            writer = csv.writer(stream)
            writer.writerow(("monotonic_ns", "source", "motor_id", "state", "wheel_rad_s",
                             "imu_stamp_ns", "yaw_rad", "gyro_z_rad_s", "q_x", "q_y", "q_z", "q_w"))
            count_imu = 0
            counts = dict.fromkeys(MOTORS, 0)

            def on_imu(msg):
                nonlocal count_imu
                q = msg.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
                stamp = msg.header.stamp
                writer.writerow((time.monotonic_ns(), "imu", "", "", "",
                                 stamp.sec * 10**9 + stamp.nanosec, yaw,
                                 msg.angular_velocity.z, q.x, q.y, q.z, q.w))
                count_imu += 1

            rclpy.init()
            node = rclpy.create_node("drive_calibration_recorder")
            node.create_subscription(Imu, "/imu/data", on_imu, qos_profile_sensor_data)
            deadline = time.monotonic() + args.duration
            next_report = time.monotonic() + 2
            print(f"Recording to {args.output}. Waiting for IMU and motor feedback.", flush=True)
            try:
                while rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.002)
                    for _ in range(32):
                        try:
                            packet = can.recv(CAN_FRAME.size)
                        except BlockingIOError:
                            break
                        received = time.monotonic_ns()
                        for motor in MOTORS:
                            state = feedback_state(packet, motor)
                            if state is not None:
                                writer.writerow((received, "can", motor, state, velocity(packet, 200.),
                                                 "", "", "", "", "", "", ""))
                                counts[motor] += 1
                                break
                    if time.monotonic() >= next_report:
                        stream.flush()
                        print(f"IMU={count_imu}, motor feedback={counts}", flush=True)
                        next_report = time.monotonic() + 2
            except KeyboardInterrupt:
                pass
            finally:
                node.destroy_node()
                rclpy.try_shutdown()
                print(f"Saved {args.output}: IMU={count_imu}, motor feedback={counts}", flush=True)
            return 0 if count_imu and all(counts.values()) else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except OSError as error:
        raise SystemExit(str(error)) from error
