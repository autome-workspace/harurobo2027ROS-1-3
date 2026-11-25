#!/usr/bin/env python3
"""Read-only robot status dashboard (tkinter + rclpy). No CAN/serial access.

Subscribes: /diagnostics, /imu/data, /cmd_vel/manual, /manual/armed,
/cmd_vel/manual/heading_preview. Displays live values, freshness and
per-node diagnostic levels. Never publishes anything.
"""
import math
import time
import tkinter as tk
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

TAU = 2.0 * math.pi
IMU_TIMEOUT = 0.100
CMD_TIMEOUT = 0.150

LEVEL_COLORS = {0: '#1a7f37', 1: '#b58900', 2: '#c0392b', 3: '#a05a2c'}


class StatusNode(Node):
    def __init__(self):
        super().__init__('harurobo_status_gui')
        self.diagnostics = {}
        self.imu = {'t': None, 'yaw': None, 'gyro_z': None, 'accel': None}
        self.imu_times = deque(maxlen=3000)
        self.cmd = {'t': None, 'vx': None, 'vy': None, 'wz': None}
        self.preview = {'t': None, 'vx': None, 'vy': None, 'wz': None}
        self.armed = None
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diag, 20)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/cmd_vel/manual', self.on_cmd, 5)
        self.create_subscription(TwistStamped, '/cmd_vel/manual/heading_preview',
                                 self.on_preview, 5)
        self.create_subscription(Bool, '/manual/armed', self.on_armed, 5)

    def on_diag(self, msg):
        for status in msg.status:
            self.diagnostics[status.name] = (status.level, status.message, status.hardware_id)

    def on_imu(self, msg):
        q = msg.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        accel = math.sqrt(msg.linear_acceleration.x ** 2 + msg.linear_acceleration.y ** 2 +
                          msg.linear_acceleration.z ** 2)
        self.imu = {'t': time.monotonic(), 'yaw': math.degrees(yaw),
                   'gyro_z': msg.angular_velocity.z, 'accel': accel}
        self.imu_times.append(self.imu['t'])

    def on_cmd(self, msg):
        self.cmd = {'t': time.monotonic(), 'vx': msg.twist.linear.x, 'vy': msg.twist.linear.y,
                    'wz': msg.twist.angular.z}

    def on_preview(self, msg):
        self.preview = {'t': time.monotonic(), 'vx': msg.twist.linear.x,
                        'vy': msg.twist.linear.y, 'wz': msg.twist.angular.z}

    def on_armed(self, msg):
        self.armed = msg.data


def fmt(value, digits=3):
    return '--' if value is None else f'{value:+.{digits}f}'


def age_ms(timestamp):
    return None if timestamp is None else (time.monotonic() - timestamp) * 1000.0


class Dashboard:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        root.title('春ロボ ステータス (read-only)')
        root.configure(bg='#101418')
        self.rows = []
        self.build()
        self.update()

    def build(self):
        def label(text, row, col, color='#d8dee9', size=10, bold=False):
            widget = tk.Label(self.root, text=text, fg=color, bg='#101418',
                              font=('Noto Sans CJK JP', size, 'bold' if bold else 'normal'))
            widget.grid(row=row, column=col, sticky='w', padx=8, pady=2)
            return widget

        def section(title, row):
            label(title, row, 0, '#7fb3ff', 11, bold=True)
            return row + 1

        row = 0
        row = section('IMU  (/imu/data)', row)
        self.imu_yaw = label('yaw [deg]      --', row, 0); row += 1
        self.imu_gyro = label('gyro_z [rad/s] --', row, 0); row += 1
        self.imu_accel = label('|accel| [m/s2] --', row, 0); row += 1
        self.imu_rate = label('rate [Hz]      --', row, 0); row += 1
        self.imu_age = label('last age [ms]  --', row, 0); row += 1

        row = section('手動指令 (/cmd_vel/manual + /manual/armed)', row)
        self.armed = label('許可           --', row, 0); row += 1
        self.cmd_vx = label('vx [m/s]       --', row, 0); row += 1
        self.cmd_vy = label('vy [m/s]       --', row, 0); row += 1
        self.cmd_wz = label('wz [rad/s]     --', row, 0); row += 1

        row = section('ヨー保持プレビュー (/cmd_vel/manual/heading_preview)', row)
        self.prev_vx = label('vx [m/s]       --', row, 0); row += 1
        self.prev_wz = label('wz 補正込み     --', row, 0); row += 1

        row = section('診断 (/diagnostics)', row)
        self.diag_anchor = row

    def set_color(self, widget, color):
        widget.configure(fg=color)

    def update(self):
        deadline = time.monotonic() + 0.02
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.001)
        now = time.monotonic()
        imu = self.node.imu
        self.imu_yaw.config(text=f'yaw [deg]      {fmt(imu["yaw"], 1)}')
        self.imu_gyro.config(text=f'gyro_z [rad/s] {fmt(imu["gyro_z"])}')
        self.imu_accel.config(text=f'|accel| [m/s2] {fmt(imu["accel"], 2)}')
        recent = [t for t in self.node.imu_times if now - t <= 1.0]
        self.imu_rate.config(text=f'rate [Hz]      '
                                  f'{"--" if not recent else len(recent)}')
        imu_age = age_ms(imu['t'])
        fresh = imu_age is not None and imu_age < IMU_TIMEOUT * 1000
        self.imu_age.config(text=f'last age [ms]  {fmt(imu_age, 0)}')
        self.set_color(self.imu_age, '#1a7f37' if fresh else '#c0392b')

        self.armed.config(text='許可           ' + ('--' if self.node.armed is None else
                                                   ('あり' if self.node.armed else 'なし')))
        self.set_color(self.armed, '#1a7f37' if self.node.armed else '#8a9199')
        cmd = self.node.cmd
        self.cmd_vx.config(text=f'vx [m/s]       {fmt(cmd["vx"])}')
        self.cmd_vy.config(text=f'vy [m/s]       {fmt(cmd["vy"])}')
        self.cmd_wz.config(text=f'wz [rad/s]     {fmt(cmd["wz"])}')
        cmd_age = age_ms(cmd['t'])
        if cmd_age is not None:
            self.set_color(self.cmd_wz, '#1a7f37' if cmd_age < CMD_TIMEOUT * 1000 else '#c0392b')

        prev = self.node.preview
        self.prev_vx.config(text=f'vx [m/s]       {fmt(prev["vx"])}')
        self.prev_wz.config(text=f'wz 補正込み     {fmt(prev["wz"])}')

        for widget in self.rows:
            widget.destroy()
        self.rows = []
        row = self.diag_anchor
        for name, (level, message, hardware) in sorted(self.node.diagnostics.items()):
            color = LEVEL_COLORS.get(level, '#8a9199')
            widget = tk.Label(self.root, text=f'{name}: {message}',
                              fg=color, bg='#101418', anchor='w', justify='left',
                              font=('Noto Sans CJK JP', 9))
            widget.grid(row=row, column=0, sticky='we', padx=8, pady=1)
            self.rows.append(widget)
            row += 1
        self.root.after(50, self.update)


def main():
    rclpy.init()
    node = StatusNode()
    root = tk.Tk()
    Dashboard(root, node)
    try:
        root.mainloop()
    finally:
        root.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
