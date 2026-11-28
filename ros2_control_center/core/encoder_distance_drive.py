"""Reuse existing motor drive and watchdog; substitute encoder distance feedback."""
import json
import os
from pathlib import Path
import sys
import threading
import time

import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'real/manual_drive/src/harurobo_base/test'))
import manual_drive as drive
from encoder_distance import DistanceMove,validate

settings_file=Path(os.environ.get('XDG_CONFIG_HOME',Path.home()/'.config'))/'ros2-control-center/encoder_distance.yaml'
settings=validate(yaml.safe_load(settings_file.read_text()))
collectors=[]


class EncoderDrive(drive.DpadDrive):
    def on_joy(self,axes,buttons,now):
        super().on_joy(axes,buttons,now)
        if len(buttons)>1 and buttons[1] and (self.move is not None or self.request is not None):
            self.finish('cancelled by circle button')
    def __init__(self,gate):
        super().__init__(gate)
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import String
        self.node=rclpy.create_node('encoder_distance_feedback')
        self.executor=SingleThreadedExecutor();self.executor.add_node(self.node)
        self.lock=threading.Lock();self.data={};self.received=0.;self.running=True
        def update(msg):
            try:
                data=json.loads(msg.data)
                if not isinstance(data,dict):return
                with self.lock:self.data=data;self.received=time.monotonic()
            except (ValueError,TypeError):pass
        self.node.create_subscription(String,'/encoders/state',update,qos_profile_sensor_data)
        self.thread=threading.Thread(target=self.spin,daemon=True);self.thread.start()
        collectors.append(self)

    def spin(self):
        import rclpy
        try:
            while self.running and rclpy.ok():self.executor.spin_once(timeout_sec=.05)
        except (rclpy.executors.ExternalShutdownException,RuntimeError):pass

    def command(self,now):
        if self.cancelled or (self.move is not None and self.gate.armed_at is None):
            self.finish('cancelled: '+(self.cancel_reason or getattr(self.gate,'last_stop_reason',None) or 'controller disarmed'))
        if self.gate.armed_at is None:self.request=None
        if self.move is None and self.request is None:return self.gate.wheels
        if self.imu is None or now-self.imu[0]>.15:
            age='missing' if self.imu is None else f'{now-self.imu[0]:.3f}s (limit=0.150s)'
            self.finish('aborted: IMU missing or stale: '+age);return self.gate.wheels
        with self.lock:data=self.data;received=self.received
        stamp=self.node.get_clock().now().nanoseconds
        try:
            rows={r['channel']:r for r in data.get('channels',[]) if r['channel'] in (1,2)}
            for r in rows.values():
                if any(type(r.get(k)) is not int for k in ('relative_counts','segment','ros_stamp_ns')):
                    raise ValueError('invalid encoder data')
        except (KeyError,ValueError,TypeError):
            self.finish('aborted: invalid encoder data');return self.gate.wheels
        valid=(data.get('can_connected') and now-received<.15 and set(rows)=={1,2})
        if valid:
            valid=all(r.get('fresh') and -.05<=(stamp-r['ros_stamp_ns'])/1e9<.15
                      and (r.get('fault') is False or (r.get('fault') is True and r.get('fault_detail')==1 and r.get('fault_level')=='info'))
                      for r in rows.values())
        if not valid or not data.get('stream_id'):
            details=[{'ch':ch,'fresh':r.get('fresh'),
                      'age_s':round((stamp-r['ros_stamp_ns'])/1e9,3),
                      'fault':r.get('fault'),'detail':r.get('fault_detail'),'level':r.get('fault_level')}
                     for ch,r in rows.items()]
            self.finish(f'aborted: encoder missing, stale or faulted; rx_age={now-received:.3f}s '
                        f'CAN={data.get("can_connected")} channels={details}');return self.gate.wheels
        _,yaw,gyro=self.imu
        epoch=(data['stream_id'],data.get('boot_notifications'))
        if self.move is not None and epoch!=self.move_epoch:
            self.finish('aborted: encoder restarted');return self.gate.wheels
        if self.move is None:
            self.move_epoch=epoch
            self.move=DistanceMove(self.request,yaw,now,rows,settings);self.request=None
            print(f"DPAD encoder PID: target={settings['distance_m']:.3f}m direction={self.move.direction} "
                  f"max_speed={settings['max_speed_mps']:.3f}m/s "
                  f"min_speed={settings['min_speed_mps']:.3f}m/s "
                  f"cross_P/D={settings['cross_kp']}/{settings['cross_kd']} "
                  f"cross_cap={settings['cross_max_speed_mps']:.3f}m/s "
                  f"P/I/D={settings['kp']}/{settings['ki']}/{settings['kd']}",flush=True)
        values,reason=self.move.command(rows,yaw,gyro,now)
        if reason:
            self.finish(reason);return self.gate.wheels
        x,y,w=values
        return drive.mix(x/(drive.FORWARD_METRES_PER_RAD*drive.WHEEL_CAP),
                         y/(drive.LEFT_METRES_PER_RAD*drive.WHEEL_CAP),w/drive.WHEEL_CAP)

    def observe(self,wheels,now):
        pass  # Position comes exclusively from the two AMT encoders.


drive.DpadDrive=EncoderDrive
try:drive.run()
finally:
    for collector in collectors:
        collector.running=False;collector.thread.join(timeout=1.)
        collector.executor.shutdown();collector.node.destroy_node()
