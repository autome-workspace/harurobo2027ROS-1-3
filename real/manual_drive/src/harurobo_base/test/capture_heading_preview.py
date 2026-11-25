"""Record preview/command/latest IMU only; never access serial or CAN."""
import argparse
import json
import math
import time

import rclpy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import qos_profile_sensor_data


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True)
    args=p.parse_args()
    rclpy.init()
    node=rclpy.create_node('capture_heading_preview')
    latest={}
    count=0
    corrections=[]
    with open(args.output,'w') as output:
        def imu(msg):
            q=msg.orientation
            latest['imu_t']=time.monotonic()
            latest['yaw']=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            latest['gyro_z']=msg.angular_velocity.z
        def manual(msg):
            latest['vx']=msg.twist.linear.x
            latest['vy']=msg.twist.linear.y
            latest['manual_wz']=msg.twist.angular.z
        def preview(msg):
            nonlocal count
            row=dict(latest,t=time.monotonic(),out_vx=msg.twist.linear.x,
                     out_vy=msg.twist.linear.y,out_wz=msg.twist.angular.z)
            print(json.dumps(row),file=output)
            count+=1
            if abs(latest.get('manual_wz',0))<1e-6 and abs(msg.twist.angular.z)>1e-4:
                corrections.append(msg.twist.angular.z)
        def diag(msg):
            for status in msg.status:
                if 'heading preview' in status.name:
                    latest['preview_diagnostic']=status.message
        node.create_subscription(Imu,'/imu/data',imu,qos_profile_sensor_data)
        node.create_subscription(TwistStamped,'/cmd_vel/manual',manual,1)
        node.create_subscription(TwistStamped,'/cmd_vel/manual/heading_preview',preview,1)
        node.create_subscription(DiagnosticArray,'/diagnostics',diag,1)
        print('CAPTURE READY: 120 seconds; no drive connection',flush=True)
        deadline=time.monotonic()+120
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01)
    print('preview samples:',count,flush=True)
    print('hold-only correction samples:',len(corrections),flush=True)
    if corrections:
        print('hold-only min/max rad/s:',min(corrections),max(corrections),flush=True)
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
