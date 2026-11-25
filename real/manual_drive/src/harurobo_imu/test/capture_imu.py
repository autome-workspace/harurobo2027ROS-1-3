"""Record IMU observations only; no serial open and no CAN access."""
import argparse
import json
import math
import statistics
import time

import rclpy
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import qos_profile_sensor_data


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=float,default=5)
    p.add_argument('--output',required=True)
    args=p.parse_args()
    if not .1<=args.seconds<=120:
        p.error('seconds must be .1..120')
    rclpy.init()
    node=rclpy.create_node('capture_imu_observations')
    samples=[]; statuses=[]
    def observe(m):
        q=m.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        samples.append(dict(t=time.monotonic(),stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                            yaw=yaw,gyro_z=m.angular_velocity.z,
                            accel=math.sqrt(sum(v*v for v in (m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z))),
                            qnorm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)))
    node.create_subscription(Imu,'/imu/data',observe,qos_profile_sensor_data)
    node.create_subscription(DiagnosticArray,'/diagnostics',statuses.append,1)
    end=time.monotonic()+args.seconds
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.01)
    with open(args.output,'w') as output:
        for row in samples:
            print(json.dumps(row),file=output)
    print('samples:',len(samples))
    if len(samples)>1:
        gaps=[b['t']-a['t'] for a,b in zip(samples,samples[1:])]
        print('Hz:',(len(samples)-1)/(samples[-1]['t']-samples[0]['t']),'max receive gap ms:',max(gaps)*1000)
        for key in ('accel','gyro_z','qnorm'):
            values=[s[key] for s in samples]
            print(key,'mean/min/max:',statistics.mean(values),min(values),max(values))
        unwrapped=[samples[0]['yaw']]
        for a,b in zip(samples,samples[1:]):
            unwrapped.append(unwrapped[-1]+math.remainder(b['yaw']-a['yaw'],2*math.pi))
        print('yaw change/range deg:',math.degrees(unwrapped[-1]-unwrapped[0]),math.degrees(max(unwrapped)-min(unwrapped)))
    if statuses:
        s=statuses[-1].status[0]
        print('diagnostics:',s.level,s.message,{v.key:v.value for v in s.values})
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
