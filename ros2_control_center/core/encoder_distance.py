"""Encoder distance feedback for the existing L1-gated drive; no CAN code."""
import math

CIRCUMFERENCE = math.pi*.05
DEFAULTS = {'max_speed_mps': .25, 'acceleration_mps2': .4,
            'kp': 2., 'ki': .1, 'kd': .25, 'cross_track_limit_m': .05,
            'min_speed_mps': .03, 'cross_kp': 2., 'cross_kd': .25,
            'cross_max_speed_mps': .08}


class DistanceMove:
    def __init__(self, direction, yaw, now, rows, settings):
        self.direction,self.yaw,self.started=direction,yaw,now
        self.settings=validate(settings)
        self.baseline={ch:row['relative_counts'] for ch,row in rows.items()}
        self.segments={ch:row['segment'] for ch,row in rows.items()}
        self.last_progress=0.;self.progress_at=now
        self.x=self.y=0.
        self.previous_time=now;self.previous_along=0.
        self.velocity=0.;self.integral=0.;self.speed=0.
        self.previous_cross=0.;self.cross_velocity=0.;self.cross_speed=0.
        self.last_error=self.settings['distance_m']

    def command(self, rows, yaw, gyro, now):
        p=self.settings
        for ch in (1,2):
            if rows[ch]['segment']!=self.segments[ch]:
                return None,'aborted: encoder reference reset'
        heading=math.remainder(yaw-self.yaw,math.tau)
        if abs(heading)>.03:
            return None,f'aborted: heading changed ({math.degrees(heading):.2f}deg, limit=1.72deg); encoder offsets are not calibrated'
        self.x=(rows[2]['relative_counts']-self.baseline[2])/65536*CIRCUMFERENCE*p['forward_sign']
        self.y=(rows[1]['relative_counts']-self.baseline[1])/65536*CIRCUMFERENCE*p['left_sign']
        dx,dy=self.direction
        along=dx*self.x+dy*self.y
        cross=-dy*self.x+dx*self.y
        remaining=p['distance_m']-along
        if along<-.005:
            return None,f'aborted: wrong direction / 逆方向移動 (along={along*1000:.1f}mm, limit=-5mm)'
        if abs(cross)>p['cross_track_limit_m']:
            return None,f'aborted: cross-track limit / 横ずれ超過 (cross={cross*1000:.1f}mm, limit={p["cross_track_limit_m"]*1000:.1f}mm)'
        if remaining<-.02:
            return None,'aborted: distance overshoot'
        if remaining<=.005 and abs(cross)<=.005:
            return None,f'complete: encoder measured {along:.3f} m, cross={cross*1000:.1f}mm'
        error=math.hypot(max(0.,remaining),cross)
        if self.last_error-error>=.001:
            self.last_error=error;self.progress_at=now
        if now-self.progress_at>1.:
            return None,(f'aborted: encoder stall / 進捗停止 (travel={along*1000:.1f}mm, '
                         f'remaining={remaining*1000:.1f}mm, command={self.speed*1000:.1f}mm/s, '
                         f'cross={cross*1000:.1f}mm, lateral_command={self.cross_speed*1000:.1f}mm/s, '
                         f'no_progress={now-self.progress_at:.2f}s)')
        if now-self.started>p['distance_m']/.01+5:
            return None,(f'aborted: time limit / 移動時間超過 (elapsed={now-self.started:.1f}s, '
                         f'travel={along*1000:.1f}mm, remaining={remaining*1000:.1f}mm)')
        dt=now-self.previous_time
        if not math.isfinite(dt) or dt<0 or dt>.2:
            return None,'aborted: control loop delayed'
        if dt>0:
            # Derivative on measured travel avoids a kick when starting a target.
            measured=(along-self.previous_along)/dt
            self.velocity+=dt/(.08+dt)*(measured-self.velocity)
            self.cross_velocity+=dt/(.08+dt)*((cross-self.previous_cross)/dt-self.cross_velocity)
            limit=min(p['max_speed_mps'],math.sqrt(2*p['acceleration_mps2']*max(0.,remaining)))
            candidate=min(.05/max(p['ki'],1e-9),self.integral+remaining*dt)
            output=p['kp']*remaining+p['ki']*candidate-p['kd']*self.velocity
            # Freeze integral while saturated, including acceleration limiting.
            if p['min_speed_mps']<output<min(limit,self.speed+p['acceleration_mps2']*dt):
                self.integral=candidate
            output=p['kp']*remaining+p['ki']*self.integral-p['kd']*self.velocity
            # Compensate low-speed stiction only while asking to move forward.
            # Negative PID output remains a brake request, never a forced creep.
            target=min(limit,max(p['min_speed_mps'],output)) if output>0 else 0.
            if remaining<=.005:target=0.
            # A perpendicular PD loop holds the line through the starting point.
            lateral=-p['cross_kp']*cross-p['cross_kd']*self.cross_velocity
            lateral_cap=min(p['cross_max_speed_mps'],p['max_speed_mps'])
            if abs(cross)<=.003:
                lateral=0.  # Avoid chasing quantization/noise around the line.
            elif lateral*cross<0 and abs(lateral)<p['min_speed_mps']:
                lateral=math.copysign(p['min_speed_mps'],lateral)
            lateral=max(-lateral_cap,min(lateral_cap,lateral))
            target=min(target,math.sqrt(max(0.,p['max_speed_mps']**2-lateral**2)))
            # Limit the change in the full translation vector, including corrections.
            step=p['acceleration_mps2']*dt
            change=math.hypot(target-self.speed,lateral-self.cross_speed)
            fraction=min(1.,step/change) if change else 1.
            self.speed+=(target-self.speed)*fraction
            self.cross_speed+=(lateral-self.cross_speed)*fraction
            self.previous_time=now;self.previous_along=along;self.previous_cross=cross
        return (self.speed*dx-self.cross_speed*dy,self.speed*dy+self.cross_speed*dx,
                max(-.08,min(.08,-1.5*heading-.2*gyro))),None


def validate(settings):
    if (not isinstance(settings,dict) or type(settings.get('distance_m')) not in (int,float)
            or not math.isfinite(settings['distance_m']) or not .01<=settings['distance_m']<=2.):
        raise ValueError('移動距離は10〜2000mmです。')
    if any(type(settings.get(k)) is not int or settings[k] not in (-1,1) for k in ('forward_sign','left_sign')):
        raise ValueError('前進・左移動時のエンコーダー増減方向を確認して設定してください。')
    result=DEFAULTS|settings
    for key,low,high in [('max_speed_mps',.03,.5),('acceleration_mps2',.05,1.),
                         ('kp',.1,10.),('ki',0.,5.),('kd',0.,2.),('cross_track_limit_m',.005,.1),
                         ('min_speed_mps',0.,.1),('cross_kp',0.,10.),('cross_kd',0.,2.),
                         ('cross_max_speed_mps',0.,.15)]:
        value=result[key]
        if type(value) not in (int,float) or not math.isfinite(value) or not low<=value<=high:
            raise ValueError(f'{key} は {low}〜{high} の範囲で設定してください。')
    if result['min_speed_mps']>result['max_speed_mps']:
        raise ValueError('最低移動速度は最大速度以下にしてください。')
    return result
