"""Closed-loop simulation with encoder quantization and a lagging drive plant."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.encoder_distance import DistanceMove,CIRCUMFERENCE,validate

for direction in ((1,0),(-1,0),(0,1),(0,-1)):
    for goal in (.01,.1,.3,1.,2.):
        for gain in (.7,1.,1.3):
            rows={ch:{'relative_counts':0,'segment':1} for ch in (1,2)}
            settings={'distance_m':goal,'forward_sign':-1,'left_sign':1}
            move=DistanceMove(direction,0.,0.,rows,settings)
            travel=velocity=previous=peak=0.
            for tick in range(1,4000):
                now=tick*.01
                rows[2]['relative_counts']=round(-direction[0]*travel/CIRCUMFERENCE*65536)
                rows[1]['relative_counts']=round(direction[1]*travel/CIRCUMFERENCE*65536)
                values,reason=move.command(rows,0.,0.,now)
                if reason:
                    assert reason.startswith('complete'),(goal,gain,reason)
                    break
                speed=values[0]*direction[0]+values[1]*direction[1]
                assert 0<=speed<=.25+1e-9
                assert abs(speed-previous)<=.4*.01+1e-9
                assert move.integral*.1<=.05+1e-9
                previous=speed;peak=max(peak,speed)
                velocity+=.01/(.1+.01)*(gain*speed-velocity)
                travel+=velocity*.01
            else:raise AssertionError('simulation did not finish')
            # Include residual coasting after the stop request in this plant.
            for _ in range(100):
                velocity+=.01/(.1+.01)*(0-velocity);travel+=velocity*.01
            assert abs(travel-goal)<.01,(goal,gain,travel)
            if goal>=.3:assert peak>.2
            if goal==.3 and gain==1.:assert now<3.
for key,value in [('max_speed_mps',float('nan')),('ki',-1),('kd',float('inf'))]:
    try:validate(settings|{key:value})
    except ValueError:pass
    else:raise AssertionError('invalid tuning accepted')
rows={ch:{'relative_counts':0,'segment':1} for ch in (1,2)}
move=DistanceMove((1,0),0.,0.,rows,settings)
assert 'delayed' in move.command(rows,0.,0.,.3)[1]
print('PASS: 60 quantized-encoder PID simulations, drive lag/gain variation, four directions, speed/acceleration/antiwindup bounds, delayed-loop abort; no motors')
