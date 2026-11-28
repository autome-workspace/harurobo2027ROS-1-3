"""Receive-only AMT102-CAN position codec, per 2026-11-28 protocol."""
from collections import deque
from dataclasses import asdict, dataclass
import math
import struct

CAN_FRAME = struct.Struct('=IB3x8s')
POSITION_BASE = 0x360
POSITION_LAST = 0x367
BOOT_ID = 0x018
SCALE = 65536


@dataclass(frozen=True)
class Position:
    channel: int
    fault: bool
    position_u24: int
    synced: bool
    time_us_low15: int


def decode_position(can_id, data):
    if not POSITION_BASE <= can_id <= POSITION_LAST or len(data) != 6 or data[0] & 0xfe:
        raise ValueError('AMT102-CAN位置フレームのID・DLC・予約bitが不正です。')
    stamp = int.from_bytes(data[4:6], 'big')
    return Position(can_id-POSITION_BASE, bool(data[0] & 1),
                    int.from_bytes(data[1:4], 'big'), bool(stamp & 0x8000), stamp & 0x7fff)


def position_delta(current, previous):
    return ((current-previous+0x800000) & 0xffffff)-0x800000


class ChannelTracker:
    def __init__(self, stale_timeout=.5):
        self.stale_timeout = stale_timeout
        self.channels = {}
        self.boot_notifications = 0

    def boot(self):
        # Boot notification has no channel: do not continue any old displacement.
        self.channels.clear()
        self.boot_notifications += 1

    def update(self, position, now, ros_stamp_ns=0):
        previous = self.channels.get(position.channel)
        fresh = previous is not None and 0 <= now-previous['received_at'] < self.stale_timeout
        total = (previous['relative_counts'] + position_delta(position.position_u24, previous['position_u24'])
                 if fresh else 0)
        times = previous['_times'] if fresh else deque(maxlen=256)
        times.append(now)
        state = asdict(position) | {
            'relative_counts': total, 'received_at': now, 'ros_stamp_ns': ros_stamp_ns,
            'frames': (previous['frames'] if previous else 0)+1,
            'segment': (previous['segment'] if previous else 1)+(1 if previous and not fresh else 0),
            '_times': times,
        }
        self.channels[position.channel] = state
        return state

    def snapshot(self, now):
        rows = []
        for channel, state in sorted(self.channels.items()):
            age = max(0., now-state['received_at'])
            times = state['_times']
            span = times[-1]-times[0]
            row = {key: value for key, value in state.items() if not key.startswith('_')}
            row.update(age=age, fresh=age<self.stale_timeout,
                       angle_deg=(state['position_u24'] % SCALE)*360./SCALE,
                       relative_turns=state['relative_counts']/SCALE,
                       relative_radians=state['relative_counts']*math.tau/SCALE,
                       rx_hz=((len(times)-1)/span if span>0 and age<self.stale_timeout else 0.))
            rows.append(row)
        return rows
