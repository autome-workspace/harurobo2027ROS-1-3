import math
import struct
from types import SimpleNamespace

import pytest
from amt102_can.protocol import CAN_FRAME, ChannelTracker, decode_position, position_delta
from amt102_can.receiver import CanReceiver


def data(position, stamp=0, flags=0):
    return bytes([flags])+position.to_bytes(3,'big')+stamp.to_bytes(2,'big')


def test_document_example_and_rejection():
    example = decode_position(0x360,bytes.fromhex('00 01 00 00 83 E8'))
    assert example.channel==0 and example.position_u24==65536
    assert not example.fault and example.synced and example.time_us_low15==1000
    assert decode_position(0x367,data(0,flags=1)).channel==7
    for cid,payload in [(0x35f,data(0)),(0x368,data(0)),(0x80000360,data(0)),
                        (0x40000360,data(0)),(0x360,data(0)[:-1]),(0x360,data(0,flags=2))]:
        with pytest.raises(ValueError):decode_position(cid,payload)


def test_two_channels_wrap_gap_and_boot():
    tracker=ChannelTracker(.5)
    tracker.update(decode_position(0x360,data(0xfffff0)),0.)
    tracker.update(decode_position(0x361,data(0x10)),0.)
    tracker.update(decode_position(0x360,data(0x10,stamp=0x8001)),.005)
    tracker.update(decode_position(0x361,data(0xfffff0)),.005)
    rows=tracker.snapshot(.01)
    assert [r['relative_counts'] for r in rows]==[32,-32]
    assert all(r['fresh'] and abs(r['rx_hz']-200)<1e-5 for r in rows)
    assert math.isclose(rows[0]['angle_deg'],16*360/65536)
    assert not tracker.snapshot(.6)[0]['fresh']
    tracker.update(decode_position(0x360,data(0x20)),.7)
    assert tracker.snapshot(.7)[0]['relative_counts']==0
    assert tracker.snapshot(.7)[0]['segment']==2
    tracker.boot()
    assert tracker.snapshot(.7)==[] and tracker.boot_notifications==1
    assert position_delta(0,0xffffff)==1
    assert position_delta(0xffffff,0)==-1


def test_receiver_only_reads_and_filters(monkeypatch):
    packets=[CAN_FRAME.pack(0x360,6,data(65536)),CAN_FRAME.pack(0x80000360,6,data(0)),
             CAN_FRAME.pack(0x40000360,6,data(0)),CAN_FRAME.pack(0x360,9,b'12345678')]
    class FakeSocket:
        def __init__(self,*args):self.closed=False;self.options=[]
        def setsockopt(self,*args):self.options.append(args)
        def bind(self,address):assert address==('can0',)
        def setblocking(self,value):assert value is False
        def recv(self,size):
            if packets:return packets.pop(0)
            raise BlockingIOError()
        def send(self,*args):raise AssertionError('CAN transmission prohibited')
        def close(self):self.closed=True
    monkeypatch.setattr('amt102_can.receiver.socket.socket',FakeSocket)
    receiver=CanReceiver('can0')
    monkeypatch.setattr(receiver,'is_up',lambda:True)
    receiver.connect()
    sock=receiver.socket
    filters=list(struct.iter_unpack('=II',sock.options[0][2]))
    # A mask containing CAN_ERR_FLAG routes to the kernel error-filter list.
    assert all(not mask & 0x20000000 for _,mask in filters)
    for channel in range(8):
        assert any(((0x360+channel)&mask)==(cid&mask) for cid,mask in filters)
    for flagged in (0x80000361,0x40000361):
        assert not any((flagged&mask)==(cid&mask) for cid,mask in filters)
    assert receiver.receive()==[(0x360,data(65536))]
    receiver.close();assert sock.closed
    monkeypatch.setattr(receiver,'is_up',lambda:False)
    with pytest.raises(OSError):receiver.connect()
