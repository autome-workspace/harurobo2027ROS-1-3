from types import SimpleNamespace

import pytest
from amt102_can.diagnostics import FaultReader
from amt102_can.protocol import CAN_FRAME
from amt102_can.receiver import CanReceiver


def reply(ch,seq,mask,result=0):
    return bytes([ch,result,5])+mask.to_bytes(4,'big')+bytes([seq])


def test_startup_history_and_real_faults_remain_distinct():
    reader=FaultReader()
    reader.observe(1,True)
    query=reader.request([1],0.)
    assert reader.describe(1,True,0.)['fault_level']=='warning'
    assert reader.reply(reply(1,query[-1],1),.01)
    state=reader.describe(1,True,.02)
    assert state['fault_level']=='info' and '再起動履歴のみ' in state['fault_text']
    # Fault bit stays set when an additional fault occurs; refresh must discover it.
    query=reader.request([1],1.1)
    assert reader.reply(reply(1,query[-1],0x101),1.11)
    state=reader.describe(1,True,1.12)
    assert state['fault_level']=='warning' and '計数サンプリング遅延' in state['fault_text']
    assert reader.describe(1,True,3.)['fault_detail'] is None
    assert reader.describe(1,False,3.)['fault_level']=='ok'


def test_reply_correlation_timeout_transition_and_boot():
    reader=FaultReader()
    reader.observe(1,True);reader.observe(2,True)
    query=reader.request([1,2],0.)
    assert reader.request([1,2],.01) is None
    assert not reader.reply(reply(2,query[-1],1),.01)
    assert not reader.reply(reply(1,(query[-1]+1)%256,1),.01)
    bad=bytearray(reply(1,query[-1],1));bad[1]=0x40
    assert not reader.reply(bytes(bad),.01)
    assert not reader.reply(reply(1,query[-1],1),.25)
    second=reader.request([1,2],.26)
    assert second[0]==2
    assert reader.reply(reply(2,second[-1],1),.27)
    reader.observe(2,False);reader.observe(2,True)
    assert reader.describe(2,True,.28)['fault_detail'] is None
    reader.reset()
    assert reader.describe(2,True,.29)['fault_level']=='warning'
    reader.observe(2,True)
    third=reader.request([2],.3)
    assert third[-1]!=second[-1]
    assert reader.reply(reply(2,third[-1],1,result=2),.31)
    assert reader.describe(2,True,.32)['fault_detail'] is None


def test_only_fault_read_operation_can_be_transmitted():
    receiver=CanReceiver()
    packets=[]
    receiver.socket=SimpleNamespace(send=lambda packet:packets.append(packet) or len(packet))
    receiver.request_fault(bytes([2,0,5,0,0,0,0,7]))
    cid,dlc,data=CAN_FRAME.unpack(packets[0])
    assert cid==0x480 and dlc==8 and data==bytes([2,0,5,0,0,0,0,7])
    for operation in (0x40,0x80,0xc0):
        with pytest.raises(ValueError):receiver.request_fault(bytes([2,operation,5,0,0,0,0,7]))
    with pytest.raises(ValueError):receiver.request_fault(bytes([2,0,32,0,0,0,0,7]))
    assert len(packets)==1
