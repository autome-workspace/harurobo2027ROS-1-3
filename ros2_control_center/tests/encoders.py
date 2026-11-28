"""Encoder GUI states without any physical CAN traffic."""
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from PySide6.QtWidgets import QApplication
from core.config_manager import Config
from core.controller import Controller
from ui.main_window import MainWindow

app=QApplication([])
with tempfile.TemporaryDirectory() as folder:
    c=Controller(Config(Path(folder)/'local.yaml'))
    c.timer.stop();c.processes.timer.stop();c.can.close();c.encoder_can.close()
    while not c.can.idle():app.processEvents()
    c.mode='real';c.snapshot=time.monotonic();c.live={'/encoders/state'}
    c.processes.jobs['encoders']=SimpleNamespace(stopping=0)
    c.started['encoders']=time.monotonic()
    c.encoders={'can_connected':True,'expected_devices':2,'channels':[
        {'channel':0,'fresh':True,'angle_deg':90.,'relative_turns':.25,'rx_hz':200.,'fault':False,'synced':False},
        {'channel':1,'fresh':True,'angle_deg':180.,'relative_turns':-.5,'rx_hz':200.,'fault':True,'synced':True}]}
    c.encoder_received_at=time.monotonic()
    window=MainWindow(c);window.refresh()
    assert '90.0000°' in window.encoder_status.text() and '180.0000°' in window.encoder_status.text()
    assert c.status('encoders')==('2台受信・異常履歴あり','warning')
    c.encoders['channels'][1].update(fault_level='info',fault_detail=1,fault_text='再起動履歴のみ（受信正常）')
    window.refresh()
    assert c.status('encoders')==('2台受信中（再起動履歴のみ）','ok')
    assert '再起動履歴のみ' in window.encoder_status.text()
    c.encoders['channels'][1].update(fault_level='warning',fault_detail=0x101,fault_text='計数サンプリング遅延')
    assert c.status('encoders')[1]=='warning'
    c.encoders['channels'][1].update(fault=False,fault_level='ok',fault_detail=0)
    assert c.status('encoders')==('2台受信中','ok')
    c.encoders['channels'][1]['fresh']=False
    window.refresh()
    assert c.status('encoders')[1]=='warning'
    assert 'CH 1：受信途絶' in window.encoder_status.text()
    c.encoders['can_connected']=False;c.encoders['error']='can0 DOWN'
    window.refresh();assert c.status('encoders')==('CANの有効化待ち','warning')
    assert 'CANの有効化待ち' in window.device_encoder_status.text()
    # A diagnostic viewer can start even while the encoder row warns.
    c.encoders['can_connected']=True
    c.snapshot=time.monotonic()
    started=[]
    c.processes.start=lambda key,*args:started.append(key)
    c.pending=['encoder_view']
    c.advance()
    assert started==['encoder_view'] and not c.pending
    c.processes.jobs.clear();window.close()
print('PASS: two encoder values, startup history, stale channel and CAN disconnect GUI states')
