"""Verify changing values through the real ROS monitor into Qt, without CAN."""
import os
from pathlib import Path
import signal
import sys
import tempfile
import time
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
APP=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(APP))
from PySide6.QtWidgets import QApplication
from core.config_manager import Config
from core.controller import Controller
from ui.main_window import MainWindow

app=QApplication([])

def wait(test, timeout=12):
    end=time.monotonic()+timeout
    while not test():
        app.processEvents()
        if time.monotonic()>end:raise AssertionError('timeout')
        time.sleep(.005)

with tempfile.TemporaryDirectory() as directory:
    folder=Path(directory)
    fake=folder/'publisher.py'
    fake.write_text('''import rclpy
from amt102_can.encoder_node import EncoderNode
class FakeReceiver:
    interface='test-only'
    def __init__(self):self.socket=object();self.counter=0
    def is_up(self):return True
    def close(self):self.socket=None
    def receive(self):
        self.counter+=1
        def packet(value):return bytes([0])+int(value&0xffffff).to_bytes(3,'big')+bytes([0,0])
        return [(0x360+ch,packet(self.counter*1024*(ch+1))) for ch in range(8)]
rclpy.init()
node=EncoderNode();node.receiver=FakeReceiver()
try:rclpy.spin(node)
except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
finally:node.destroy_node();rclpy.try_shutdown()
''')
    config=Config(folder/'local.yaml');config.domains['real']=90
    controller=Controller(config);controller.mode='real'
    window=MainWindow(controller);window.show()
    texts=[]
    controller.encoder_changed.connect(lambda:texts.append((time.monotonic(),window.encoder_status.text())))
    controller.processes.start('encoders',config.wrapped([
        '/usr/bin/python3','-u',str(fake)],['install/setup.bash']),config.environment('real'))
    controller.started['encoders']=time.monotonic()
    source=controller.processes.jobs['encoders'].process
    try:
        wait(lambda:controller.encoder_available() and len(texts)>10)
        begin=time.monotonic()
        while time.monotonic()-begin<1.2:
            app.processEvents();time.sleep(.005)
        recent=[text for at,text in texts if at>=begin]
        assert len(set(recent))>=10,(len(recent),len(set(recent)),recent)
        assert 'CH 1' in recent[-1] and 'CH 2' in recent[-1] and 'CH 7' in recent[-1]
        assert len(controller.encoders['channels'])==8
        # Paused input must not look like a live, unchanged encoder.
        source.send_signal(signal.SIGSTOP)
        wait(lambda:not controller.encoder_available(),timeout=3)
        window.update_encoder_display()
        assert '受信待ち' in window.encoder_status.text()
        source.send_signal(signal.SIGCONT)
        wait(controller.encoder_available)
        print(f'PASS: ROS→monitor→Qt angle updates ({len(set(recent))} distinct values / 1.2s), eight-channel large JSON, stale input and recovery; no CAN opened')
    finally:
        if source.poll() is None:source.send_signal(signal.SIGCONT)
        window.close()
        wait(lambda:not controller.processes.jobs)
        wait(lambda:controller.can.idle() and controller.encoder_can.idle())
