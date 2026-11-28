from pathlib import Path
import tempfile

import yaml
from PySide6.QtWidgets import QDialog,QVBoxLayout,QFormLayout,QLabel,QDoubleSpinBox,QComboBox,QCheckBox,QDialogButtonBox,QMessageBox
from core.encoder_distance import validate,DEFAULTS


class EncoderDistanceSettings(QDialog):
    def __init__(self,controller,parent=None):
        super().__init__(parent);self.controller=controller
        self.setWindowTitle('エンコーダー指定距離移動')
        self.path=controller.config.path.parent/'encoder_distance.yaml'
        p=yaml.safe_load(self.path.read_text()) if self.path.exists() else {}
        layout=QVBoxLayout(self)
        hint=QLabel('CH2：前後、CH1：左右。直径50mm・直結、1回転157.08mm。\n'
                    '足回りを停止した状態で手押しし、前進・左移動時の相対回転数の増減を確認してください。\n'
                    '距離PID＋横ずれPD＋加減速制限で移動します。L1保持＋十字キーで開始、L1解除・スティック操作で中止。\n'
                    '回転を伴う移動は取付位置の補正が必要なため、姿勢変化時に中止します。')
        hint.setWordWrap(True);layout.addWidget(hint);form=QFormLayout();layout.addLayout(form)
        self.enabled=QCheckBox('足回りの十字キー移動にエンコーダーを使う')
        self.enabled.setChecked(controller.config.variables.get('MOTOR_ENCODER_DISTANCE',False));form.addRow(self.enabled)
        self.distance=QDoubleSpinBox(minimum=10,maximum=2000,value=p.get('distance_m',.1)*1000,suffix=' mm')
        form.addRow('移動距離',self.distance)
        self.tuning={}
        for key,label,low,high,scale in [
            ('max_speed_mps','最大速度（mm/s）',30,500,1000),
            ('min_speed_mps','最低移動速度（mm/s・0で無効）',0,100,1000),
            ('acceleration_mps2','加減速度（mm/s²）',50,1000,1000),
            ('cross_track_limit_m','横ずれ許容値（mm）',5,100,1000),
            ('cross_max_speed_mps','横ずれ補正の最大速度（mm/s）',0,150,1000),
            ('cross_kp','横ずれ P（1/s）',0,10,1),('cross_kd','横ずれ D',0,2,1),
            ('kp','距離 P（1/s）',.1,10,1),('ki','距離 I（1/s²）',0,5,1),('kd','速度減衰 D',0,2,1)]:
            field=QDoubleSpinBox();field.setRange(low,high);field.setDecimals(3 if scale==1 else 0)
            field.setSingleStep(.05 if scale==1 else 10);field.setValue(p.get(key,DEFAULTS[key])*scale)
            self.tuning[key]=(field,scale);form.addRow(label,field)
        self.signs={}
        for key,label in [('forward_sign','前進時のCH2'),('left_sign','左移動時のCH1')]:
            combo=QComboBox();combo.addItem('未確認',0);combo.addItem('回転数が増える',1);combo.addItem('回転数が減る',-1)
            combo.setCurrentIndex(combo.findData(p.get(key,0)));self.signs[key]=combo;form.addRow(label,combo)
        buttons=QDialogButtonBox(QDialogButtonBox.StandardButton.Save|QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.save);buttons.rejected.connect(self.reject);layout.addWidget(buttons)

    def save(self):
        try:
            c=self.controller
            if 'motor' in c.processes.jobs or c.pending or c.shutdown:raise ValueError('足回りと起動処理を停止してから設定してください。')
            settings={'distance_m':self.distance.value()/1000,**{k:w.currentData() for k,w in self.signs.items()}}
            settings.update({k:w.value()/scale for k,(w,scale) in self.tuning.items()})
            if self.enabled.isChecked():validate(settings)
            self.path.parent.mkdir(parents=True,exist_ok=True)
            with tempfile.NamedTemporaryFile(mode='w',dir=self.path.parent,delete=False) as f:
                yaml.safe_dump(settings,f);temporary=Path(f.name)
            temporary.replace(self.path)
            c.config.save_variables({'MOTOR_ENCODER_DISTANCE':self.enabled.isChecked(),
                                     **({'MOTOR_AUTO_APPROACH':False} if self.enabled.isChecked() else {})})
            self.accept()
        except (ValueError,OSError) as error:QMessageBox.warning(self,'指定距離移動設定',str(error))
