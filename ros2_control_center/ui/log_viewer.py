from collections import deque
from datetime import datetime
from pathlib import Path

from PySide6.QtGui import QColor, QTextCharFormat, QTextCursor
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QFileDialog, QHBoxLayout, QLineEdit, QMessageBox,
    QPlainTextEdit, QPushButton, QVBoxLayout, QWidget,
)

COLORS = {'INFO': '#bac9db', 'WARN': '#f0c16c', 'ERROR': '#ff898e'}


class LogView(QWidget):
    def __init__(self, programs):
        super().__init__()
        self.records = deque(maxlen=2000)
        self.pending = deque(maxlen=2000)
        self.names = {key: row['name'] for key, row in programs.items()} | {'_monitor': 'ROS 監視'}
        layout = QVBoxLayout(self)
        tools = QHBoxLayout()
        layout.addLayout(tools)
        self.filter = QComboBox()
        self.filter.addItem('すべて', None)
        for key, name in self.names.items():
            self.filter.addItem(name, key)
        tools.addWidget(self.filter)
        self.search = QLineEdit(placeholderText='キーワード検索')
        tools.addWidget(self.search, 1)
        self.auto = QCheckBox('自動スクロール')
        self.auto.setChecked(True)
        tools.addWidget(self.auto)
        for label, callback in [('クリア', self.clear), ('保存', self.save)]:
            button = QPushButton(label)
            button.clicked.connect(callback)
            tools.addWidget(button)
        self.view = QPlainTextEdit(readOnly=True)
        self.view.setMaximumBlockCount(2000)
        layout.addWidget(self.view)
        self.filter.currentIndexChanged.connect(self.rebuild)
        self.search.textChanged.connect(self.rebuild)

    def record(self, key, level, line):
        entry = (key, level, f'{datetime.now():%H:%M:%S}  {level}  {self.names.get(key, key)}  {line}')
        self.records.append(entry)
        self.pending.append(entry)

    def selected(self, entries):
        key = self.filter.currentData()
        word = self.search.text().casefold()
        return (entry for entry in entries if (key is None or key == entry[0]) and word in entry[2].casefold())

    def flush(self):
        if not self.pending:
            return
        self.render(self.pending)
        self.pending.clear()

    def render(self, entries):
        cursor = QTextCursor(self.view.document())
        cursor.movePosition(QTextCursor.MoveOperation.End)
        cursor.beginEditBlock()
        for _, level, line in self.selected(entries):
            if cursor.position():
                cursor.insertBlock()
            style = QTextCharFormat()
            style.setForeground(QColor(COLORS[level]))
            cursor.insertText(line, style)
        cursor.endEditBlock()
        if self.auto.isChecked():
            scroll = self.view.verticalScrollBar()
            scroll.setValue(scroll.maximum())

    def rebuild(self):
        self.pending.clear()
        self.view.clear()
        self.render(self.records)

    def clear(self):
        self.records.clear()
        self.rebuild()

    def save(self):
        path, _ = QFileDialog.getSaveFileName(self, 'ログを保存', 'control-center.log')
        if path:
            try:
                Path(path).write_text(''.join(line + '\n' for _, _, line in self.selected(self.records)))
            except OSError as error:
                QMessageBox.warning(self, '保存エラー', str(error))
