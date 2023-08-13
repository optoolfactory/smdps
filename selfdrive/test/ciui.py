#!/usr/bin/env python3
import subprocess

from PyQt5.QtCore import QTimer  # pylint: disable=no-name-in-module, import-error
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel # pylint: disable=no-name-in-module, import-error
from selfdrive.ui.qt.python_helpers import set_main_window

class Window(QWidget):
  def __init__(self, parent=None):
    super(Window, self).__init__(parent)

    layout = QVBoxLayout()
    layout.addStretch(1)
    self.setLayout(layout)

    cmds = [
      "cat /etc/hostname",
      "uptime",
      "cat /VERSION",
      "cat /BUILD",
    ]
    self.labels = {}
    for c in cmds:
      self.labels[c] = QLabel(c)
      layout.addWidget(self.labels[c])

    self.setStyleSheet("""
      * {
        color: white;
        font-size: 55px;
        background-color: black;
      }
    """)

    self.timer = QTimer()
    self.timer.timeout.connect(self.update)
    self.timer.start(10 * 1000)
    #self.update()

  def update(self):
    for cmd, label in self.labels.items():
      out = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           shell=True, check=False, encoding='utf8').stdout
      label.setText(out.strip())

if __name__ == "__main__":
  app = QApplication([])
  w = Window()
  set_main_window(w)
  app.exec_()
