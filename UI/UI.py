from PyQt5.QtGui import QPainter, QBrush, QColor
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (QApplication, QWidget,
                             QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QLineEdit,
                             QGroupBox, QDoubleSpinBox,
                             QGraphicsView)

import Map

#all length are in meters
startup_scale = 2000
startup_resolution = 0.1
background_color = QColor(216,222,233) #chose because I like it

class My_view(QGraphicsView):
    def __init__(self, parent=None):
        super(My_view, self).__init__(parent)

        self.setTransformationAnchor(QGraphicsView.NoAnchor)
        self.setResizeAnchor(QGraphicsView.NoAnchor)

    def wheelEvent(self, event):
        zoomFactor = 1.25

        oldPos = self.mapToScene(event.pos())

        if event.angleDelta().y() < 0:
            zoomFactor = 1/zoomFactor
        self.scale(zoomFactor, zoomFactor)

        newPos = self.mapToScene(event.pos())

        delta = newPos - oldPos
        self.translate(delta.x(), delta.y())

class Main_window:
    def __init__(self):
        self.port = "/dev/ttyACM1"
        self.listening = False

        self.app = QApplication([])
        self.window = QWidget()

        self.map = Map.Map()

        self.main_layout = QHBoxLayout()

        self.controls_layout = QVBoxLayout()
        self.view = My_view(self.map.scene)

        self.port_box = QGroupBox()
        self.port_box_layout = QVBoxLayout()

        self.port_line_layout = QHBoxLayout()
        self.port_line = QLineEdit(self.port)
        self.port_line_label = QLabel('Port:')

        self.button_listen = QPushButton('Listen')

        self.resolution_box = QGroupBox()
        self.resolution_box_layout = QHBoxLayout()
        self.resolution_label = QLabel("Resolution: ")
        self.resolution_spin_box = QDoubleSpinBox()
        
        self.reset_button = QPushButton("Reset")

        self.window.setLayout(self.main_layout)

        self.init_controls(self.main_layout)
        self.init_graphics(self.main_layout)

        self.view.show()
        self.window.show()
        self.app.exec()

    def init_graphics(self, parent_layout):
        parent_layout.addWidget(self.view)
        self.view.setMinimumSize(500,500)
        self.view.setRenderHint(QPainter.Antialiasing)
        self.view.scale(startup_scale, startup_scale)

        brush = QBrush(background_color)
        self.view.setBackgroundBrush(brush)

    def init_controls(self, parent_layout):
        parent_layout.addLayout(self.controls_layout)

        self.init_port_box(self.controls_layout)
        self.init_resolution_box(self.controls_layout)

        self.reset_button.clicked.connect(self.reset_button_event)
        self.controls_layout.addWidget(self.reset_button)

        self.controls_layout.addStretch()

    def init_port_box(self, parent_layout):
        self.port_box.setMaximumWidth(300)
        parent_layout.addWidget(self.port_box)
        self.port_box.setLayout(self.port_box_layout)

        self.init_port_line(self.port_box_layout)

        self.button_listen.setCheckable(True)
        self.button_listen.clicked.connect(self.toggle_listen)
        self.port_box_layout.addWidget(self.button_listen)

    def init_port_line(self, parent_layout):
        self.port_line.editingFinished.connect(self.update_port)
        self.port_line.setMinimumWidth(100)
        self.port_line_layout.addWidget(self.port_line_label)
        self.port_line_layout.addWidget(self.port_line)
        parent_layout.addLayout(self.port_line_layout)

    def init_resolution_box(self, parent_layout):
        parent_layout.addWidget(self.resolution_box)
        self.resolution_box.setLayout(self.resolution_box_layout)
        self.resolution_box_layout.addWidget(self.resolution_label)
        self.resolution_box_layout.addWidget(self.resolution_spin_box)

        self.resolution_spin_box.setSuffix(' cm')
        self.resolution_spin_box.setDecimals(1)
        self.resolution_spin_box.setMinimum(1)
        self.resolution_spin_box.setMaximum(20)
        self.resolution_spin_box.setStepType(QDoubleSpinBox.AdaptiveDecimalStepType)

        self.resolution_spin_box.setValue(100*self.map.resolution)
        self.resolution_spin_box.valueChanged.connect(self.update_resolution)

    def update_resolution(self, resolution):
        def delayed_update_resolution():
            self.map.update_resolution(0.01*resolution)
            self.resolution_timer.stop()
        self.resolution_timer = QTimer()
        self.resolution_timer.timeout.connect(delayed_update_resolution)
        self.resolution_timer.start(1000)

    def reset_button_event(self):
        self.map.reset_scene()

    def update_port(self):
        self.port = self.port_line.text()

    def toggle_listen(self):
        self.listening = self.button_listen.isChecked()
        if self.listening:
            self.map.update_scene()
        else:
            self.map.hide_robot()
        print(self.listening)

Main_window()

