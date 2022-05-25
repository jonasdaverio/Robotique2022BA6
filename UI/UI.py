from math import log10, floor
from PyQt5.QtGui import QPainter, QBrush, QPen, QLinearGradient, QIcon, QFont
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QThread
from PyQt5.QtWidgets import (QApplication, QWidget,
                             QVBoxLayout, QHBoxLayout, QGridLayout,
                             QPushButton, QLabel, QLineEdit,
                             QGroupBox, QDoubleSpinBox,
                             QGraphicsView, QGraphicsScene,
                             QGraphicsLineItem,
                             QGraphicsRectItem, QGraphicsSimpleTextItem)

import Map
import Serial

startup_scale = 2000
suffixes = { -24:'y', -21:'z', -18:'a', -15:'f', -12:'p',
              -9:'n',  -6:'µ',  -3:'m',  -2:'c',   0: '', 3:'k'}

class Map_view(QGraphicsView):
    change_zoom = pyqtSignal(float)

    def __init__(self, parent=None):
        super(Map_view, self).__init__(parent)

        self.setTransformationAnchor(QGraphicsView.NoAnchor)
        self.setResizeAnchor(QGraphicsView.NoAnchor)

        self.zoom = startup_scale

    def wheelEvent(self, event):
        zoomFactor = 1.15
        if event.angleDelta().y() < 0:
            zoomFactor = 1/zoomFactor

        oldPos = self.mapToScene(event.pos())
        self.scale(zoomFactor, zoomFactor)
        newPos = self.mapToScene(event.pos())
        delta = newPos - oldPos
        self.translate(delta.x(), delta.y())

        self.zoom *= zoomFactor
        self.change_zoom.emit(self.zoom)

class Main_window:
    def __init__(self):
        self.port = "/dev/ttyACM1"
        self.listening = False

        self.app = QApplication([])
        self.window = QWidget()
        self.window.setWindowIcon(QIcon("map.png"))

        self.map = Map.Map()

        self.main_layout = QHBoxLayout()

        self.controls_layout = QVBoxLayout()
        self.map_layout = QGridLayout()

        self.map_view = Map_view(self.map.scene)
        self.height_view = QGraphicsView()
        self.min_height_text = QGraphicsSimpleTextItem()
        self.max_height_text = QGraphicsSimpleTextItem()
        self.scale_view = QGraphicsView()
        self.scale_line = QGraphicsLineItem()
        self.scale_text = QGraphicsSimpleTextItem()

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
        self.init_graphics(self.map_layout)

        self.serial_worker = Serial.Serial_worker()
        thread = QThread()
        self.serial_worker.moveToThread(thread)
        thread.started.connect(self.serial_worker.run)
        thread.start()

        self.map_view.show()
        self.window.show()
        self.app.exec()

    def init_graphics(self, parent_layout):
        self.main_layout.addLayout(parent_layout)
        parent_layout.setSpacing(0)
        parent_layout.addWidget(self.map_view, 0, 0)
        parent_layout.addWidget(self.height_view, 0, 1)
        parent_layout.addWidget(self.scale_view, 1, 0, 1, 2)

        self.init_map()
        self.init_height()
        self.init_scale()

    def init_map(self):
        self.map_view.setStyleSheet("border-style: none;")
        self.map_view.setMinimumSize(500,500)
        self.map_view.setRenderHint(QPainter.Antialiasing)
        self.map_view.scale(startup_scale, startup_scale)
        self.map_view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.map_view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.map_view.change_zoom.connect(self.update_scale)

        brush = QBrush(Map.background_color)
        self.map_view.setBackgroundBrush(brush)

    def init_scale(self):
        brush = QBrush(Map.background_color)
        self.scale_view.setBackgroundBrush(brush)
        self.scale_view.setStyleSheet("border-style: none;")
        self.scale_view.setMaximumHeight(120)

        scale_scene = QGraphicsScene()
        self.scale_view.setScene(scale_scene)

        scale_scene.addItem(self.scale_line)
        pen = QPen(Qt.black)
        pen.setWidthF(4)
        self.scale_line.setPen(pen)

        scale_scene.addItem(self.scale_text)
        font = QFont()
        font.setPixelSize(20)
        self.scale_text.setFont(font)
        self.scale_text.setPos(0, 0)
        self.update_scale(startup_scale)

    def update_scale(self, scale):
        (px, text) = self.compute_scale(scale)
        self.scale_line.setLine(-px/2, 0, px/2, 0)
        self.scale_text.setText(text)
        self.scale_text.setPos(-self.scale_text.boundingRect().center().x(), 0)

    def compute_scale(self, scale):
        target_size_px = 200
        target_size_m = target_size_px/scale
        
        target_size_exponent = floor(log10(target_size_m))
        order_of_mag = pow(10, target_size_exponent)
        target_size_mantissa = target_size_m/order_of_mag

        possible_values = [1, 2, 5, 10]
        nearest_mantissa = min(possible_values, key=lambda x:abs(x-target_size_mantissa))
        if nearest_mantissa == 10:
            nearest_mantissa = 1
            target_size_exponent += 1
            order_of_mag *= 10

        pixel = nearest_mantissa * order_of_mag * scale
        length = self.floatToSI(nearest_mantissa, target_size_exponent)
        return (pixel, length)

    def floatToSI(self, mantissa, exponent):
        if exponent in range(-2,0):
            new_exponent = -2
        else:
            new_exponent = exponent - exponent%3

        if new_exponent < -24:
            new_exponent = -24
        elif new_exponent > 3:
            new_exponent = 3

        new_mantissa = mantissa * pow(10, exponent - new_exponent)
        if new_exponent in suffixes:
            suffixe = suffixes[new_exponent]
        else:
            suffixe = '?'

        return str(new_mantissa) + " " + suffixe + "m"


    def update_height(self, min_height, max_height):
        if min_height != 0:
            min_exponent = floor(log10(abs(min_height)))
            min_mantissa = min_height / pow(10, min_exponent)
        else:
            min_exponent = 0
            min_mantissa = min_height
        if max_height != 0:
            max_exponent = floor(log10(abs(max_height)))
            max_mantissa = max_height / pow(10, max_exponent)
        else:
            max_exponent = 0
            max_mantissa = max_height

        self.min_height_text.setText(self.floatToSI(min_mantissa, min_exponent))
        self.max_height_text.setText(self.floatToSI(max_mantissa, max_exponent))
        min_pos = self.min_height_text.pos().y()
        max_pos = self.max_height_text.pos().y()
        self.min_height_text.setPos(-self.min_height_text.boundingRect().center().x(), min_pos)
        self.max_height_text.setPos(-self.max_height_text.boundingRect().center().x(), max_pos)

    def init_height(self):
        self.map.height_extrema_changed.connect(self.update_height)

        self.height_view.setStyleSheet("border-style: none;")
        self.height_view.setMinimumWidth(100)
        self.height_view.setMaximumWidth(120)
        brush = QBrush(Map.background_color)
        self.height_view.setBackgroundBrush(brush)

        height_scene = QGraphicsScene()
        self.height_view.setScene(height_scene)

        height = 250
        width = 80
        margin_x = 25
        margin_y = 30

        height_background = QGraphicsRectItem(-width/2, height/2, width, -height)
        height_rect = QGraphicsRectItem(-(width/2-margin_x), height/2-margin_y,
                                  width-2*margin_x, -height+2*margin_y)

        gradient = QLinearGradient(0, height/2-margin_y, 0, -(height/2-margin_y))
        gradient.setColorAt(0, Map.high_color) 
        gradient.setColorAt(1, Map.low_color)
        h_brush = QBrush(gradient)
        bg_brush = QBrush(Qt.white)
        height_rect.setBrush(h_brush)
        height_background.setBrush(bg_brush)

        font = QFont()
        font.setPixelSize(20)
        self.min_height_text.setFont(font)
        self.max_height_text.setFont(font)
        self.max_height_text.setText("0.0 m")
        self.min_height_text.setPos(0, height/2-margin_y)
        self.max_height_text.setPos(0, -(height/2-margin_y)
                                       -self.max_height_text.boundingRect().height())
        
        height_scene.addItem(height_background)
        height_scene.addItem(height_rect)
        height_scene.addItem(self.min_height_text)
        height_scene.addItem(self.max_height_text)

        self.update_height(0, 0)

    def init_controls(self, parent_layout):
        parent_layout.addLayout(self.controls_layout)

        self.init_port_box(self.controls_layout)
        self.init_resolution_box(self.controls_layout)

        self.reset_button.clicked.connect(self.reset_button_event)
        self.controls_layout.addWidget(self.reset_button)

        self.controls_layout.addStretch()

    def init_port_box(self, parent_layout):
        self.port_box.setMaximumWidth(200)
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

