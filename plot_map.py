from PyQt5.QtWidgets import (QApplication, QWidget,
                             QVBoxLayout, QHBoxLayout,
                             QLineEdit, QPushButton, QLabel,
                             QGroupBox, QDoubleSpinBox,
                             QGraphicsView, QGraphicsScene)
from PyQt5.QtCore import QTimer

class Map:
    scene = QGraphicsScene()
    
    #The grid format is as follow :
    #The key is tuple representing the coordinate of the grid element
    #The value is a list whose first element is the probability of
    #the grid element being an obstacle, and the second element
    #is a list of the height of each corner of the grid element
    grid = {(0,0) : [0.0, [0,0,0,0]]}

    def update_scene():
        for key in grid.keys():
            

class Main_window:
    port = "/dev/ttyACM1"
    listening = False
    resolution = 1.0

    app = QApplication([])
    window = QWidget()

    main_layout = QHBoxLayout()

    controls_layout = QVBoxLayout()
    view = QGraphicsView()

    port_box = QGroupBox()
    port_box_layout = QVBoxLayout()

    port_line_layout = QHBoxLayout()
    port_line = QLineEdit(port)
    port_line_label = QLabel('Port:')

    button_listen = QPushButton('Listen')

    resolution_box = QGroupBox()
    resolution_box_layout = QHBoxLayout()
    resolution_label = QLabel("Resolution: ")
    resolution_spin_box = QDoubleSpinBox()

    my_map = Map()

    def __init__(self):

        self.window.setLayout(self.main_layout)

        self.init_controls(self.main_layout)
        self.init_graphics(self.main_layout)

        self.window.show()
        self.app.exec()

    def init_graphics(self, parent_layout):
        parent_layout.addWidget(self.view)
        self.view.setMinimumSize(500,500)

    def init_controls(self, parent_layout):
        parent_layout.addLayout(self.controls_layout)

        self.init_port_box(self.controls_layout)
        self.init_resolution_box(self.controls_layout)

        self.controls_layout.addStretch()

    def init_resolution_box(self, parent_layout):
        None

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

        self.resolution_spin_box.setValue(self.resolution)
        self.resolution_spin_box.valueChanged.connect(self.wait_resolution)


    def update_port(self):
        self.port = self.line_port.text()

    def toggle_listen(self):
        self.listening = self.button_listen.isChecked()
        print(self.listening)

    def wait_resolution(self, resolution):
        def update_resolution_delayed():
            self.resolution = resolution
            print(resolution)
            self.resolution_timer.stop()

        self.resolution_timer = QTimer()
        self.resolution_timer.timeout.connect(update_resolution_delayed)
        self.resolution_timer.start(1000)

Main_window()

