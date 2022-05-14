from PyQt5.QtWidgets import (QApplication, QWidget,
                             QVBoxLayout, QLineEdit,
                             QPushButton, QLabel,
                             QGroupBox)

class Main_window:
    port = "/dev/ttyACM1"
    listening = False

    def __init__(self):
        self.app = QApplication([])
        self.window = QWidget()
        self.layout = QVBoxLayout()

        self.line_vbox = QVBoxLayout()
        self.label_line_port = QLabel('Port:')
        self.line_port = QLineEdit(self.port)
        self.line_port.editingFinished.connect(self.update_port)
        self.line_vbox.addWidget(self.label_line_port)
        self.line_vbox.addWidget(self.line_port)
        
        self.line_groupBox = QGroupBox()
        self.line_groupBox.setLayout(self.line_vbox)

        self.button_listen = QPushButton('Listen')
        self.button_listen.setCheckable(True)
        self.button_listen.clicked.connect(self.toggle_listen)

        self.layout.addWidget(self.line_groupBox)
        self.layout.addWidget(self.button_listen)

        self.window.setLayout(self.layout)
        self.window.show()

        self.app.exec()

    def update_port(self):
        self.port = self.line_port.text()

    def toggle_listen(self):
        self.listening = self.button_listen.isChecked()
        print(self.listening)

Main_window()

