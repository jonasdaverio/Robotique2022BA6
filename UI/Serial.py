from PyQt5.QtCore import QObject, QTimer
import serial
import struct
from dataclasses import dataclass
from queue import Queue

@dataclass
class Measurement:
    position: list[float]
    orientation: list[list[float]]
    tof: float
    ir: list[bool]

class Serial_worker(QObject):
    def __init__(self):
        super().__init__()

        self.port = None
        self.data: list[Measurement]
        self.listening = False

    # This function is almost copy paste from the script plotImage from the TPs    
    @staticmethod
    def readMeasurementSerial(port):
        state = 0
        while(state != 5):
            character = port.read(1)
            if(character == b''):
                return None

            #Wait until we see 'START'
            if(state == 0):
                if(character == b'S'):
                    state = 1
                else:
                    state = 0
            elif(state == 1):
                if(character == b'T'):
                    state = 2
                elif(character == b'S'):
                    state = 1
                else:
                    state = 0
            elif(state == 2):
                if(character == b'A'):
                    state = 3
                elif(character == b'S'):
                    state = 1
                else:
                    state = 0
            elif(state == 3):
                if(character == b'R'):
                    state = 4
                elif (character == b'S'):
                    state = 1
                else:
                    state = 0
            elif(state == 4):
                if(character == b'T'):
                    state = 5
                elif (character == b'S'):
                    state = 1
                else:
                    state = 0
        
        size = struct.unpack('<h', port.read(2))
        size = size[0] #The second element is unused (see doc)

        rcv_buffer = port.read(size)
        if(len(rcv_buffer) == size):
            position = []
            for i in range(0, 3):
                position.append(struct.unpack_from("<f", rcv_buffer, i*4)[0])

            orientation = []
            for i in range(0,3):
                row = []
                for j in range(0,3):
                    row.append(struct.unpack_from("<f", rcv_buffer, 4*(3*i+j+3))[0])
                orientation.append(row)

            tof = struct.unpack_from("<f", rcv_buffer, 4*(3*3 + 3))[0]

            ir = []
            for i in range(0,8):
                byte = int(struct.unpack_from("<B", rcv_buffer, 4*(3*3 + 3 + 1))[0])
                ir.append(bool(byte >> i & 0b1))
            measurement = Measurement(position, orientation, tof, ir)
            return measurement
        else:
            return None

    def open_port(self, port):
        try:
            self.port = serial.Serial(port, timeout=0.5)
            return True
        except:
            return False
    def close_port(self):
        self.port = None

    def listen(self):
        if self.port:
            self.listening = True
    def stop_listening(self):
        self.listening = False

    def run(self):
        if True: #self.port and self.listening:
            print("patate")
            QTimer.singleShot(500, self.run)
