import serial
from threading import Thread
import time

last_received = ''

COMport = 'com9'

def receiving(serial_port):
    pass

class SerialData(object):
    def __init__(self):
        try:
            self.serial_port = serial.Serial(COMport,115200)
        except serial.serialutil.SerialException:
            self.serial_port = None
        else:
            Thread(target=receiving,args=(self.serial_port)).start()

    def send(self,data):
        self.serial_port.write(data + ",")

    def __del__(self):
        if self.serial_port is not None:
            self.serial_port.close()

if __name__ == '__main__':
    s = SerialData()
    while True:
        a = 0
        while a < 180:
            a += 1
            s.send(str(a))
            time.sleep(0.01)
        while a > 0:
            a -= 1
            s.send(str(a))
            time.sleep(0.01)