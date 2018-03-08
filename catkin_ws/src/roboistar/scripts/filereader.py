from threading import Thread
import time
import os

filepath = 'lidarmsg.txt'

def str2bool(v):
    return v.lower() in ('true', 'True', '1')

class RosFileReader():
    def __init__(self, msgpath):
        try:
            self.fp = open(msgpath, 'r', os.O_NONBLOCK)
            self.msgpath = msgpath
            self.isObstacle = False
            self.motorLeft = 1500
            self.motorRight = 1500
        except:
            self.fp = None
            print("file name error")
        else:
            Thread(target = self.reading).start()

    def reading(self):
        self.fp = open(self.msgpath, 'r', os.O_NONBLOCK)
        line = self.fp.readline()
        self.isObstacle = str2bool(line.split(':')[1].strip())
        line = self.fp.readline()
        self.motorLeft = int(line.split(':')[1].strip())
        line = self.fp.readline()
        self.motorRight = int(line.split(':')[1].strip())
        self.fp.close()
        time.sleep(0.5)

if __name__ == "__main__":
    rosfilereader = RosFileReader(filepath)
    while True:
        print (rosfilereader.isObstacle)
        print (rosfilereader.motorLeft)
        print (rosfilereader.motorRight)
        time.sleep(1)
