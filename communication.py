import time
import serial
from xbee import XBee


class XBeeComm:

    def __init__(self, com='/dev/ttyUSB0', baud=9600):
        self.xbee = XBee(serial.Serial(com,baud))
        
    def tx(self, data, dest):
        #print(data)
        #return
        self.xbee.tx(dest_addr=dest,data=data)


class Packets:

    def __init__(self):
        self.data = []

    def push(self, data):

        if type(data) is tuple:
            for i in data:
                self.push(i)
            return

        data = str(data)
        if data.isdigit():
            data = data + '|'

        self.data.append(data)

    def resetData(self):
        self.data = []

    def createPacket(self):
        packet = '<' + ''.join(self.data) + '>'
        return packet
