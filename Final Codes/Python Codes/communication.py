import time
import serial
from xbee import XBee

from config import USB_PORT

class XBeeComm:
    """A class to handle communication between laptop and the robots

    Attributes:
        xbee (XBee class): an object that initializes xbee communication

    Methods:
        tx(): a method to send data to reciever xbee
    """

    def __init__(self, com=USB_PORT, baud=9600):
        """Creates an instance of XBeeComm class
        Arguments:
            com (str): directory of the connected port
            baud (int): baud rate
        """
        self.xbee = XBee(serial.Serial(com,baud))
        
    def tx(self, data, dest):
        """Sends data to the reciever given its address
        Arguments:
            data (str): data to be sent to the reciever xbee
            dest (str): address of the reciever xbee
        """
        self.xbee.tx(dest_addr=dest,data=data)


class Packets:
    """A class to create data packets to be sent

    Attributes:
        data (list): a list containing small packets of data

    Methods:
        push(): to push small packets of data to data list
        reset(): empty the data list
        createPackets(): combine the small data packets in the data list to create a single large data packet
    """

    def __init__(self):
        """Creates an instance of Packets class
        Arguments:
            com (str): directory of the connected port
            baud (int): baud rate
        """
        self.data = []

    def push(self, data):
        """Creates small packets of data and pushes it into data list
        Arguments:
            data (int/str/tuple): data to sent 
        """

        #data is tuple recursively call for each element in tuple
        if type(data) is tuple:
            for i in data:
                self.push(i)
            return

        data = str(data)

        #end each number with a pipe (|)
        if data.isdigit():
            data = data + '|'

        self.data.append(data)

    def resetData(self):
        """Empties the data list"""
        self.data = []

    def createPacket(self):
        """Combines all the elements in data to create a single large packet of data
        Returns:
            packet (str): small data packets combined between angle brackets
        """
        packet = '<' + ''.join(self.data) + '>'
        return packet
