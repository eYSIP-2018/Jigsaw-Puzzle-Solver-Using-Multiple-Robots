from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress
import time
def communication(input_data):
    # Instantiate an XBee device object.
    device = XBeeDevice("COM3", 9600)
    
    inp = str(input_data)
    device.open()

    # Instantiate a remote XBee device object.
    remote_device = RemoteXBeeDevice(device, XBee64BitAddress.from_hex_string("0013A20040F655F0"))

    # Send data using the remote object.
    device.send_data_async(remote_device, inp)

    device.close()
    time.sleep(0)
#communication(2)    
