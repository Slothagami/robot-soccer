# LEGO type:standard slot:1 autostart
from mindstorms import MSHub
import hub as hubdata

def acceleration():
    x, z, y = hubdata.motion.accelerometer()
    # return (-x, -y, z)
    return complex(-x, z)

class Port:
    def __init__(self):
        self.com = hubdata.BT_VCP()
        print(hubdata.bluetooth.info().get("mac_addr"))

    def send(self, data):
        data = str(data)
        if "~" in data: raise Exception('Sending data cannot include "~"')
        self.com.write("~" + data + "~")

port = Port()
while True:
    port.send(acceleration())
    # print(acceleration())
