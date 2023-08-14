# LEGO type:standard slot:1 autostart
from mindstorms import MSHub
import hub     as hubdata

hub = MSHub()

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
    if hub.left_button.was_pressed(): 
        port.send(complex(42,513))
