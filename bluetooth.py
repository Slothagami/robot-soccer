# LEGO type:standard slot:0 autostart
# COM11
from mindstorms import MSHub
import ustruct as struct
import hub     as hubdata

#region Screen
def screen(data):
    hub.light_matrix.show(data)

class Screen:
    alert   = "40904:40904:40904:00000:40904"
    amazed  = "09090:00000:09990:09990:09990"
    dead    = "09990:99999:90909:99999:09090"
    happy   = "09090:00000:97779:99999:09990"
    neutral = "00000:09090:00000:09990:00000"
    sqirm   = "00000:92029:29092:00000:09990"
    serious = "00000:90009:99099:00000:09990"
    off     = "00000:00000:00000:00000:00000"
    square  = "00000:09990:09990:09990:00000"

#endregion

## Functional Classes ##
class Bluetooth:
    def __init__(self, connect_address):
        self.bt = hubdata.BT_VCP()
        self.connect_adress = connect_address

        if not self.bt.isconnected():
            hubdata.bluetooth.rfcomm_connect(connect_address)

    def disconnect(self):
        hubdata.bluetooth.rfcomm_disconnect()

    @staticmethod
    def device_adress():
        return hubdata.bluetooth.info().get("mac_addr")

    def read(self, type, nbytes=4):
        if self.bt.any():
            if type is float: nbytes = 4
            if type is int:   nbytes = 2

            data = self.bt.read(nbytes)

            if type is int:
                return struct.unpack("<H", data)[0]

            if type is float:
                return struct.unpack("f", data)[0]

    def write(self, data):
        buffer = None
        if type(data) is int:
            buffer = struct.pack("<H", data)

        if type(data) is float:
            buffer = struct.pack("f", data)
            
        if buffer is not None:
            self.bt.write(buffer)
            self.bt.send(buffer)

hub = MSHub()
screen(Screen.square)

bt = Bluetooth(Bluetooth.device_adress())
while True:
    if hub.left_button.was_pressed():
        bt.write(27)

    read = bt.read(int)
    if read is not None:
        print(read)
