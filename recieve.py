from serial import Serial
import re

class Port:
    def __init__(self, port):
        self.port = Serial(port=port, bytesize=8)

    def recieve(self):
        data = self.port.read_all().decode("UTF-8")
        data = re.findall("~(.*?)~", data)

        return [self.decode_str(msg) for msg in data]

    def decode_str(self, data):
        if self.match("\(?.*[\+\-]?.*j\)?", data): return complex(data)
        if self.match("\d+", data):     return int(data)
        if self.match("[\d\.]+", data): return float(data)
        return data

    def match(self, pattern, data):
        return len(re.findall(f"^{pattern}$", data)) > 0

if __name__ == "__main__":
    from time import sleep

    port = Port("COM15")
    while True:
        print(port.recieve())
        sleep(.5)
