import smbus

class Sensor:
    def __init__(self, address, bus_num=1):
        self.bus = smbus.SMBus(bus_num)
        self.address = address

    def write_byte_data(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)

    def read_byte_data(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def read_word_be(self, reg):
        msb = self.read_byte_data(reg)
        lsb = self.read_byte_data(reg + 1)
        return (msb << 8) + lsb

    def to_signed(self, val):
        return val - 65536 if val > 32767 else val
    def close(self):
        self.bus.close()