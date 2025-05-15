import time
from sensor import Sensor

class ADXL345(Sensor):
    def __init__(self):
        super().__init__(0x53)
        self.setup()
        self.x = 0
        self.y = 0
        self.z = 0


    def setup(self):
        self.write_byte_data(0x2D, 0x08)
        self.write_byte_data(0x31, 0x08)

    def read_aceel(self):
        data = self.bus.read_i2c_block_data(self.address, 0x32, 6)
        x = self.to_signed(data[1] << 8 | data[0]) * 0.004
        y = self.to_signed(data[3] << 8 | data[2]) * 0.004
        z = self.to_signed(data[5] << 8 | data[4]) * 0.004
        self.x = x
        self.y = y
        self.z = z
