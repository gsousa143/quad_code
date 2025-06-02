import math
from .Sensor import Sensor

class Magnetometro(Sensor):
    def __init__(self):
        super().__init__(0x1E)
        self.setup()

    def setup(self):
        self.write_byte_data(0x00, 0x70)
        self.write_byte_data(0x01, 0xA0)
        self.write_byte_data(0x02, 0x00)

    def read(self):
        data = self.bus.read_i2c_block_data(self.address, 0x03, 6)
        x = self.to_signed(data[0] << 8 | data[1])
        y = self.to_signed(data[6] << 8 | data[7])
        heading = math.atan2(y, x)
        return heading
