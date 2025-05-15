import math
from sensor import Sensor

class HMC5883L(Sensor):
    def __init__(self):
        super().__init__(0x1E)
        self.setup()
        self.x = 0
        self.y = 0
        self.z = 0
        self.heading = 0

    def setup(self):
        self.write_byte_data(0x00, 0x70)
        self.write_byte_data(0x01, 0xA0)
        self.write_byte_data(0x02, 0x00)

    def read(self):
        data = self.bus.read_i2c_block_data(self.address, 0x03, 6)
        self.x = self.to_signed(data[0] << 8 | data[1])
        self.z = self.to_signed(data[2] << 8 | data[3])
        self.y = self.to_signed(data[4] << 8 | data[5])
        self.heading = math.degrees(math.atan2(y, x))
        if self.heading < 0:
            self.heading += 360
        return self.x, self.y, self.z, self.heading
