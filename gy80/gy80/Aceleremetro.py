from sensor import Sensor

class Acelerometro(Sensor):
    def __init__(self):
        super().__init__(0x53)
        self.setup()

    def setup(self):
        self.write_byte_data(0x2D, 0x08)
        self.write_byte_data(0x31, 0x08)

    def read(self):
        data = self.bus.read_i2c_block_data(self.address, 0x32, 6)
        a_x = self.to_signed(data[1] << 8 | data[0]) * 0.004 * 9.81
        a_y = self.to_signed(data[3] << 8 | data[2]) * 0.004 * 9.81
        a_z = self.to_signed(data[5] << 8 | data[4]) * 0.004 * 9.81
        return a_x, a_y, a_z