import time
from sensor import Sensor

class L3G4200D(Sensor):
    def __init__(self):
        super().__init__(0x69)
        self.setup()
        self.bias = self.calibrate()
        self.x = 0
        self.y = 0
        self.z = 0


    def setup(self):
        self.write_byte_data(0x20, 0x0F)
        self.write_byte_data(0x23, 0x30)

    def read(self):
        data = self.bus.read_i2c_block_data(self.address, 0x28 | 0x80, 6)
        x = self.to_signed(data[1] << 8 | data[0])
        y = self.to_signed(data[3] << 8 | data[2])
        z = self.to_signed(data[5] << 8 | data[4])
        return x - self.bias[0], y - self.bias[1], z - self.bias[2]

    def calibrate(self, samples=500):
        print("L3G4200D inicializando a calibração")
        sum_x = sum_y = sum_z = 0
        for _ in range(samples):
            x, y, z = self.read()
            sum_x += x
            sum_y += y
            sum_z += z
            time.sleep(0.01)
        self.bias = (sum_x / samples, sum_y / samples, sum_z / samples)
        print("L3G4200D calibração concluída")
