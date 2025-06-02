import time
from math import pi
from sensor import Sensor

class Giroscopio(Sensor):
    def __init__(self):
        super().__init__(0x69)
        self.setup()
        self.bias = self.calibrate()



    def setup(self):
        self.write_byte_data(0x20, 0x0F)
        self.write_byte_data(0x23, 0x30)

    def read(self):
        data = self.bus.read_i2c_block_data(self.address, 0x28 | 0x80, 6)
        p = (self.to_signed(data[1] << 8 | data[0]) - self.bias[0]) * 0.07 * pi / 180
        q = (self.to_signed(data[3] << 8 | data[2]) - self.bias[1]) * 0.07 * pi / 180
        r = (self.to_signed(data[5] << 8 | data[4]) - self.bias[2]) * 0.07 * pi / 180
        return p, q, r

    def calibrate(self, samples=500):
        print("L3G4200D inicializando a calibração")
        sum_p = sum_q = sum_r = 0
        for _ in range(samples):
            p, q, r = self.read()
            sum_p += p
            sum_q += q
            sum_r += r
            time.sleep(0.01)
        self.bias = (sum_p / samples, sum_q / samples, sum_r / samples)
        print("L3G4200D calibração concluída")
