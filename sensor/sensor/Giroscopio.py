import time
from math import pi
from .SensorI2C import SensorI2C

class Giroscopio(SensorI2C):
    def __init__(self):
        super().__init__(0x69)
        self.write_byte_data(0x20, 0x0F)
        self.write_byte_data(0x23, 0x30)
        self.bias = (0.0, 0.0, 0.0)
        self.bias = self.calibrate()

    def read_raw(self):
        try:
            data = self.bus.read_i2c_block_data(self.address, 0x28 | 0x80, 6)
            p = self.to_signed(data[1] << 8 | data[0])
            q = self.to_signed(data[3] << 8 | data[2])
            r = self.to_signed(data[5] << 8 | data[4])
            return p, q, r
        except Exception as e:
            print(f"[ERRO] Leitura do giroscópio falhou: {e}")
            return 0, 0, 0
    
    def read(self):
        p, q, r = self.read_raw()
        p = (p - self.bias[0]) * 0.07 * pi / 180
        q = (q - self.bias[1]) * 0.07 * pi / 180
        r = (r - self.bias[2]) * 0.07 * pi / 180
        return p, q, r

    def calibrate(self, samples=500):
        print("L3G4200D inicializando a calibração...")
        sum_p = sum_q = sum_r = 0
        for _ in range(samples):
            p, q, r = self.read_raw()
            sum_p += p
            sum_q += q
            sum_r += r
            time.sleep(0.05)
        print("L3G4200D calibração concluída!")
        return (sum_p / samples, sum_q / samples, sum_r / samples)
        
