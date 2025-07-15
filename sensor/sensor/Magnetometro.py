import math
from .SensorI2C import SensorI2C

class Magnetometro(SensorI2C):
    def __init__(self):
        super().__init__(0x1E)
        # Configuração do magnetômetro (HMC5883L)
        self.write_byte_data(0x00, 0x18)  # 1 sample, 75Hz, normal measurement
        self.write_byte_data(0x01, 0xA0)  # CRB: Gain = 5
        self.write_byte_data(0x02, 0x00)  # Mode: Continuous measurement

    def to_signed(self, val):
        """Converte valor de 16 bits para formato com sinal."""
        if val > 32767:
            val -= 65536
        return val

    def read(self):
        data = self.bus.read_i2c_block_data(self.address, 0x03, 6)
        
        x = self.to_signed(data[0] << 8 | data[1])
        z = self.to_signed(data[2] << 8 | data[3])
        y = self.to_signed(data[4] << 8 | data[5])

        heading = math.atan2(y, x)  # Radianos, no intervalo [-π, π]

        return x, y, z, heading
