import time
from .SensorI2C import SensorI2C

class Barometro(SensorI2C):
    def __init__(self, oversampling=3):
        super().__init__(0x77)
        self.altitude_inicial = 0.0
        self.oversampling = oversampling
        self.c = {}
        self.pressure_at_sea_level = 101325.0
        self.calibrate()

    def calibrate(self):
        print("BMP180 inicializando a calibração...")
        self.c['AC1'] = self.to_signed(self.read_word_be(0xAA))
        self.c['AC2'] = self.to_signed(self.read_word_be(0xAC))
        self.c['AC3'] = self.to_signed(self.read_word_be(0xAE))
        self.c['AC4'] = self.read_word_be(0xB0)
        self.c['AC5'] = self.read_word_be(0xB2)
        self.c['AC6'] = self.read_word_be(0xB4)
        self.c['B1'] = self.to_signed(self.read_word_be(0xB6))
        self.c['B2'] = self.to_signed(self.read_word_be(0xB8))
        self.c['MB'] = self.to_signed(self.read_word_be(0xBA))
        self.c['MC'] = self.to_signed(self.read_word_be(0xBC))
        self.c['MD'] = self.to_signed(self.read_word_be(0xBE))
        self.altitude_inicial = self.read()
        print("BMP180 calibração concluída!")
        
    def read(self):
        self.write_byte_data(0xF4, 0x2E)
        time.sleep(0.005)
        ut = (self.read_byte_data(0xF6) << 8) + self.read_byte_data(0xF7)

        # Pressão
        self.write_byte_data(0xF4, 0x34 + (self.oversampling << 6))
        time.sleep(0.005 + 0.003 * (2 ** self.oversampling))
        msb = self.read_byte_data(0xF6)
        lsb = self.read_byte_data(0xF7)
        xlsb = self.read_byte_data(0xF8)
        up = ((msb << 16) + (lsb << 8) + xlsb) >> (8 - self.oversampling)

        X1 = ((ut - self.c['AC6']) * self.c['AC5']) >> 15
        X2 = (self.c['MC'] << 11) // (X1 + self.c['MD'])
        B5 = X1 + X2

        B6 = B5 - 4000
        X1 = (self.c['B2'] * ((B6 * B6) >> 12)) >> 11
        X2 = (self.c['AC2'] * B6) >> 11
        X3 = X1 + X2
        B3 = (((self.c['AC1'] * 4 + X3) << self.oversampling) + 2) >> 2

        X1 = (self.c['AC3'] * B6) >> 13
        X2 = (self.c['B1'] * ((B6 * B6) >> 12)) >> 16
        X3 = ((X1 + X2) + 2) >> 2
        B4 = (self.c['AC4'] * (X3 + 32768)) >> 15
        B7 = (up - B3) * (50000 >> self.oversampling)

        if B7 < 0x80000000:
            p = (B7 * 2) // B4
        else:
            p = (B7 // B4) * 2

        X1 = (p >> 8) * (p >> 8)
        X1 = (X1 * 3038) >> 16
        X2 = (-7357 * p) >> 16
        pressure = p + ((X1 + X2 + 3791) >> 4)

        # Altitude
        altitude = 44330.0 * (1.0 - (pressure / self.pressure_at_sea_level) ** (1 / 5.255)) - self.altitude_inicial

        return altitude

