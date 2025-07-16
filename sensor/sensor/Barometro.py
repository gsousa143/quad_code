import time
from collections import deque
from .SensorI2C import SensorI2C

class Barometro(SensorI2C):
    def __init__(self, oversampling=3, janela=5):
        super().__init__(0x77)
        self.oversampling = oversampling
        self.janela = janela
        self.pressure_at_sea_level = 101325.0
        self.c = {}
        self.buffer_altitude = deque(maxlen=janela)
        
        self.calibrate()

        # Preenche o buffer inicial com leituras reais
        for _ in range(janela):
            alt = self._read_raw()
            self.buffer_altitude.append(alt)
            time.sleep(0.01)  # pequeno delay para estabilidade

        # Define altitude inicial como a média inicial do buffer
        self.altitude_inicial = sum(self.buffer_altitude) / len(self.buffer_altitude)

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
        print("BMP180 calibração concluída!")

    def _read_raw(self):
        # Temperatura
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

        # Cálculo de temperatura compensada
        X1 = ((ut - self.c['AC6']) * self.c['AC5']) >> 15
        X2 = (self.c['MC'] << 11) // (X1 + self.c['MD'])
        B5 = X1 + X2

        # Cálculo de pressão compensada
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

        # Cálculo da altitude em metros
        altitude = 44330.0 * (1.0 - (pressure / self.pressure_at_sea_level) ** (1 / 5.255))
        return altitude

    def read(self):
        nova_altitude = self._read_raw() - self.altitude_inicial
        self.buffer_altitude.append(nova_altitude)
        media = sum(self.buffer_altitude) / len(self.buffer_altitude)
        return media
