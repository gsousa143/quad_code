from .SensorI2C import SensorI2C
from collections import deque
import time  # necessário para inserir pequenos delays entre leituras

class Acelerometro(SensorI2C):
    def __init__(self, janela=20, delay_amostra=0.01):
        super().__init__(0x53)
        self.write_byte_data(0x2D, 0x08)  # POWER_CTL: ativa medição
        self.write_byte_data(0x31, 0x08)  # DATA_FORMAT: full resolution, ±2g

        self.janela = janela
        self.buffer_x = deque(maxlen=janela)
        self.buffer_y = deque(maxlen=janela)
        self.buffer_z = deque(maxlen=janela)

        # Pré-preenche os buffers com leituras reais
        for _ in range(janela):
            a_x, a_y, a_z = self._read_raw()
            self.buffer_x.append(a_x)
            self.buffer_y.append(a_y)
            self.buffer_z.append(a_z)
            time.sleep(delay_amostra)  # dá tempo ao sensor entre leituras

    def _read_raw(self):
        try:
            data = self.bus.read_i2c_block_data(self.address, 0x32, 6)

            a_x_raw = self.to_signed((data[1] << 8) | data[0])
            a_y_raw = self.to_signed((data[3] << 8) | data[2])
            a_z_raw = self.to_signed((data[5] << 8) | data[4])

            # Conversão para m/s² com float
            a_x = float(a_x_raw) * 0.004 * 9.81
            a_y = float(a_y_raw) * 0.004 * 9.81
            a_z = float(a_z_raw) * 0.004 * 9.81
            return a_x, a_y, a_z

        except Exception as e:
            print(f"[ERRO] Leitura do acelerômetro falhou: {e}")
            return 0.0, 0.0, 0.0

        

    def read(self):
        a_x, a_y, a_z = self._read_raw()

        self.buffer_x.append(a_x)
        self.buffer_y.append(a_y)
        self.buffer_z.append(a_z)

        # Calcula a média móvel
        avg_x = sum(self.buffer_x) / len(self.buffer_x)
        avg_y = sum(self.buffer_y) / len(self.buffer_y)
        avg_z = sum(self.buffer_z) / len(self.buffer_z)

        return avg_x, avg_y, avg_z
