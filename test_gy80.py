import time
import math
from gy80 import Acelerometro, Giroscopio, Magnetometro, Barometro


def coletar_dados(tempo_leitura):
    acc = Acelerometro()
    gyro = Giroscopio()
    mag = Magnetometro()
    bar = Barometro()

    intervalo = 1  # segundos entre leituras
    fim = time.time() + tempo_leitura

    print(f"Iniciando coleta por {tempo_leitura} segundos...")

    while time.time() < fim:
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())

        ax, ay, az = acc.read()
        p, q, r = gyro.read()
        heading = mag.read()
        altitude = bar.read()

        print(f"[{timestamp}] Acc: ({ax:.2f}, {ay:.2f}, {az:.2f}) m/s² | "
              f"Gyro: ({math.degrees(p):.2f}, {math.degrees(q):.2f}, {math.degrees(r):.2f}) º/s | "
              f"Mag: {(math.degrees(heading) + 360) % 360:.2f} º | "
              f"Alt: {altitude:.2f} m")

        time.sleep(intervalo)


if __name__ == "__main__":
    duracao = float(input("Digite o tempo de coleta em segundos: "))
    coletar_dados(duracao)
