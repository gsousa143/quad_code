import math
import time
import serial
from gy80 import Acelerometro, Giroscopio, Magnetometro, Barometro
from random import random as rand

class Quad:
    def __init__(self):
        # Estado do drone
        self.x = self.y = self.z = 0.0
        self.phi = self.theta = self.psi = 0.0
        self.u = self.v = self.w = 0.0
        self.p = self.q = self.r = 0.0

        # Sensores
        self.acc = Acelerometro()
        self.gyro = Giroscopio()
        self.mag = Magnetometro()
        self.bar = Barometro()

        # Comunicação serial com Arduino
        self.serial = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Ajuste a porta conforme necessário
        time.sleep(2)  # Aguarda inicialização do Arduino

    def ler_sensores(self):
        ax, ay, az = self.acc.read()
        p, q, r = self.gyro.read()
        heading = self.mag.read()
        altitude = self.bar.read()

        return {
            'acc': (ax, ay, az),
            'gyro': (p, q, r),
            'mag': heading,
            'bar': altitude
        }

    def matriz_rotacao(self):
        c_phi = math.cos(self.phi)
        s_phi = math.sin(self.phi)
        c_theta = math.cos(self.theta)
        s_theta = math.sin(self.theta)
        c_psi = math.cos(self.psi)
        s_psi = math.sin(self.psi)

        return [
            [c_psi * c_theta, c_psi * s_theta * s_phi - s_psi * c_phi, c_psi * s_theta * c_phi + s_psi * s_phi],
            [s_psi * c_theta, s_psi * s_theta * s_phi + c_psi * c_phi, s_psi * s_theta * c_phi - c_psi * s_phi],
            [-s_theta,         c_theta * s_phi,                        c_theta * c_phi]
        ]

    def atualizar_estado(self, dt):
        sensores = self.ler_sensores()
        ax, ay, az = sensores['acc']
        p_meas, q_meas, r_meas = sensores['gyro']
        heading = sensores['mag']
        altitude = sensores['bar']

        # Atualiza taxas angulares e integra orientação
        self.p, self.q, self.r = p_meas, q_meas, r_meas
        self.phi += self.p * dt
        self.theta += self.q * dt
        self.psi += self.r * dt

        # Correção do yaw com o magnetômetro
        self.psi = 0.98 * self.psi + 0.02 * heading

        # Subtrai gravidade da aceleração medida
        R = self.matriz_rotacao()
        g = 9.81
        gravidade_corpo = [-R[0][2] * g, -R[1][2] * g, -R[2][2] * g]

        ax_lin = ax - gravidade_corpo[0]
        ay_lin = ay - gravidade_corpo[1]
        az_lin = az - gravidade_corpo[2]

        self.u += ax_lin * dt
        self.v += ay_lin * dt
        self.w += az_lin * dt

        vel_mundo = [
            R[0][0] * self.u + R[0][1] * self.v + R[0][2] * self.w,
            R[1][0] * self.u + R[1][1] * self.v + R[1][2] * self.w,
            R[2][0] * self.u + R[2][1] * self.v + R[2][2] * self.w
        ]

        self.x += vel_mundo[0] * dt
        self.y += vel_mundo[1] * dt
        self.z = altitude  # usa altitude do barômetro

        return sensores

    def estado_atual(self):
        return {
            'posicao': (self.x, self.y, self.z),
            'orientacao': (
                math.degrees(self.phi),
                math.degrees(self.theta),
                (math.degrees(self.psi) + 360) % 360
            ),
            'vel_linear': (self.u, self.v, self.w),
            'vel_angular': (
                math.degrees(self.p),
                math.degrees(self.q),
                math.degrees(self.r)
            )
        }

    def imprimir_estado(self, sensores):
        estado = self.estado_atual()

        print(f"\n--- ESTADO ESTIMADO DO DRONE ---")
        print(f"Posição (m): x={estado['posicao'][0]:.2f}, y={estado['posicao'][1]:.2f}, z={estado['posicao'][2]:.2f}")
        print(f"Orientação (graus): roll={estado['orientacao'][0]:.2f}, "
            f"pitch={estado['orientacao'][1]:.2f}, yaw={estado['orientacao'][2]:.2f}")
        print(f"Vel linear (m/s): u={estado['vel_linear'][0]:.2f}, v={estado['vel_linear'][1]:.2f}, w={estado['vel_linear'][2]:.2f}")
        print(f"Vel angular (deg/s): p={estado['vel_angular'][0]:.2f}, "
            f"q={estado['vel_angular'][1]:.2f}, r={estado['vel_angular'][2]:.2f}")

        print(f"\n--- LEITURAS DOS SENSORES ---")
        print(f"Acelerômetro (m/s²): ax={sensores['acc'][0]:.2f}, ay={sensores['acc'][1]:.2f}, az={sensores['acc'][2]:.2f}")
        print(f"Giroscópio (rad/s): p={sensores['gyro'][0]:.2f}, q={sensores['gyro'][1]:.2f}, r={sensores['gyro'][2]:.2f}")
        print(f"Magnetômetro (heading): {math.degrees(sensores['mag']):.2f}°")
        print(f"Barômetro (altitude): {sensores['bar']:.2f} m")


    def controle(self, setpoint=[]):
        # Comando de controle aleatório (substitua por seu controlador real)
        u = [0.1*rand() for _ in range(4)]

        # Garante que valores estejam entre 0.0 e 1.0
        u = [max(0.0, min(0.1, val)) for val in u]

        # Constrói string para o Arduino
        comando = ','.join(f"{val:.3f}" for val in u) + '\n'
        # Envia comando pela serial
        if self.serial.is_open:
            self.serial.write(comando.encode('utf-8'))
            print(f"[Serial] Comando enviado: {comando.strip()}")

# Loop principal
if __name__ == "__main__":
    quad = Quad()
    intervalo = 0.1  # 10 Hz
    duracao = float(input("Digite o tempo de coleta em segundos: "))
    fim = time.time() + duracao
    t_ultimo = time.time()

    while time.time() < fim:
        t_atual = time.time()
        dt = t_atual - t_ultimo
        t_ultimo = t_atual

        sensores = quad.atualizar_estado(dt)
        quad.imprimir_estado(sensores)
        quad.controle()

        time.sleep(intervalo)
