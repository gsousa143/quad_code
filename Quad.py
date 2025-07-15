import time
import serial
from control import PID_quad
from gy80 import Acelerometro, Giroscopio, Magnetometro, Barometro
import numpy as np

class Quad:
    def __init__(self, initial_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])):
        self.x = initial_state[0]
        self.y = initial_state[1]
        self.z = initial_state[2]
        self.phi = initial_state[3]
        self.theta = initial_state[4]
        self.psi = initial_state[5]
        self.u = initial_state[6]
        self.v = initial_state[7]
        self.w = initial_state[8]
        self.p = initial_state[9]
        self.q = initial_state[10]
        self.r = initial_state[11]

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
        c_phi = np.cos(self.phi)
        s_phi = np.sin(self.phi)
        c_theta = np.cos(self.theta)
        s_theta = np.sin(self.theta)
        c_psi = np.cos(self.psi)
        s_psi = np.sin(self.psi)

        return np.array([
            [c_psi*c_theta, c_psi*s_theta*s_phi - s_psi*c_phi, c_psi*s_theta*c_phi + s_psi*s_phi],
            [s_psi*c_theta, s_psi*s_theta*s_phi + c_psi*c_phi, s_psi*s_theta*c_phi - c_psi*s_phi],
            [-s_theta,      c_theta*s_phi,                     c_theta*c_phi]
        ])

    def att_state(self, dt):
        sensores = self.ler_sensores()
        ax, ay, az = sensores['acc']
        p_meas, q_meas, r_meas = sensores['gyro']
        heading = sensores['mag']
        altitude = sensores['bar']

        # Atualiza taxas angulares e integra orientação
        self.p, self.q, self.r = p_meas, q_meas, r_meas
        self.phi += self.p*dt
        self.theta += self.q*dt
        self.psi += self.r*dt

        # Correção do yaw com o magnetômetro
        self.psi = 0.9*self.psi + 0.1*heading

        # Normaliza para [-π, π]
        self.phi = (self.phi + 2 * np.pi) % (2 * np.pi) - np.pi
        self.theta = (self.theta + 2 * np.pi) % (2 * np.pi) - np.pi
        self.psi = (self.psi + 2 * np.pi) % (2 * np.pi) -np.pi

        # Subtrai gravidade da aceleração medida
        R = self.matriz_rotacao()
        g = 9.81
        gravidade_corpo = [-R[0][2]*g, -R[1][2]*g, -R[2][2]*g]


        self.u += (ax - gravidade_corpo[0])*dt
        self.v += (ay - gravidade_corpo[1])*dt
        self.w += (az - gravidade_corpo[2])*dt

        vel_mundo = [
            R[0][0]*self.u + R[0][1]*self.v + R[0][2]*self.w,
            R[1][0]*self.u + R[1][1]*self.v + R[1][2]*self.w,
            R[2][0]*self.u + R[2][1]*self.v + R[2][2]*self.w
        ]

        self.x += vel_mundo[0]*dt
        self.y += vel_mundo[1]*dt
        self.z += vel_mundo[1]*dt  # usa altitude do barômetro
        self.z =  0.9*self.z + 0.1*altitude  # filtro complementar para altitude


    def estado_atual(self):
        return {
            'posicao': (self.x, self.y, self.z),
            'orientacao': (
                np.degrees(self.phi),
                np.degrees(self.theta),
                (np.degrees(self.psi) + 360) % 360
            ),
            'vel_linear': (self.u, self.v, self.w),
            'vel_angular': (
                np.degrees(self.p),
                np.degrees(self.q),
                np.degrees(self.r)
            )
        }

    def imprimir_estado(self, estado, sensores):
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
        print(f"Magnetômetro (heading): {np.degrees(sensores['mag']):.2f}°")
        print(f"Barômetro (altitude): {sensores['bar']:.2f} m")


