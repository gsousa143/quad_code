import math
import time
from gy80 import Acelerometro, Giroscopio, Magnetometro, Barometro


class Quad:
    def __init__(self):
        # Estado do drone
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.phi = 0.0     # Roll (rad)
        self.theta = 0.0   # Pitch (rad)
        self.psi = 0.0     # Yaw (rad)

        self.u = 0.0       # Velocidade no corpo X (m/s)
        self.v = 0.0       # Velocidade no corpo Y (m/s)
        self.w = 0.0       # Velocidade no corpo Z (m/s)

        self.p = 0.0       # Roll rate (rad/s)
        self.q = 0.0       # Pitch rate (rad/s)
        self.r = 0.0       # Yaw rate (rad/s)

        # Sensores
        self.acc = Acelerometro()
        self.gyro = Giroscopio()
        self.mag = Magnetometro()
        self.bar = Barometro()

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
        # Matriz de rotação corpo -> mundo
        phi = self.phi
        theta = self.theta
        psi = self.psi

        c_phi = math.cos(phi)
        s_phi = math.sin(phi)
        c_theta = math.cos(theta)
        s_theta = math.sin(theta)
        c_psi = math.cos(psi)
        s_psi = math.sin(psi)

        R = [
            [c_psi * c_theta, c_psi * s_theta * s_phi - s_psi * c_phi, c_psi * s_theta * c_phi + s_psi * s_phi],
            [s_psi * c_theta, s_psi * s_theta * s_phi + c_psi * c_phi, s_psi * s_theta * c_phi - c_psi * s_phi],
            [-s_theta,         c_theta * s_phi,                        c_theta * c_phi]
        ]
        return R

    def atualizar_estado(self, dt):
        # Ler sensores
        sensores = self.ler_sensores()
        ax, ay, az = sensores['acc']
        p_meas, q_meas, r_meas = sensores['gyro']
        heading = sensores['mag']
        altitude = sensores['bar']

        # Atualizar velocidades angulares
        self.p = p_meas
        self.q = q_meas
        self.r = r_meas

        # Atualizar orientação (integrando giroscópio)
        self.phi += self.p * dt
        self.theta += self.q * dt
        self.psi += self.r * dt

        # Correção simples do yaw com magnetômetro
        self.psi = 0.98 * self.psi + 0.02 * heading

        # Matriz de rotação
        R = self.matriz_rotacao()

        # Aceleração medida inclui gravidade no corpo, então removemos
        # Gravidade no mundo [0, 0, g]
        g = 9.81
        gravidade_corpo = [
            -R[0][2] * g,  # eixo x do corpo
            -R[1][2] * g,  # eixo y do corpo
            -R[2][2] * g   # eixo z do corpo
        ]

        ax_lin = ax - gravidade_corpo[0]
        ay_lin = ay - gravidade_corpo[1]
        az_lin = az - gravidade_corpo[2]

        # Atualizar velocidades lineares no corpo
        self.u += ax_lin * dt
        self.v += ay_lin * dt
        self.w += az_lin * dt

        # Converter velocidades do corpo para mundo usando R
        vel_mundo = [
            R[0][0] * self.u + R[0][1] * self.v + R[0][2] * self.w,
            R[1][0] * self.u + R[1][1] * self.v + R[1][2] * self.w,
            R[2][0] * self.u + R[2][1] * self.v + R[2][2] * self.w,
        ]

        # Atualizar posição no mundo
        self.x += vel_mundo[0] * dt
        self.y += vel_mundo[1] * dt
        self.z += vel_mundo[2] * dt

        # Atualizar z pelo barômetro também (opcional, para estabilidade)
        self.z = altitude

    def estado_atual(self):
        return {
            'posicao': (self.x, self.y, self.z),
            'orientacao': (math.degrees(self.phi),
                           math.degrees(self.theta),
                           (math.degrees(self.psi) + 360) % 360),
            'vel_linear': (self.u, self.v, self.w),
            'vel_angular': (math.degrees(self.p),
                            math.degrees(self.q),
                            math.degrees(self.r))
        }

    def imprimir_estado(self):
        estado = self.estado_atual()
        print(f"Posição (m): x={estado['posicao'][0]:.2f}, "
              f"y={estado['posicao'][1]:.2f}, z={estado['posicao'][2]:.2f} | "
              f"Orientação (deg): roll={estado['orientacao'][0]:.2f}, "
              f"pitch={estado['orientacao'][1]:.2f}, yaw={estado['orientacao'][2]:.2f} | "
              f"Vel linear (m/s): u={estado['vel_linear'][0]:.2f}, "
              f"v={estado['vel_linear'][1]:.2f}, w={estado['vel_linear'][2]:.2f} | "
              f"Vel ang (deg/s): p={estado['vel_angular'][0]:.2f}, "
              f"q={estado['vel_angular'][1]:.2f}, r={estado['vel_angular'][2]:.2f}")


# Loop de execução
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

        quad.atualizar_estado(dt)
        quad.imprimir_estado()

        time.sleep(intervalo)
