import time
import serial
from sensor import Sensor
import numpy as np

class Quad:
    def __init__(self, initial_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
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
        # Inicializa o sensor
        self.sensor = Sensor()

        # Comunicação serial com Arduino
        self.serial = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Ajuste a porta conforme necessário
        time.sleep(2)  # Aguarda inicialização do Arduino


    def matriz_rotacao(self):
        c_phi = np.cos(self.phi)
        s_phi = np.sin(self.phi)
        c_theta = np.cos(self.theta)
        s_theta = np.sin(self.theta)
        c_psi = np.cos(self.psi)
        s_psi = np.sin(self.psi)

        return [
            [c_psi*c_theta, c_psi*s_theta*s_phi - s_psi*c_phi, c_psi*s_theta*c_phi + s_psi*s_phi],
            [s_psi*c_theta, s_psi*s_theta*s_phi + c_psi*c_phi, s_psi*s_theta*c_phi - c_psi*s_phi],
            [-s_theta,      c_theta*s_phi,                     c_theta*c_phi]
        ]

    def att_state(self, dt):
        sensores = self.sensor.read()
        ax, ay, az = sensores['acc']
        p_meas, q_meas, r_meas = sensores['gyro']
        heading = sensores['mag'][3]
        altitude = sensores['bar']

        # Atualiza taxas angulares e integra orientação
        self.p, self.q, self.r = p_meas, q_meas, r_meas
        self.phi += self.p*dt
        self.theta += self.q*dt
        self.psi += self.r*dt

        # Correção do yaw com o magnetômetro
        self.psi = 0.7*self.psi + 0.3*heading

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
        self.z =  0.7*self.z + 0.3*altitude  # filtro complementar para altitude


    def log_estado(self):
        print(f"Estado atual: \nx={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}, "
              f"phi={self.phi:.2f}, theta={self.theta:.2f}, psi={self.psi:.2f}, "
              f"u={self.u:.2f}, v={self.v:.2f}, w={self.w:.2f}, "
              f"p={self.p:.2f}, q={self.q:.2f}, r={self.r:.2f}")
        print("-------------------------------")
        


if __name__ == "__main__":
    quad = Quad()
    dt = 0.5  # Intervalo de tempo para atualização

    try:
        while True:
            quad.att_state(dt)
            quad.sensor.log()
            quad.log_estado()
            time.sleep(dt)
    except KeyboardInterrupt:
        print("Simulação interrompida.")
    finally:
        quad.serial.close()
        




