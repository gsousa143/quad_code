
import time
import serial
from sensor import Sensor
from control import pid_quad
import numpy as np

class Quad:
    def __init__(self, dt=0.5, initial_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        self.dt = dt # Intervalo de tempo para atualização
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
        # Inicializa o controlador PID para quadricóptero
        self.pid = pid_quad(T=self.dt)

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

    def att_state(self):
        sensores = self.sensor.read()
        ax, ay, az = sensores['acc']
        p_meas, q_meas, r_meas = sensores['gyro']
        heading = sensores['mag'][3]
        altitude = sensores['bar']

        # Atualiza taxas angulares e integra orientação
        self.p, self.q, self.r = p_meas, q_meas, r_meas
        self.phi += self.p*self.dt
        self.theta += self.q*self.dt
        self.psi += self.r*self.dt

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


        self.u += (ax - gravidade_corpo[0])*self.dt
        self.v += (ay - gravidade_corpo[1])*self.dt
        self.w += (az - gravidade_corpo[2])*self.dt

        vel_mundo = [
            R[0][0]*self.u + R[0][1]*self.v + R[0][2]*self.w,
            R[1][0]*self.u + R[1][1]*self.v + R[1][2]*self.w,
            R[2][0]*self.u + R[2][1]*self.v + R[2][2]*self.w
        ]

        self.x += vel_mundo[0]*self.dt
        self.y += vel_mundo[1]*self.dt
        self.z += vel_mundo[1]*self.dt  # usa altitude do barômetro
        self.z =  0.7*self.z + 0.3*altitude  # filtro complementar para altitude



def log_estado(self):
    """
    Imprime o log de estado do sistema, com os ângulos de atitude em graus.
    """
    # Converte os ângulos de radianos para graus para melhor legibilidade
    phi_deg = np.degrees(self.phi)
    theta_deg = np.degrees(self.theta)
    psi_deg = np.degrees(self.psi)

    log_formatado = f"""
    Posição (m):        x={self.x: 8.2f}, y={self.y: 8.2f}, z={self.z: 8.2f}
    Atitude (°):        φ={phi_deg: 8.2f}, θ={theta_deg: 8.2f}, ψ={psi_deg: 8.2f}
    Vel. Linear (m/s):  u={self.u: 8.2f}, v={self.v: 8.2f}, w={self.w: 8.2f}
    Vel. Angular (rad/s): p={self.p: 8.2f}, q={self.q: 8.2f}, r={self.r: 8.2f}
    """
    print(log_formatado)
        


if __name__ == "__main__":
    quad = Quad()
    

    try:
        while True:
            tic = time.time()
            quad.att_state()
            quad.sensor.log()
            w1, w2, w3, w4 = quad.pid.control(quad.x, quad.y, quad.z, quad.phi, quad.theta, quad.psi,
                                            x_sp=0.0, y_sp=0.0, z_sp=1.0, yaw_sp=0.0)
            command = f"{w1:.4f} {w2:.4f} {w3:.4f} {w4:.4f}\n"
            quad.serial.write(command.encode())
            print(f"Comando enviado: {command.strip()}")
            quad.log_estado()
            toc = time.time()
            elapsed = toc - tic
            print(f"Tempo de execução: {elapsed:.4f} segundos")
    except KeyboardInterrupt:
        print("Simulação interrompida.")
    finally:
        quad.serial.close()
        




