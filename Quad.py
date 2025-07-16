import time
import serial
import csv
from datetime import datetime
from sensor import Sensor
from control import pid_quad
import numpy as np

class Quad:
    """
    Represents the quadcopter, handling state estimation, sensor fusion,
    and communication.
    """
    def __init__(self, dt=0.05):
        """
        Initializes the quadcopter system.
        Args:
            dt (float): The time step for integration and control loops.
        """
        self.sensor = Sensor()
        sensores = self.sensor.read()
        headling = sensores['mag'][3]

        self.dt = dt

        self.x = self.y = self.z = 0
        self.phi = self.theta = 0
        self.psi = headling

        self.u = self.v = self.w = 0
        self.p = self.q = self.r = 0

        self.pid = pid_quad(T=self.dt)

        try:
            self.serial = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            time.sleep(2)
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            print("Running in simulation mode without Arduino communication.")
            self.serial = None

        # Criar arquivo CSV para salvar os dados
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = f"log_voo_{now}.csv"
        with open(self.log_filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp",
                "x", "y", "z", "phi", "theta", "psi",
                "u", "v", "w", "p", "q", "r",
                "x_sp", "y_sp", "z_sp", "yaw_sp",
                "w1", "w2", "w3", "w4",
                "ax", "ay", "az", "p_meas", "q_meas", "r_meas",
                "mx", "my", "mz", "altitude"
            ])

    def salvar_dados(self, x_sp, y_sp, z_sp, yaw_sp, w1, w2, w3, w4, sensores):
        timestamp = time.time()
        ax, ay, az = sensores['acc']
        p_meas, q_meas, r_meas = sensores['gyro']
        mx, my, mz = sensores['mag']
        altitude = sensores['bar']

        with open(self.log_filename, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp,
                self.x, self.y, self.z, self.phi, self.theta, self.psi,
                self.u, self.v, self.w, self.p, self.q, self.r,
                x_sp, y_sp, z_sp, yaw_sp,
                w1, w2, w3, w4,
                ax, ay, az, p_meas, q_meas, r_meas,
                mx, my, mz, altitude
            ])

    def att_state(self):
        sensores = self.sensor.read()
        ax, ay, az = sensores['acc']
        p_meas, q_meas, r_meas = sensores['gyro']
        mx, my, mz = sensores['mag'][0:3]
        altitude = sensores['bar']

        c_phi, s_phi = np.cos(self.phi), np.sin(self.phi)
        c_theta, s_theta = np.cos(self.theta), np.sin(self.theta)
        t_theta = np.tan(self.theta)

        T = [[1, s_phi * t_theta, c_phi * t_theta],
             [0, c_phi, -s_phi],
             [0, s_phi / c_theta, c_phi / c_theta]]

        self.p, self.q, self.r = p_meas, q_meas, r_meas

        self.phi += (self.p * T[0][0] + self.q * T[0][1] + self.r * T[0][2]) * self.dt
        self.theta += (self.p * T[1][0] + self.q * T[1][1] + self.r * T[1][2]) * self.dt
        self.psi += (self.p * T[2][0] + self.q * T[2][1] + self.r * T[2][2]) * self.dt

        self.phi = 0.98 * self.phi + 0.02 * np.arctan2(ay, az)
        self.theta = 0.98 * self.theta + 0.02 * np.arctan2(-ax, np.sqrt(ay**2 + az**2))

        psi_mag = np.arctan2(-my * c_phi + mz * s_phi,
                             mx * c_theta + my * s_theta * s_phi + mz * s_theta * c_phi)
        self.psi = 0.98 * self.psi + 0.02 * psi_mag

        self.phi = (self.phi + np.pi) % (2 * np.pi) - np.pi
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi
        self.psi = (self.psi + np.pi) % (2 * np.pi) - np.pi

        c_psi, s_psi = np.cos(self.psi), np.sin(self.psi)

        R = [[c_psi*c_theta, c_psi*s_theta*s_phi - s_psi*c_phi, c_psi*s_theta*c_phi + s_psi*s_phi],
             [s_psi*c_theta, s_psi*s_theta*s_phi + c_psi*c_phi, s_psi*s_theta*c_phi - c_psi*s_phi],
             [-s_theta,      c_theta*s_phi,                     c_theta*c_phi]]

        g = 9.81
        gravidade_corpo = [-R[2][0]*g, -R[2][1]*g, -R[2][2]*g]

        self.u += (ax - gravidade_corpo[0]) * self.dt
        self.v += (ay - gravidade_corpo[1]) * self.dt
        self.w += (az - gravidade_corpo[2]) * self.dt

        vel_mundo = [
            R[0][0]*self.u + R[0][1]*self.v + R[0][2]*self.w,
            R[1][0]*self.u + R[1][1]*self.v + R[1][2]*self.w,
            R[2][0]*self.u + R[2][1]*self.v + R[2][2]*self.w
        ]

        self.x += vel_mundo[0] * self.dt
        self.y += vel_mundo[1] * self.dt
        z_integrated = self.z + vel_mundo[2] * self.dt
        self.z = 0.98 * z_integrated + 0.02 * altitude

    def log_estado(self):
        phi_deg = np.degrees(self.phi)
        theta_deg = np.degrees(self.theta)
        psi_deg = np.degrees(self.psi)

        log_formatado = f"""
        ----------------------------------------------------------------------
        Position (m):       x={self.x: 2.2f}, y={self.y: 2.2f}, z={self.z: 2.2f}
        Attitude (deg):     φ={phi_deg: 2.2f}, θ={theta_deg: 2.2f}, ψ={psi_deg: 2.2f}
        Linear Vel (m/s):   u={self.u: 2.2f}, v={self.v: 2.2f}, w={self.w: 2.2f}
        Angular Vel (rad/s):p={self.p: 2.2f}, q={self.q: 2.2f}, r={self.r: 2.2f}
        ----------------------------------------------------------------------
        """
        print(log_formatado)

if __name__ == "__main__":
    quad = Quad(dt=0.05)
    quad.sensor.log()

    print("--- Setpoint Configuration ---")
    try:
        x_sp = float(input("Enter the setpoint for X position (m): "))
        y_sp = float(input("Enter the setpoint for Y position (m): "))
        z_sp = float(input("Enter the setpoint for Z position (height) (m): "))
        yaw_sp_deg = float(input("Enter the setpoint for Yaw (degrees): "))
        yaw_sp = np.radians(yaw_sp_deg)
        print("Setpoints configured successfully.")
    except ValueError:
        print("Invalid input. Using default setpoints.")
        x_sp = 0.0
        y_sp = 0.0
        z_sp = 1.0
        yaw_sp = 0.0

    try:
        command = ""
        while True:
            tic = time.time()

            quad.att_state()
            sensores = quad.sensor.read()

            w1, w2, w3, w4 = quad.pid.control(
                quad.x, quad.y, quad.z,
                quad.phi, quad.theta, quad.psi,
                x_sp=x_sp, y_sp=y_sp, z_sp=z_sp, yaw_sp=yaw_sp
            )

            if quad.serial:
                command = f"{w1:.4f} {w2:.4f} {w3:.4f} {w4:.4f}\n"
                quad.serial.write(command.encode())

            quad.sensor.log()
            quad.log_estado()
            quad.salvar_dados(x_sp, y_sp, z_sp, yaw_sp, w1, w2, w3, w4, sensores)

            print(f"\nTarget -> x: {x_sp}m, y: {y_sp}m, z: {z_sp}m, yaw: {np.degrees(yaw_sp):.2f}°\n")
            print(f"Command Sent: {command.strip()}")
            toc = time.time()
            print(f"Loop execution time: {toc - tic:.4f} seconds")

    except KeyboardInterrupt:
        print("Simulation interrupted by user.")
    finally:
        if quad.serial and quad.serial.is_open:
            quad.serial.close()
            print("Serial port closed.")
