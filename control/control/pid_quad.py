from .pid import pid
import numpy as np

class pid_quad:
    '''A class for a PID controller designed for quadcopter control.
    It includes PID controllers for position (x, y), altitude (z) and attitude (roll, pitch, yaw).'''
    def __init__(self, T = 0.01):

        self.b = 1
        self.d = 1
        self.l = 1
        self.w_max = (2*np.pi/60)*1534*11.1 #2pi/60 = 1 revolution per second, 1534 is the motor KV, 11.1 is the max battery voltage

        self.mma = [[1/(4*self.b), -np.sqrt(2)/(4*self.b*self.l),  np.sqrt(2)/(4*self.b*self.l), -1/(4*self.d)],
                    [1/(4*self.b), -np.sqrt(2)/(4*self.b*self.l), -np.sqrt(2)/(4*self.b*self.l),  1/(4*self.d)], 
                    [1/(4*self.b),  np.sqrt(2)/(4*self.b*self.l),  np.sqrt(2)/(4*self.b*self.l),  1/(4*self.d)],
                    [1/(4*self.b),  np.sqrt(2)/(4*self.b*self.l), -np.sqrt(2)/(4*self.b*self.l), -1/(4*self.d)]]
        



        # Initialize PID controllers for position, altitude, and attitude
        self.pid_x = pid(T=T, kp=0.02, kd=0.1, saturation=np.pi*12/180) 
        self.pid_y = pid(T=T, kp=0.02, kd=0.1, saturation=np.pi*12/180)
        self.pid_z = pid(T=T, kp=10, ki=2.5)
        self.pid_roll = pid(T=T, kp=0.05, kd=0.1)
        self.pid_pitch = pid(T=T, kp=0.05, kd=0.1)
        self.pid_yaw = pid(T=T, kp=0.05, kd=0.2)
    
    def control(self, x, y, z, roll, pitch, yaw, x_sp, y_sp, z_sp, yaw_sp) -> tuple:
        '''Calculates the control signals for the quadcopter based on the current state and setpoints.
        Returns a tuple of motor speeds (w1, w2, w3, w4).'''
        # Calculate the position error in the inertial frame
        x_error = x_sp - x
        y_error = y_sp - y
        z_error = z_sp - z
        
        # Rotate the position error to the body frame
        x_error_boryframe = x_error*np.cos(yaw) + y_error*np.sin(yaw)
        y_error_boryframe = -x_error*np.sin(yaw) + y_error*np.cos(yaw)
        z_error_boryframe = z_error/(np.cos(roll)*np.cos(pitch))
        
        
        # Calculate the new setpoints for roll and pitch
        # based on the position error in the body frame
        roll_sp = self.pid_x.control(x_error_boryframe)
        pitch_sp = -self.pid_y.control(y_error_boryframe)
        
        # Calculate the control for altitude
        F_z = self.pid_z.control(z_error_boryframe)

        # Calculate the control for roll, pitch, and yaw
        roll_error = roll_sp - roll
        pitch_error = pitch_sp - pitch
        yaw_error = yaw_sp - yaw
        tau_phi = self.pid_roll.control(roll_error)
        tau_theta = self.pid_pitch.control(pitch_error)
        tau_psi = self.pid_yaw.control(yaw_error)
        m1 = F_z*self.mma[0][0] + tau_phi*self.mma[0][1] + tau_theta*self.mma[0][2] + tau_psi*self.mma[0][3]
        m2 = F_z*self.mma[1][0] + tau_phi*self.mma[1][1] + tau_theta*self.mma[1][2] + tau_psi*self.mma[1][3]
        m3 = F_z*self.mma[2][0] + tau_phi*self.mma[2][1] + tau_theta*self.mma[2][2] + tau_psi*self.mma[2][3]
        m4 = F_z*self.mma[3][0] + tau_phi*self.mma[3][1] + tau_theta*self.mma[3][2] + tau_psi*self.mma[3][3]

        m1 = np.clip(m1, 0, self.w_max**2)
        m2 = np.clip(m2, 0, self.w_max**2)
        m3 = np.clip(m3, 0, self.w_max**2)
        m4 = np.clip(m4, 0, self.w_max**2)

        w1 = np.sqrt(m1)/self.w_max
        w2 = np.sqrt(m2)/self.w_max
        w3 = np.sqrt(m3)/self.w_max
        w4 = np.sqrt(m4)/self.w_max

        return (w1, w2, w3, w4)


