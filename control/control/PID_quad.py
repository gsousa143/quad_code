from .PID import PID
import numpy as np

class PID_quad:
    '''A class for a PID controller designed for quadcopter control.
    It includes PID controllers for position (x, y), altitude (z) and attitude (roll, pitch, yaw).'''
    def __init__(self, T = 0.01):

        self.b = 1
        self.d = 1
        self.l = 1
        self.w_max = (2*np.pi/60)*1534*11.1 #2pi/60 = 1 revolution per second, 1534 is the motor KV, 11.1 is the max battery voltage

        self.mma = np.array([[1/(4*self.b), -np.sqrt(2)/(4*self.b*self.l),  np.sqrt(2)/(4*self.b*self.l), -1/(4*self.d)],
                             [1/(4*self.b), -np.sqrt(2)/(4*self.b*self.l), -np.sqrt(2)/(4*self.b*self.l),  1/(4*self.d)], 
                             [1/(4*self.b),  np.sqrt(2)/(4*self.b*self.l),  np.sqrt(2)/(4*self.b*self.l),  1/(4*self.d)],
                             [1/(4*self.b),  np.sqrt(2)/(4*self.b*self.l), -np.sqrt(2)/(4*self.b*self.l), -1/(4*self.d)]])
        
        self.U = np.array([0, 0, 0, 0])  # Control input vector


        # Initialize PID controllers for position, altitude, and attitude
        self.pid_x = PID(T=T, kp=0.02, kd=0.1, saturation=np.pi*10/180) 
        self.pid_y = PID(T=T, kp=0.02, kd=0.1, saturation=np.pi*10/180)
        self.pid_z = PID(T=T, kp=10, ki=2.5)
        self.pid_roll = PID(T=T, kp=0.05, kd=0.1)
        self.pid_pitch = PID(T=T, kp=0.05, kd=0.1)
        self.pid_yaw = PID(T=T, kp=0.05, kd=0.2)
    
    def control(self, reference: list[float], state: list[float]) -> np.ndarray:
        # Unpack the reference vector
        x_sp = reference[0]
        y_sp = reference[1]
        z_sp = reference[2]
        yaw_sp = reference[3]
        # Unpack the state vector
        x = state[0]
        y = state[1]
        z = state[2]
        roll = state[3]
        pitch = state[4]
        yaw = state[5]
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
        self.U[0] = self.pid_z.control(z_error_boryframe)

        # Calculate the control for roll, pitch, and yaw
        roll_error = roll_sp - roll
        pitch_error = pitch_sp - pitch
        yaw_error = yaw_sp - yaw
        self.U[1] = self.pid_roll.control(roll_error)
        self.U[2] = self.pid_pitch.control(pitch_error)
        self.U[3] = self.pid_yaw.control(yaw_error)

        W =  np.sqrt(np.abs(np.matmul(self.mma, self.U)))/self.w_max
        
        W = np.clip(W, 0, 1)  # Ensure the control inputs are within bounds

        return W



