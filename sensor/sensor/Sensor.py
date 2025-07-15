from .Acelerometro import Acelerometro
from .Giroscopio import Giroscopio 
from .Magnetometro import Magnetometro
from .Barometro import Barometro

class Sensor:
    def __init__(self):
        self.acc = Acelerometro()
        self.gyro = Giroscopio()
        self.mag = Magnetometro()
        self.bar = Barometro()
        self.ax = self.ay = self.az = 0.0
        self.p = self.q = self.r = 0.0
        self.mx = self.my = self.mz = self.heading =  0.0
        self.z_bar = 0.0


    def read(self):
        self.ax, self.ay, self.az = self.acc.read()
        self.p, self.q, self.r = self.gyro.read()
        self.mx, self.my, self.mz, self.heading = self.mag.read()
        self.z_bar = self.bar.read()


        return {
            'acc': (self.ax, self.ay, self.az),
            'gyro': (self.p, self.q, self.r),
            'mag': (self.mx, self.my, self.mz, self.heading),
            'bar': self.z_bar,
        }
    
    def log(self):
        print(f"Acelerômetro (m/s^2): \t \t ax:{self.ax}, ay:{self.ay}, az:{self.az}")
        print(f"Giroscópio: (rad/s): \t \t p:{self.p}, q:{self.q}, r:{self.r}")
        print(f"Magnetômetro: (T): \t \t mx:{self.mx}, my:{self.my}, mz:{self.mz}, heading:{self.heading} rad")
        print(f"Barômetro: (m): \t \t z:{self.z_bar}")
        print("--------------------------------------------")
