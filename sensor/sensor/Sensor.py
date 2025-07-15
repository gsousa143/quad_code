from .Acelerometro import Acelerometro
from .Giroscopio import Giroscopio 
from .Magnetometro import Magnetometro
from .Ultrasonico import Ultrasonico
from .Barometro import Barometro

class Sensor:
    def __init__(self):
        self.acc = Acelerometro()
        self.gyro = Giroscopio()
        self.mag = Magnetometro()
        self.bar = Barometro()
        self.ult  = Ultrasonico()
        self.ax = self.ay = self.az = 0.0
        self.p = self.q = self.r = 0.0
        self.mx = self.my = self.mz = self.heading =  0.0
        self.z_bar = 0.0
        self.z_ult = 0.0


    def ler_sensores(self):
        self.ax, self.ay, self.az = self.acc.read()
        self.p, self.q, self.r = self.gyro.read()
        self.mx, self.my, self.mz, self.heading = self.mag.read()
        self.z_bar = self.bar.read()
        self.z_ult = self.ult.read()

        return {
            'acc': (self.ax, self.ay, self.az),
            'gyro': (self.p, self.q, self.r),
            'mag': (self.mx, self.my, self.mz, self.heading),
            'bar': self.z_bar,
            'ult': self.z_ult
        }
    
    def imprimir_sensores(self):
        print(f"Acelerômetro (m/s^2): \t \t ax:{self.ax}, ay:{self.ay}, az:{self.az}")
        print(f"Giroscópio: (rad/s): \t \t p:{self.p}, q:{self.q}, r:{self.r}")
        print(f"Magnetômetro: (T): \t \t mx:{self.mx}, my:{self.my}, mz:{self.mz}, heading:{self.heading} rad")
        print(f"Barômetro: (m): \t \t z:{self.z_bar}")
        print(f"Ultrassônico: (m):  \t \t z:{self.z_ult}")