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
        # Rótulos alinhados à esquerda em um espaço de 22 caracteres para organização.
        # Valores de ponto flutuante (float) formatados para exibir 4 casas decimais.
        print(f"Acelerômetro (m/s²): ax:{self.ax: .4f}, ay:{self.ay: .4f}, az:{self.az: .4f}")
        print(f"Giroscópio (rad/s): p:{self.p: .4f}, q:{self.q: .4f}, r:{self.r: .4f}")
        print(f"Magnetômetro (µT): mx:{self.mx: .4f}, my:{self.my: .4f}, mz:{self.mz: .4f}")
        
        # Converte o heading de radianos para graus para facilitar a leitura.
        heading_em_graus = math.degrees(self.heading)
        print(f"Heading: {self.heading: .4f} rad ({heading_em_graus: .2f}°)")
        print(f"Barômetro (m): z:{self.z_bar: .4f}")
  
