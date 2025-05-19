# main.py
from .adxl345 import ADXL345
from .l3g4200d import L3G4200D
from .hmc5883l import HMC5883L
from .bmp180 import BMP180
import time

def main():
    acc = ADXL345()
    gyro = L3G4200D()
    mag = HMC5883L()
    baro = BMP180()

    print("Calibrando giroscópio...")
    gyro.calibrate()

    start_time = time.time()
    duration = 60

    try:
        while time.time() - start_time < duration:
            ax, ay, az = acc.read()
            gx, gy, gz = gyro.read()
            mx, my, mz, heading = mag.read()
            temp, press, alt = baro.read()

            print(f"Acel: X={ax:.2f}, Y={ay:.2f}, Z={az:.2f}")
            print(f"Giro: X={gx:.2f}, Y={gy:.2f}, Z={gz:.2f}")
            print(f"Mag: X={mx}, Y={my}, Z={mz}, Heading={heading:.1f}°")
            print(f"Temp: {temp:.2f} °C | Pressão: {press} Pa | Altitude: {alt:.2f} m")
            print("-" * 60)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Leitura interrompida pelo usuário.")

if __name__ == "__main__":
    main()
