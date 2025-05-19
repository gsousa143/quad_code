import smbus
import time
import math

bus = smbus.SMBus(1)

# === ENDEREÇOS ===
ADXL345_ADDR = 0x53
L3G4200D_ADDR = 0x69
HMC5883L_ADDR = 0x1E
BMP180_ADDR = 0x77

# === Funções auxiliares ===
def to_signed(val):
    return val - 65536 if val > 32767 else val

def read_word_be(addr, reg):
    msb = bus.read_byte_data(addr, reg)
    lsb = bus.read_byte_data(addr, reg + 1)
    return (msb << 8) + lsb

def to_signed_16(val):
    return val - 65536 if val > 32767 else val

# === ADXL345 - Acelerômetro ===
def setup_adxl345():
    bus.write_byte_data(ADXL345_ADDR, 0x2D, 0x08)
    bus.write_byte_data(ADXL345_ADDR, 0x31, 0x08)

def read_adxl345():
    data = bus.read_i2c_block_data(ADXL345_ADDR, 0x32, 6)
    x = to_signed(data[1] << 8 | data[0]) * 0.004
    y = to_signed(data[3] << 8 | data[2]) * 0.004
    z = to_signed(data[5] << 8 | data[4]) * 0.004
    return x, y, z

def read_adxl345_avg(samples=10):
    sum_x = sum_y = sum_z = 0
    for _ in range(samples):
        x, y, z = read_adxl345()
        sum_x += x
        sum_y += y
        sum_z += z
        time.sleep(0.01)
    return sum_x / samples, sum_y / samples, sum_z / samples

# === L3G4200D - Giroscópio ===
def setup_l3g4200d():
    bus.write_byte_data(L3G4200D_ADDR, 0x20, 0x0F)
    bus.write_byte_data(L3G4200D_ADDR, 0x23, 0x30)

def read_l3g4200d():
    data = bus.read_i2c_block_data(L3G4200D_ADDR, 0x28 | 0x80, 6)
    x = to_signed(data[1] << 8 | data[0])
    y = to_signed(data[3] << 8 | data[2])
    z = to_signed(data[5] << 8 | data[4])
    return x, y, z

def calibrate_gyro(samples=300):
    sum_x = sum_y = sum_z = 0
    for _ in range(samples):
        x, y, z = read_l3g4200d()
        sum_x += x
        sum_y += y
        sum_z += z
        time.sleep(0.01)
    return sum_x / samples, sum_y / samples, sum_z / samples

# === HMC5883L - Magnetômetro ===
def setup_hmc5883l():
    bus.write_byte_data(HMC5883L_ADDR, 0x00, 0x70)
    bus.write_byte_data(HMC5883L_ADDR, 0x01, 0xA0)
    bus.write_byte_data(HMC5883L_ADDR, 0x02, 0x00)

def read_hmc5883l():
    data = bus.read_i2c_block_data(HMC5883L_ADDR, 0x03, 6)
    x = to_signed(data[0] << 8 | data[1])
    z = to_signed(data[2] << 8 | data[3])
    y = to_signed(data[4] << 8 | data[5])
    heading = math.degrees(math.atan2(y, x))
    if heading < 0:
        heading += 360
    return x, y, z, heading

# === BMP180 - Barômetro/Temperatura ===
def read_bmp180_calibration():
    calib = {}
    calib['AC1'] = to_signed_16(read_word_be(BMP180_ADDR, 0xAA))
    calib['AC2'] = to_signed_16(read_word_be(BMP180_ADDR, 0xAC))
    calib['AC3'] = to_signed_16(read_word_be(BMP180_ADDR, 0xAE))
    calib['AC4'] = read_word_be(BMP180_ADDR, 0xB0)
    calib['AC5'] = read_word_be(BMP180_ADDR, 0xB2)
    calib['AC6'] = read_word_be(BMP180_ADDR, 0xB4)
    calib['B1'] = to_signed_16(read_word_be(BMP180_ADDR, 0xB6))
    calib['B2'] = to_signed_16(read_word_be(BMP180_ADDR, 0xB8))
    calib['MB'] = to_signed_16(read_word_be(BMP180_ADDR, 0xBA))
    calib['MC'] = to_signed_16(read_word_be(BMP180_ADDR, 0xBC))
    calib['MD'] = to_signed_16(read_word_be(BMP180_ADDR, 0xBE))
    return calib

def read_bmp180_temp_pressure(calib, oversampling=3):
    # Temperatura
    bus.write_byte_data(BMP180_ADDR, 0xF4, 0x2E)
    time.sleep(0.005)
    msb = bus.read_byte_data(BMP180_ADDR, 0xF6)
    lsb = bus.read_byte_data(BMP180_ADDR, 0xF7)
    ut = (msb << 8) + lsb

    # Pressão
    bus.write_byte_data(BMP180_ADDR, 0xF4, 0x34 + (oversampling << 6))
    time.sleep(0.005 + 0.003 * (2 ** oversampling))
    msb = bus.read_byte_data(BMP180_ADDR, 0xF6)
    lsb = bus.read_byte_data(BMP180_ADDR, 0xF7)
    xlsb = bus.read_byte_data(BMP180_ADDR, 0xF8)
    up = ((msb << 16) + (lsb << 8) + xlsb) >> (8 - oversampling)

    # Cálculos de compensação
    X1 = ((ut - calib['AC6']) * calib['AC5']) >> 15
    X2 = (calib['MC'] << 11) // (X1 + calib['MD'])
    B5 = X1 + X2
    temp = ((B5 + 8) >> 4) / 10.0

    B6 = B5 - 4000
    X1 = (calib['B2'] * ((B6 * B6) >> 12)) >> 11
    X2 = (calib['AC2'] * B6) >> 11
    X3 = X1 + X2
    B3 = (((calib['AC1'] * 4 + X3) << oversampling) + 2) >> 2

    X1 = (calib['AC3'] * B6) >> 13
    X2 = (calib['B1'] * ((B6 * B6) >> 12)) >> 16
    X3 = ((X1 + X2) + 2) >> 2
    B4 = (calib['AC4'] * (X3 + 32768)) >> 15
    B7 = (up - B3) * (50000 >> oversampling)

    if B7 < 0x80000000:
        p = (B7 * 2) // B4
    else:
        p = (B7 // B4) * 2

    X1 = (p >> 8) * (p >> 8)
    X1 = (X1 * 3038) >> 16
    X2 = (-7357 * p) >> 16
    pressure = p + ((X1 + X2 + 3791) >> 4)

    return temp, pressure

def pressure_to_altitude(pressure, sea_level=101325.0):
    return 44330.0 * (1.0 - (pressure / sea_level) ** (1 / 5.255))

# === Loop principal ===
def main():
    setup_adxl345()
    setup_l3g4200d()
    setup_hmc5883l()
    calib = read_bmp180_calibration()
    print("Calibrando giroscópio...")
    gx_bias, gy_bias, gz_bias = calibrate_gyro()
    start_time = time.time()
    duration = 120

    try:
        while time.time() - start_time < duration:
            ax, ay, az = read_adxl345_avg()
            gx, gy, gz = read_l3g4200d()
            gx -= gx_bias
            gy -= gy_bias
            gz -= gz_bias
            mx, my, mz, heading = read_hmc5883l()
            temp, press = read_bmp180_temp_pressure(calib)
            altitude = pressure_to_altitude(press)

            print(f"Acel: X={ax:.2f}, Y={ay:.2f}, Z={az:.2f}")
            print(f"Giro: X={gx:.2f}, Y={gy:.2f}, Z={gz:.2f}")
            print(f"Mag: X={mx}, Y={my}, Z={mz}, Heading={heading:.1f}°")
            print(f"Temp: {temp:.2f} °C | Pressão: {press} Pa | Altitude: {altitude:.2f} m")
            print("-" * 60)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Leitura interrompida pelo usuário.")

    print("Coleta finalizada.")

if __name__ == "__main__":
    main()
