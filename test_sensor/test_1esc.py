import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)

try:
    while True:
        texto = input("Digite velocidade (1000-2000 µs): ")
        if texto.lower() == "sair":
            break
        ser.write((texto + '\n').encode())
        print("Enviado:", texto)
finally:
    ser.close()
