import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)  # Aguarda inicialização do Arduino

comando = "0.1,0.1,0.1,0.1\n"
ser.write(comando.encode())
ser.close()
