import serial
import time

# Configure a serial (ajuste o device conforme seu setup)
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)  # espera estabilizar a conexão serial

try:
    while True:
        texto = input("Digite um texto para enviar ao Arduino: ")
        if texto.lower() == "sair":
            break
        ser.write((texto + '\n').encode())  # envia o texto com quebra de linha
        print("Enviado:", texto)
finally:
    ser.close()
