import RPi.GPIO as GPIO
import time

# Configuração dos pinos
TRIG = 23 #pino 16
ECHO = 24 #pino 18

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def medir_distancia():
    # Garante que o TRIG está desligado
    GPIO.output(TRIG, False)
    time.sleep(0.5)

    # Envia pulso de 10µs para o TRIG
    GPIO.output(TRIG, True)
    time.sleep(0.00001)  # 10µs
    GPIO.output(TRIG, False)

    # Espera o início do sinal no ECHO
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    # Espera o final do sinal no ECHO
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    # Calcula a duração do pulso
    pulse_duration = pulse_end - pulse_start

    # Calcula a distância (velocidade do som = 34300 cm/s)
    distancia = pulse_duration * 17150
    distancia = round(distancia, 2)

    return distancia

try:
    while True:
        dist = medir_distancia()
        print(f"Distância: {dist} cm")
        time.sleep(1)

except KeyboardInterrupt:
    print("Medida interrompida pelo usuário")

finally:
    GPIO.cleanup()

