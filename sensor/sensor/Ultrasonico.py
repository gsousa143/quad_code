import RPi.GPIO as GPIO
import time

class Ultrasonico:
    def __init__(self, trigger=23, echo=24):
        self.trigger_pin = trigger
        self.echo_pin = echo
        self.timeout = 0.1 # Timeout de 100ms para evitar bloqueio

        # Configura o modo GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        GPIO.output(self.trigger_pin, False)
        time.sleep(1) # Aguarda o sensor estabilizar
        
    
    def read(self):
        # Envia um pulso de 10µs para o pino de trigger
        GPIO.output(self.trigger_pin, True)
        time.sleep(0.00001)
        GPIO.output(self.trigger_pin, False)

        pulse_start_time = time.time()
        start_loop = time.time()
        # Aguarda o início do eco (com timeout)
        while GPIO.input(self.echo_pin) == 0:
            pulse_start_time = time.time()
            if pulse_start_time - start_loop > self.timeout:
                return -1 # Retorna um valor de erro

        pulse_end_time = time.time()
        start_loop = time.time()
        # Aguarda o fim do eco (com timeout)
        while GPIO.input(self.echo_pin) == 1:
            pulse_end_time = time.time()
            if pulse_end_time - start_loop > self.timeout:
                return -1 # Retorna um valor de erro

        # Calcula a distância em metros
        pulse_duration = pulse_end_time - pulse_start_time
        distance = pulse_duration * 171.50  # Velocidade do som (343 m/s)
        return distance
