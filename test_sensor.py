from sensor import Sensor
import time

if __name__ == "__main__":
	sensores = Sensor()
	tic = time.time()
	sensores.ler_sensores()
	sensores.imprimir_sensores()
	toc = time.time() - tic
	print(toc)
