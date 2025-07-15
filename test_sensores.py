from sensor import Sensor
import time

if __name__ == "__main__":
	sensores = Sensor()
	tic = time.time()
	sensores.read()
	sensores.log()
	toc = time.time() - tic
	print(toc)
