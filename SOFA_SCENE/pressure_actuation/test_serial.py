import numpy as np
import time
import serial

SerialObj1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5) #port used by the arduino mega board
# SerialObj1.open()

# time.sleep(0.2)

# pres = [1.8,1.8,1.8]
pres = [0.2,0.2,0.2]
# pres = [0,0,0]

nb_tr = 3 # nb de trames
# on envoie 3 trames à chaque fois pour être sûr que le message passe

for i in range(nb_tr):
# while(1):
	S = "{:,.3f}".format(pres[0]) + "," + "{:,.3f}".format(pres[1]) + "," + "{:,.3f}".format(pres[2]) + "\n"
	# print(i)
	print(S)
	ByteStr = S.encode("utf-8")
	# print(ByteStr)
	SerialObj1.write(ByteStr)
	time.sleep(0.2)

	# print("Step: " + str(i) + ", Bytes sent: " + S)


# while(1): # working
# 	kk = SerialObj1.read()
# 	print(kk)

L = SerialObj1.readlines()
print(L)