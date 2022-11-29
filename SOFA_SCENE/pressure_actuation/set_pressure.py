import numpy as np
import time
import serial


import PressureActuation_fcn as actuation # V002 #

pv = 0.5
pres = [pv,0,0]

## V001 ##

# SerialObj1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5) #port used by the arduino mega board

# # pv = 0.7 # pressure value (bar)
# pv = 0.5
# pres = [pv,0,0]

# # pres = [1.5,0,0]

# # nb_tr = 3 # nb de trames
# # on envoie 3 trames à chaque fois pour être sûr que le message passe

# # for i in range(nb_tr):
# while(1):
# 	S = "{:,.3f}".format(pres[0]) + "," + "{:,.3f}".format(pres[1]) + "," + "{:,.3f}".format(pres[2]) + "\n"
# 	# print(i)
# 	print(S)
# 	ByteStr = S.encode("utf-8")
# 	# print(ByteStr)
# 	SerialObj1.write(ByteStr)
# 	time.sleep(0.2)

# L = SerialObj1.readlines()
# print(L)

## V002 ##*

Pressure_actuation = actuation.PressureActuation()

while(1):
	Pressure_actuation.SetPressure(p1 = pres[0],p2 = pres[1],p3 = pres[2])
	time.sleep(0.2)
	# S = "{:,.3f}".format(pres[0]) + "," + "{:,.3f}".format(pres[1]) + "," + "{:,.3f}".format(pres[2]) + "\n"
	# print(S)