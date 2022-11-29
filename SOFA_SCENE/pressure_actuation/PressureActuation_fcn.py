import numpy as np
import time
import serial


class PressureActuation:
	def __init__(self):
		self.port = '/dev/ttyUSB0'
		self.baud = 115200
		self.ser = serial.Serial(self.port, self.baud , timeout=0.5) #port used by the arduino mega board
# SerialObj1.open()

	def SetPressure(self,p1,p2,p3):
		# Attention, si la mise à jour des trames est trop rapide => tremblement, actionnement non stable et inconstant

		pres = [p1,p2,p3]

		S = "{:,.3f}".format(pres[0]) + "," + "{:,.3f}".format(pres[1]) + "," + "{:,.3f}".format(pres[2]) + "\n"

		print(S)
		ByteStr = S.encode("utf-8")

		self.ser.write(ByteStr)

		# S = "{:,.3f}".format(pres[0]) + "," + "{:,.3f}".format(pres[1]) + "," + "{:,.3f}".format(pres[2]) + "\n"
		# # print(i)
		# print(S)
		# ByteStr = S.encode("utf-8")
		# # print(ByteStr)
		# SerialObj1.write(ByteStr)

