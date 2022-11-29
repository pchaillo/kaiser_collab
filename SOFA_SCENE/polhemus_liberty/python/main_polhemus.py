#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 25 17:23:53 2022

@author: pchaillo
"""

# import serial

# a tester !!! (sinon revenir au commit précédent)


import PolhemusUSB

p = PolhemusUSB.PolhemusUSB() # only once

nom_fichier = "polhemus_record_02.txt"

f = open(nom_fichier,'a')

while 1 :
    p.UpdateSensors()
    position = p.sensors[0].GetLastPosition()
    print('Position  : '+str(position))
    f.write(str(position)+'\n')
    f.close()
    f = open(nom_fichier,'a')
    

    # senso = s.sensor
    # print(str(senso.GetPressure()))
    # print(str(s.sensor.GetPressure()))

# ser = serial.Serial('/dev/ttyACM0',9600) # choisir le bon port serie si windows ('com1',9600) par exemple

# f = open('fichier.txt','a')

# while 1 :
#     print(str(ser.readline()))
#     f.write(str(ser.readline())+'\n')
#     f.close()
#     f = open('fichier.txt','a')