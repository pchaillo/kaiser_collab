#!/usr/bin/env python;
# -*- coding: utf-8 -*-
import Sofa.Core
import Sofa.constants.Key as Key
import os
import csv
import time
import numpy
import math
import serial
import six
from spicy import *
from datetime import datetime

try :
    import PolhemusUSB as PolhemusUSB
except :
    import polhemus_liberty.python.PolhemusUSB as PolhemusUSB

# import Connexion_Function_ucl as connect # ATTENTION => cette ligne dépend de la fonction de connexion utilisé (ucl_collaboration)
import Connexion_Function_kais as connect # ATTENTION => cette ligne dépend de la fonction de connexion utilisé (kaiserlautern)

class PolhemusTracking(Sofa.Core.Controller):
        """
Fonction to upload Polhemus data in SOFA

        """
        def __init__(self,node, name,offset = [0,0,0],*args, **kwargs):
            Sofa.Core.Controller.__init__(self,args,kwargs)
            self.stiffNode = node
            self.position = self.stiffNode.getObject(name)

            self.conv = 10 # conversion coefficient, to pass from cm to mm


            # QUESTION(Damien): is this only once ? 
            self.p = PolhemusUSB.PolhemusUSB() # only once ? 

            for i in range(10): # on récupère les positions 10 fois, pour éviter les valeurs fausses à l'initialisation, qui entreneraient un recalage des référentiels faux
                self.p.UpdateSensors()
                position = self.p.sensors[0].GetLastPosition()

            x_i, y_i, z_i = position

            self.inv = 1 # -1 => inversion de l'axe z // 1 = > pas d'inversion # !!!! A faire mettre en argument de la fonction ! !!!!!!!!!!!
            # inversion z pour réaligner les axes SOFA et polhemus
            if self.inv == -1:
                z_i = - z_i
            self.displacement = [ offset[0]-x_i*self.conv, offset[1]-y_i*self.conv, offset[2]-z_i*self.conv ]

        def onAnimateBeginEvent(self,e):
            self.p.UpdateSensors()
            position = self.p.sensors[0].GetLastPosition()
            pos_raw = position[:3]

            pos = [pos_raw[0]*self.conv + self.displacement[0], pos_raw[1]*self.conv + self.displacement[1],  self.inv*pos_raw[2]*self.conv + self.displacement[2]]
            self.position.position = [pos]

## Exemple du capteur Aurora (similaire au Polhemus (UCL))
#            nb_module = module.nb_module
#            h_module = module.h_module
#            z_eff_pos = nb_module * h_module 
# AuroraTracking(stiffNode, mechanicalPosition, offset=[nb_mobule*h_module])
class AuroraTracking(Sofa.Core.Controller):
        """Doc string"""
        def __init__(self, child_name, name, offset=[0,0,0], *args, **kwargs):
            Sofa.Core.Controller.__init__(self,args,kwargs)
            self.RootNode = kwargs["RootNode"]        # aurora setting 
            self.settings_aurora = { "tracker type": "aurora", "ports to use" : [10]}
            self.tracker = NDITracker(self.settings_aurora)
            self.tracker.start_tracking()

            self.stiffNode = self.RootNode.getChild(child_name) # for the generic one
            self.position = self.stiffNode.getObject(name)

            # first frames are invalid so we drop a given number of them
            for frame_to_drop in range(10):
                self.tracker.get_frame()
            
            pos_raw = get_data()
            self.displacement = [-pos_raw[0]+offset[0], pos_raw[1]+offset[1], -pos_raw[2]+offset[2]]

        def get_data():
            self.aurora_frame = self.tracker.get_frame();
            data = self.aurora_frame[3][0] 
            
            x_i = data[0][3]
            y_i = data[1][3]
            z_i = data[2][3]
            return [x_i, y_i, z_i]

        def onAnimateBeginEvent(self,e):
            pos_raw = get_data()
            pos = [pos_raw[0] + self.displacement[0], pos_raw[1] + self.displacement[1],  pos_raw[2] + self.displacement[2]]
            self.position.position = [pos]

