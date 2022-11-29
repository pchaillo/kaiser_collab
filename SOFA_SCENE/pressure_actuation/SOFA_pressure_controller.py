import Sofa.Core
import Sofa.constants.Key as Key
from spicy import *
import serial

# import Connexion_Function_ucl as connect # ATTENTION => cette ligne dépend de la fonction de connexion utilisé (ucl_collaboration)
import Connexion_Function_kais as connect # ATTENTION => cette ligne dépend de la fonction de connexion utilisé (kaiserlautern)


try :
    import PressureActuation_fcn as actuation
except : 
    import pressure_actuation.PressureActuation_fcn as actuation

class ArduinoPressure(Sofa.Core.Controller): # à déporter dans le fichier pressure_actuation (bonne pratique)
    """
    # pour envoyer les pressions avec les valves connectées à la MegaPi 
    # DEFROST SETUP

    INPUT : 
    module = variable stiff qui contient toutes les données du robot
    """
    def __init__(self,module,node,*args, **kwargs):
        Sofa.Core.Controller.__init__(self,*args,**kwargs)
        # self.RootNode = kwargs["RootNode"]
        # self.stiffNode = self.RootNode.getChild('RigidFrames')
        # self.position = self.stiffNode.getObject('DOFs')
        # self.nb_poutre = module.nb_poutre
        # self.nb_module = module.nb_module
        # self.nb_cavity = module.nb_cavity
        # self.step = step
        self.IterSimu = 0 # Counter for dt steps before stopping simulation
        self.ecart = 0 # ecart entre la simulation et la réalité, en mm
        ind = -1
        self.pressure, txt_chmbre = connect.CavityConnect(node=node,module=module)

        if module.dyn_flag == 1:
            self.time_step = module.dt
        else :
            self.time_step = 1

        # self.board = pyfirmata.Arduino('/dev/ttyACM0') # pyfirmata connexion
        # self.led = self.board.get_pin('d:13:o')

        self.Pressure_actuation = actuation.PressureActuation()
 

    def onAnimateBeginEvent(self, dt): 
        print("TEST")
        pres_tab = [copy(self.pressure[0].pressure.value)/self.time_step,copy(self.pressure[1].pressure.value)/self.time_step,copy(self.pressure[2].pressure.value)/self.time_step]
        bar_tab = connect.kPa_to_bar(pres_tab)
        self.Pressure_actuation.SetPressure(p1 = bar_tab[0],p2 = bar_tab[1],p3 = bar_tab[2])  


# ## Old version 
# class ArduinoPressure(Sofa.Core.Controller): # à déporter dans le fichier pressure_actuation (bonne pratique)
#     """
#     # pour envoyer les pressions avec les valves connectées à la MegaPi 
#     # DEFROST SETUP

#     INPUT : 
#     module = variable stiff qui contient toutes les données du robot
#     """
#     def __init__(self,module,node,*args, **kwargs):
#         Sofa.Core.Controller.__init__(self,*args,**kwargs)
#         # self.RootNode = kwargs["RootNode"]
#         # self.stiffNode = self.RootNode.getChild('RigidFrames')
#         # self.position = self.stiffNode.getObject('DOFs')
#         self.nb_poutre = module.nb_poutre
#         self.nb_module = module.nb_module
#         self.nb_cavity = module.nb_cavity
#         # self.step = step
#         self.IterSimu = 0 # Counter for dt steps before stopping simulation
#         self.ecart = 0 # ecart entre la simulation et la réalité, en mm
#         ind = -1
#         self.pressure, txt_chmbre = connect.CavityConnect(node=node,module=module)

#         if module.dyn_flag == 1:
#             self.time_step = module.dt
#         else :
#             self.time_step = 1

#         # self.board = pyfirmata.Arduino('/dev/ttyACM0') # pyfirmata connexion
#         # self.led = self.board.get_pin('d:13:o')

#         self.SerialObj1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5) #port used by the arduino mega board
 

#     def onAnimateBeginEvent(self, dt): 
#         pres_tab = [copy(self.pressure[0].pressure.value)/self.time_step,copy(self.pressure[1].pressure.value)/self.time_step,copy(self.pressure[2].pressure.value)/self.time_step]
#         # print(pres_tab)

#         bar_tab = connect.kPa_to_bar(pres_tab)

#         S = "{:,.3f}".format(bar_tab[0]) + "," + "{:,.3f}".format(bar_tab[1]) + "," + "{:,.3f}".format(bar_tab[2]) + "\n"

#         print(S)
#         ByteStr = S.encode("utf-8")

#         self.SerialObj1.write(ByteStr)
#         # time.sleep(0.2) #usefull ?
#         # print("Step: " + str(i) + ", Bytes sent: " + S)
#         # print("  Bytes sent: " + S)

#         # # pour le test avec la led
#         # pres = pres_tab[0]
#         # if pres > 50 :
#         #     self.led.write(1)
#         # else :
#         #     self.led.write(0)