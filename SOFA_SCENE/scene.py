 # coding=utf-8

import Sofa
# import SofaPython3
from math import sin,cos, sqrt, acos
import array

from Kaiserlautern_Function import *
from Kaiserlautern_Controller import *
from TrajectoryController import *

# Sous-dépots git :
from polhemus_liberty.python.Polhemus_SOFA_Controller import *
from pressure_actuation.SOFA_pressure_controller import *

# from SimuController_kais import *
# from CloseLoopController import *
# from polhemus_liberty.python.Polhemus_SOFA_Controller import *
# from polhemus_liberty.Polhemus_SOFA_Controller import *

from stlib3.physics.mixedmaterial import Rigidify
from stlib3.physics.deformable import ElasticMaterialObject
#from tutorial import *
import time

############## PARAM7TRES A FIXER ####################

act_flag = 0 # set 0 for IP and 1 for direct control
version = 1 # // 1 => Kaiserlautern robot // 2 => 2 section kaiserlautern robot
record = 0 # 0 => no record // 1 => record
setup = 0 # 0 => no hardware // 1 => DEFROST setup 
dynamic = 0 # 0 => static (first order) // 1 => dynamic


close_loop = 0 # 0 => no close loop
if close_loop == 0 :
    K_P = 0
    K_I = 0
else :    
    K_P = 0.1
    # K_I = 0.0001
    K_I = 0

dt = 0.001

# Paramètres controller
pressure_step = 20 # pour argument controller (attention aux unités !)
max_pression = 150 # en kPa
min_pression = 0
init_pressure_value = 0
# value_type = "1" # pour commande en volume (avec beam6)
value_type = "pressure" # pour commande en pression (avec beam5)
# value_type = "volumeGrowth" # pour commande en pression (avec beam5)


# Paramètres ROBOT
nb_module = 1 # nombre de modules
# module
masse_module = 0.01 # en kg, soit 10g
nb_cavity = 3  # nombre de paires de cavités
# soft part
coef_poisson = 0.45 # coefficient de poisson
# stiff parts
rigid_base = 4 # hauteur de rigidification de la base des modules en mm
rigid_top = 2 # hauteur de rigidification de l'extrémité des modules en mm


### Goal contoller
goal_pas = 5

if version == 1 :
    h_module = 44+3+3 # hauteur du module en mm
    chamber_model =  'kaiserlautern_chambre_1section_t_remesh.stl'
    # module_model = 'kaiserlautern_1sections.vtk'
    module_model = 'kaiserlautern_1sections_simp.vtk'
    radius = 15
    YM_soft = 100 # young modulus of the soft part (kPa)
    rigid_bool = 0 # 0 => no rigid parts (pas de partie rigide) // 1 => rigids parts  
    YM_stiff_part = 1075 # young modulus of the stiff part
elif version == 2 :
    h_module = (44+3+3)*2 # hauteur du module en mm
    chamber_model =  'kaiserlautern_chambre_2sec_remsh.stl'
    module_model = 'kaiserlautern_2sections.vtk'
    radius = 15
    YM_soft = 100 # young modulus of the soft part (kPa)
    rigid_bool = 0 # 0 => no rigid parts (pas de partie rigide) // 1 => rigids parts  
    YM_stiff_part = 1075 # young modulus of the stiff part


# Paramètres Simulation
name_module = 'Module'
name_cavity = 'Bellow'
nb_poutre = nb_module*4
h_effector = h_module * nb_module

d_et_h = str(datetime.now())
nom_dossier = d_et_h[0:19]

position = [0,0,h_effector]

## Circle trajectory parameters :
circle_radius =20
nb_iter_circle = 100
circle_height = h_effector
square_height = circle_height
square_radius = 20

############## PARAM7TRES -- FIN ####################

# stiff = Stiff_Flop(h_module,init_pressure_value,value_type,YM_soft_part,YM_stiff_part,coef_poi,nb_cavity,chamber_model,nb_module,module_model,max_pression,name_cavity,masse_module,nb_poutre,rigid_base,rigid_top,rigid_bool)

def MyScene(rootNode, out_flag,step,YM_soft_part,coef_poi,act_flag,data_exp):


    kaiser = KaiserRobot(h_module,init_pressure_value,value_type,YM_soft_part,YM_stiff_part,coef_poi,nb_cavity,chamber_model,nb_module,module_model,max_pression,name_cavity,masse_module,nb_poutre,rigid_base,rigid_top,rigid_bool,min_pression,dynamic,dt)
    
    rootNode.addObject('AddPluginRepository', path = '/home/pchaillo/Documents/10-SOFA/sofa/build/master/external_directories/plugins/SoftRobots/lib/') #libSoftRobots.so 1.0
    rootNode.addObject('AddPluginRepository', path = '/home/pchaillo/Documents/10-SOFA/sofa/build/master/external_directories/plugins/ModelOrderReduction/lib/') #libSoftRobots.so 1.0
    rootNode.addObject('AddPluginRepository', path = '/home/pchaillo/Documents/10-SOFA/sofa/build/master/external_directories/plugins/BeamAdapter/lib')#/libBeamAdapter.so 1.0

    # required plugins:
    rootNode.addObject('RequiredPlugin', name='SoftRobots.Inverse')
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='BeamAdapter')
    rootNode.addObject('RequiredPlugin', name='SofaConstraint')
    rootNode.addObject('RequiredPlugin', name='SofaDeformable')
    rootNode.addObject('RequiredPlugin', name='SofaGeneralAnimationLoop')
    rootNode.addObject('RequiredPlugin', name='SofaImplicitOdeSolver')
    rootNode.addObject('RequiredPlugin', name='SofaLoader')
    rootNode.addObject('RequiredPlugin', name='SofaMeshCollision')
    rootNode.addObject('RequiredPlugin', name='SofaSimpleFem')
    rootNode.addObject('RequiredPlugin', name='SofaSparseSolver')
    rootNode.addObject('RequiredPlugin', name='SofaEngine')
    rootNode.addObject('RequiredPlugin', name='SofaGeneralLoader')
    #rootNode.addObject('RequiredPlugin', name='SofaPython')

    # rootNode.findData('gravity').value=[0, 0, 9810];
    rootNode.findData('gravity').value=[0, 0, 0];

        #visual dispaly
    rootNode.addObject('VisualStyle', displayFlags='showVisualModels showBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe')
    rootNode.addObject('BackgroundSetting', color='0 0.168627 0.211765')
    #rootNode.createObject('OglSceneFrame', style="Arrows", alignment="TopRight")
    
    rootNode.findData('dt').value= 0.001;
    
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')   

    rigidFramesNode  = rootNode.addChild('RigidFrames')
    rigidFramesNode.addObject('EulerImplicitSolver', firstOrder='1', vdamping=0)
    rigidFramesNode.addObject('SparseLDLSolver', name='ldlsolveur',template="CompressedRowSparseMatrixd")    
    # rigidFramesNode.addObject('SofaDenseSolver', name='ldlsolveur')  # test  
    rigidFramesNode.addObject('GenericConstraintCorrection')

    rigidFramesNode.addObject('RegularGridTopology',  name='meshLinesCombined',  nx=nb_poutre, ny='1', nz='1', xmax=h_module*nb_module, xmin='0.0', ymin='0', ymax='0',zmin='0',zmax='0')
    rigidFramesNode.addObject('MechanicalObject',  name='DOFs', template='Rigid3d', showObject='1', showObjectScale='1', rotation=[0, -90 ,0], translation = [0,0,0])
    rigidFramesNode.addObject('BeamInterpolation', name='BeamInterpolation', printLog = '1', defaultYoungModulus=YM_soft_part, dofsAndBeamsAligned='true', straight='1', crossSectionShape='circular', radius=radius)#, radius=5*radius)
    # rigidFramesNode.addObject('AdaptiveBeamForceFieldAndMass', name='BeamForceField', computeMass='0', massDensity=0.001)
    rigidFramesNode.addObject('RestShapeSpringsForceField', name='anchor', points='0', stiffness='1e12', angularStiffness='1e12')
      

    kaiser_robot = kaiser.createRobot(parent = rigidFramesNode, name = "MyStiffFlop",out_flag = out_flag, act_flag = act_flag)

    MeasuredPosition = EffectorGoal(node=rootNode, position = [0,0,0],name = 'MeasuredPosition',taille = 6)
    DesiredPosition = EffectorGoal(node=rootNode, position = [0,0,h_effector],name = 'DesiredPosition',taille = 0.5)

    if act_flag == 0 :
        rootNode.addObject('QPInverseProblemSolver', name="QP", printLog='0', saveMatrices = True ,epsilon = 0.01) # initialement epsilon = 0.001

        goal = EffectorGoal(node=rootNode, position = [0,0,h_effector],name = 'goal',taille = 0.5)

        ## 2nd goal point, with a shift with the 1st one, in order to catch it easily with the mouse.
        if close_loop == 0 :
            goal2 = EffectorGoal(node=rootNode, position = [0,0,h_effector+5],name = 'goal2',taille = 0.5)

        ## CLASSIC 
        controlledPoints = kaiser_robot.addChild('controlledPoints')
        controlledPoints.addObject('MechanicalObject', name="actuatedPoints", template="Vec3",position=[h_effector, 0, 0])#,rotation=[0, 90 ,0]) # classic
        controlledPoints.addObject('PositionEffector', template="Vec3d", indices='0', effectorGoal="@../../../goal/goalM0.position") # classic
        controlledPoints.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

    elif act_flag == 1:
        rootNode.addObject('GenericConstraintSolver', maxIterations='100', tolerance = '0.0000001')

    if out_flag == 1: # controller simu automatique
        # print("fete à la saucisse j'invite personne")
        cam_pos_tab = data_exp[7]
        rootNode.addObject("InteractiveCamera", name="camera", position=cam_pos_tab)#, zNear=0.1, zFar=500, computeZClip = False,  projectionType=0)
        rootNode.addObject(simu_controller(name="simu_controller",step = step,RootNode=rootNode,data_exp=data_exp))#, Step=step))
        # rootNode.addObject(SimuPrinterCsv(kaiser,rootNode))
        rootNode.addObject(PositionComparator_2d(name="simu_pos_compar", step=step,module = kaiser,RootNode = rootNode,data_exp = data_exp))#, Step=step)) # ne fonctionne pas pour inverse
        # rootNode.addObject(PressureComparator(name="simu_pres_compar", step=step,module = kaiser,RootNode = rootNode))#, Step=step))
        # rootNode.addObject(PositionComparator_3d(name="simu_pos_compar", step=step,module = kaiser,RootNode = rootNode,data_exp=data_exp))#, Step=step)) # ne fonctionne pas pour inverse
        # rootNode.addObject(PressurePrinter(name="printer", step=step,module = kaiser,RootNode = rootNode))#, Step=step)) # ne fonctionne pas pour inverse
        if act_flag == 0 :
            rootNode.addObject(InversePositionController(name="inverse_csv_controller", nb_module = nb_module,nb_cavity = nb_cavity,step=step,RootNode=rootNode,data_exp=data_exp))
        elif act_flag == 1 :
            # rootNode.addObject(CsvPressureController(name="simu_csv_controller", nb_module = nb_module,nb_cavity = nb_cavity,step=step,RootNode=rootNode))#, Step=step))
            rootNode.addObject(CsvPressureController(name="simu_csv_controller",module =kaiser,step=step,RootNode=rootNode,data_exp=data_exp))#, Step=step))
            # rootNode.addObject(ProgressivePressure(name="simu_pos_controller", nb_module = nb_module,nb_cavity = nb_cavity,step=step,RootNode=rootNode))#, Step=step))


    else : # controller runSofa
        if record == 1:
            rootNode.addObject(ParameterPrinterCsv(module =kaiser,nom_dossier = nom_dossier,RootNode=rootNode,K_I = K_I, K_P = K_P))
            rootNode.addObject(PositionPrinterCsv(child_name = 'RigidFrames',name = 'DOFs',module =kaiser,nom_dossier = nom_dossier,beam_flag = 1,RootNode=rootNode))
            rootNode.addObject(PositionPrinterCsv(child_name = 'MeasuredPosition',name = 'MeasuredPositionM0',module =kaiser,nom_dossier = nom_dossier,beam_flag = 0,RootNode=rootNode))
            rootNode.addObject(PositionPrinterCsv(child_name = 'DesiredPosition',name = 'DesiredPositionM0',module =kaiser,nom_dossier = nom_dossier,beam_flag = 0,RootNode=rootNode))
            rootNode.addObject(PositionPrinterCsv(child_name = 'goal',name = 'goalM0',module =kaiser,nom_dossier = nom_dossier,beam_flag = 0,RootNode=rootNode))
            rootNode.addObject(PressurePrinterCsv(module =kaiser,nom_dossier = nom_dossier,RootNode=rootNode,act_flag=act_flag))

        # rootNode.addObject(ArduinoPressure(module = stiff,RootNode = rootNode)) # pour envoyer les pressions calculées par le modèle inverse au robot (hardware) # (mettre après le if suivant !)
        # rootNode.addObject(ArduinoPressure_UCL(module = stiff,RootNode = rootNode)) # pour envoyer les pressions calculées par le modèle inverse au robot (hardware)
        # rootNode.addObject(PressurePrinter_local(module = stiff,RootNode = rootNode)) 

        if setup == 1:
            rootNode.addObject(ArduinoPressure(module =kaiser ,node = rigidFramesNode)) # pour envoyer les pressions calculées par le modèle inverse au robot (hardware) # (mettre après le if suivant !)
            rootNode.addObject(PolhemusTracking(node = MeasuredPosition,name = 'MeasuredPositionM0',offset = [0,0,h_effector]) )

        
        if act_flag == 0 :
            if close_loop == 1 :
                # test = 2
                # rootNode.addObject(GoalKeyboardController(goal_pas = goal_pas,child_name = 'DesiredPosition',name = 'DesiredPositionM0', RootNode = rootNode)) # for goal without shift
                rootNode.addObject(CloseLoopController(name="CloseLoopController",RootNode=rootNode, K_P = K_P, K_I = K_I))

                # rootNode.addObject(PolhemusTracking(node = MeasuredPosition,name = 'MeasuredPositionM0',offset = [0,0,h_effector]) )

                rootNode.addObject(CircleTrajectory(rayon =circle_radius, nb_iter = nb_iter_circle,node = DesiredPosition,name = 'DesiredPositionM0',circle_height = circle_height,module=kaiser))
                # rootNode.addObject(SquareTrajectory(RootNode = rootNode, rayon =square_radius, nb_iter = nb_iter_circle,child_name = 'DesiredPosition',name = 'DesiredPositionM0',square_height = square_height,module=stiff))
                # rootNode.addObject(PrintGoalPos(name="CloseLoopController",RootNode=rootNode))
                # rootNode.addObject(PatternTrajectory(RootNode = rootNode, rayon =square_radius, nb_iter = nb_iter_circle,child_name = 'DesiredPosition',name = 'DesiredPositionM0',square_height = square_height,module=stiff))
            else : # open loop ( close_loop == 0 )
                rootNode.addObject(CircleTrajectory(rayon =circle_radius, nb_iter = nb_iter_circle, node = goal2,name = 'goal2M0',circle_height = circle_height+10,module=kaiser))

                # rootNode.addObject(SquareTrajectory(rayon =square_radius, nb_iter = nb_iter_circle,node = goal2,name = 'goal2M0',square_height = square_height+5,module=stiff))

                # rootNode.addObject(PolhemusTracking(node = MeasuredPosition,name = 'MeasuredPositionM0',offset = [0,0,h_effector]) )

                rootNode.addObject(GoalKeyboardController(goal_pas = goal_pas,node = goal2,name = 'goal2M0')) # for goal with shift
                rootNode.addObject(GoalShift(node_follow= goal ,object_follow = 'goalM0',node_master = goal2,object_master = 'goal2M0',shift_tab = [0,0,5]))

        elif act_flag == 1 :
            rootNode.addObject(StiffController(pas=pressure_step,module = kaiser,parentNode = rigidFramesNode))


    return rootNode

def createScene(rootNode):
    MyScene(rootNode,out_flag = 0,step = 0,YM_soft_part=YM_soft,coef_poi = coef_poisson,data_exp = 0,act_flag = act_flag) # act_flag = 0- IP ; 1- Direct Control


    
    
   

    
   
