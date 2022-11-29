 # coding=utf-8

import Sofa
# import SofaPython3
from math import sin,cos, sqrt, acos
import array

from Polhemus_SOFA_Controller import *

def createScene(rootNode):

    rootNode.addObject('VisualStyle', displayFlags="showCollision" )

    MeasuredPosition = rootNode.addChild('MeasuredPosition')
    MeasuredPosition.addObject('EulerImplicitSolver', firstOrder=True)
    MeasuredPosition.addObject('CGLinearSolver', iterations='1000',threshold="1e-5", tolerance="1e-5")
    MeasuredPosition.addObject('MechanicalObject', name='MeasuredPositionM0', position=[0, 0,0])
    MeasuredPosition.addObject('SphereCollisionModel', radius='2')#, group='1')
    MeasuredPosition.addObject('UncoupledConstraintCorrection')
                
    rootNode.addObject(PolhemusTracking(node = MeasuredPosition,name = 'MeasuredPositionM0', offset=[10,10,0]))

    return rootNode
