import Sofa.Core

# utilise Rigidify de STLIB
from stlib3.physics.mixedmaterial import Rigidify
from os import getcwd

#FRENCH :
# permet d'initialiser un robot au nb de module voulu :
# - mettre tous les paramètres dans l'initialisation de la variable
# - appeler ensuite seulemnt createRobot qui appelle les autres fonctions, tous les paramètres étant dans l'initialisation (voir scene.py)

#ENGLISH :
# allows to initialize a robot with the desired number of modules:
# - put all parameters in variable initialization
# - then call only createRobot which calls the other functions, all the parameters being in the initialization (see scene.py)

def EffectorGoal(node, position,name,taille):
    """
    Function to create goal point
        INPUT : 
        node = parent node of the goal point
        position = initial position of the goal point (change the name of the variable to make it more clear (for all the argument of this function) ? -> "initial_position" / "meca_object_name" / "goal_size")
        name = name of the mechanical object
        taille = size of the goal point (radius in mm)
    """
    goal = node.addChild(name)
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=100, threshold=1e-12, tolerance=1e-10)
    goal.addObject('MechanicalObject', name=name + 'M0', position=position)
    goal.addObject('SphereCollisionModel', radius=taille)
    # goal.addObject('RestShapeSpringsForceField', points=0, angularStiffness=1e5, stiffness=1e5)
    goal.addObject('UncoupledConstraintCorrection')
    return goal

def define_mesh_path(mesh_name,out_flag):
    """
    Give the absolute path to get the mesh files.

    Absolute because it's better if we launch the scene from another folder than the main one.
    (For example I use the variable out_flag == 1 to say that I'm launching my scene in a loop for optimisation purpose)
    """
    path = getcwd()
    if out_flag == 0:
        inter = '/mesh/'
    else :
        inter = '/ucl_collaboration/mesh/'

    file_path = path + inter +mesh_name
    print("###########################")
    print(str(file_path))
    return file_path


class KaiserRobot() :  

    def __init__(self,h_module,init_pressure_value,value_type,YM_soft_part,YM_stiff_part,coef_poi,nb_cavity,chamber_model, nb_module,module_model,max_pression,name_cavity,masse_module,nb_poutre,rigid_base,rigid_top,rigid_bool,min_pression,dynamic,dt):
        self.h_module = h_module
        self.init_pressure_value = init_pressure_value
        self.value_type = value_type
        self.YM_soft_part = YM_soft_part
        self.YM_stiff_part = YM_stiff_part
        self.coef_poi = coef_poi
        self.nb_cavity = nb_cavity
        self.i_cavity = 0
        self.ang_dec = 360/nb_cavity # calcul du placement des cavités
        self.chamber_model = chamber_model
        self.nb_module = nb_module
        self.module_model = module_model
        self.max_pression = max_pression
        self.name_cavity = name_cavity
        self.masse_module = masse_module
        self.nb_poutre = nb_poutre
        self.rigid_base = rigid_base
        self.rigid_top = rigid_top
        self.rigid_bool = rigid_bool
        self.min_pression = min_pression
        self.dyn_flag = dynamic
        self.dt = dt
        self.angle_a = 50 # rotation en degré pour aligner le modèle et le hardware

    def createCavity(self,parent,name_c,i,cavity_model,act_flag): # for v1 -------
        """
        Import the cavity surface model from .stl file
        """
        dec_centre = 0 # décalage au centre des cavités pneumatiques en CAO
        base_offset = 3 # offset bewteen the floor (z = 0) and the beginning of the cavity (z = 3 here) // offset entre les cavités et le sol en CAO
        bellowNode = parent.addChild(name_c+str(i+1))
        bellowNode.addObject('MeshSTLLoader', filename=cavity_model, flipNormals='0', triangulate='true', name='meshLoader',rotation=[0,0,self.ang_dec*self.i_cavity+self.angle_a], translation=[dec_centre, 0,base_offset+self.h_module*i])#, rotation=[self.ang_dec*self.i_cavity,0,0] if pre-rotated 3D model
        bellowNode.addObject('MeshTopology', src='@meshLoader', name='chambreAMesh'+str(i+1))
        bellowNode.addObject('MechanicalObject', name='chambreA'+str(i+1),rotation=[0, 90 , 0])#,translation = [0,0,h_module*i])
        bellowNode.addObject('TriangleCollisionModel', moving='0', simulated='1')
        if act_flag == 0 :
            bellowNode.addObject('SurfacePressureActuator', name='SPC', template = 'Vec3d',triangles='@chambreAMesh'+str(i+1)+'.triangles',minPressure = self.min_pression,maxPressure = self.max_pression)#,maxPressureVariation = 20)#,valueType=self.value_type)
        elif  act_flag == 1 :
            bellowNode.addObject('SurfacePressureConstraint', name='SPC', triangles='@chambreAMesh'+str(i+1)+'.triangles', value=self.init_pressure_value,minPressure = self.min_pression,maxPressure = self.max_pression, valueType=self.value_type)#,maxPressureVariation = 20)#,
        # bellowNode.addObject('SurfacePressureModel', name='SPC_model', minPressure = 0,maxPressure = 20)
        # bellowNode.addObject('AdaptiveBeamMapping', interpolation='@../BeamInterpolation', input='@../DOFs', output='@./chambreA'+str(i+1) )
        self.i_cavity = self.i_cavity + 1;
        if self.i_cavity == self.nb_cavity :
            self.i_cavity = 0
        return bellowNode

    def createModule(self,parent,name_c,i,model):
        """
         Import the volumic FEM model from the .vtk file
        """
        module = parent.addChild('stiff_flop'+str(i+1))
        module.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.1, rayleighMass=0.1)
        module.addObject('SparseLDLSolver', name='directSolver' , template="CompressedRowSparseMatrixd")
        # module.addObject('MeshSTLLoader', filename='mesh/solid_STIFF_FLOP02_attach_s.stl', flipNormals='0', triangulate='true', name='loader' )#, translation=[4., 0,-3.275], rotation=[0,0,90])
        module.addObject('MeshVTKLoader', name='loader', filename=model,translation = [0,0,self.h_module*i], rotation=[0, 0 , self.angle_a]) # 
        module.addObject('MeshTopology', src='@loader', name='container')
        module.addObject('MechanicalObject', name='tetras', template='Vec3', showObject=True, showObjectScale=1,rotation=[0, 90 , 0])#,translation = [0,0,h_module*i])
        module.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=self.coef_poi,  youngModulus=self.YM_soft_part) # stable youngModulus = 500 / réel ? = 103
        module.addObject('UniformMass', totalMass=self.masse_module)
        return module

    def createRobot(self,parent,name,out_flag,act_flag):
        """
        Call createModule() and createCavity() to import .vtk and .stl file to create the robot.

        Add also the Mapping between the cavity, the FEM model and the beam ( MechanicalMatrixMapper and AdaptiveBeamMapping)

        Add also the rigidification of subpart of the module (if rigid_bool == 1)
        """
        for i in range(self.nb_module):

            parent.addObject('MechanicalMatrixMapper',template='Rigid3d,Rigid3d', object1='@DOFs', object2='@DOFs', nodeToParse='@./stiff_flop'+str(i+1))  # je ne projete que dans la structure globale => mechanical object
            
            module_model_path = define_mesh_path(self.module_model,out_flag)
            chamber_model_path = define_mesh_path(self.chamber_model,out_flag)

            name = 'module'
            module = self.createModule(parent,name,i,module_model_path)
            module.addObject('AdaptiveBeamMapping', interpolation='@../BeamInterpolation', input='@../DOFs', output='@./tetras' )

           ### choisir strict a False ou True
            module.addObject('BoxROI', name='boxROI_base'+str(i+1), box=[self.h_module*i-1, -8, -8, self.h_module*i+self.rigid_base + 1, 8, 8], drawBoxes=False, strict=False,drawTetrahedra = False) # si autom complète, mettre 8 dépendant des dimensions du robot
            module.addObject('BoxROI', name='boxROI_top'+str(i+1), box=[self.h_module*(i+1)-self.rigid_top, -8, -8, self.h_module*(i+1)+ 1, 8, 8 ], drawBoxes=False, strict=False,drawTetrahedra = False) # utilisé initialement pour rendre rigide une des parois du robot => ici le sommet du module

            if self.rigid_bool == 1 :
                print("-- Application de la rigidification des extrémités des modules --")
                modelSubTopo = module.addChild('modelSubTopo')
                modelSubTopo.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra='@boxROI_base'+str(i+1)+'.tetrahedraInROI', name='container')
                modelSubTopo.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra='@boxROI_top'+str(i+1)+'.tetrahedraInROI', name='container')
                modelSubTopo.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=self.coef_poi,  youngModulus=self.YM_stiff_part)

            for j in range(self.nb_cavity):
                name = 'Bellow' + str(j+1)
                bellowNode1 = self.createCavity(parent,name,i,chamber_model_path,act_flag)
                bellowNode1.addObject('AdaptiveBeamMapping', interpolation='@../BeamInterpolation', input='@../DOFs', output='@./chambreA'+str(i+1) )

        return module

class Kaiserlautern_FEM() :   # For the full FEM version, not finished and used yet (maybe for later)

    def __init__(self,h_module,init_pressure_value,value_type,YM_soft_part,YM_stiff_part,coef_poi,nb_cavity,chamber_model, nb_module,module_model,max_pression,name_cavity,masse_module,nb_poutre,rigid_base,rigid_top,rigid_bool,min_pression):
        self.h_module = h_module
        self.init_pressure_value = init_pressure_value
        self.value_type = value_type
        self.YM_soft_part = YM_soft_part
        self.YM_stiff_part = YM_stiff_part
        self.coef_poi = coef_poi
        self.nb_cavity = nb_cavity
        self.i_cavity = 0
        self.ang_dec = 360/nb_cavity # calcul du placement des cavités
        self.chamber_model = chamber_model
        self.nb_module = nb_module
        self.module_model = module_model
        self.max_pression = max_pression
        self.name_cavity = name_cavity
        self.masse_module = masse_module
        self.nb_poutre = nb_poutre
        self.rigid_base = rigid_base
        self.rigid_top = rigid_top
        self.rigid_bool = rigid_bool
        self.min_pression = min_pression

    def createCavity(self,parent,name_c,i,cavity_model,act_flag): 
        """
        Import the cavity surface model from .stl file
        """
        dec_centre = 0 # décalage au centre des cavités pneumatiques en CAO
        base_offset = 3 # offset entre les cavités et le sol en CAO
        bellowNode = parent.addChild(name_c+str(i+1))
        bellowNode.addObject('MeshSTLLoader', filename=cavity_model, flipNormals='0', triangulate='true', name='meshLoader',rotation=[0,0,self.ang_dec*self.i_cavity], translation=[dec_centre, 0,base_offset+self.h_module*i])#, rotation=[self.ang_dec*self.i_cavity,0,0] if pre-rotated 3D model
        bellowNode.addObject('MeshTopology', src='@meshLoader', name='chambreAMesh'+str(i+1))
        bellowNode.addObject('MechanicalObject', name='chambreA'+str(i+1),rotation=[0, 0 , 0]) #,rotation=[0, 90 , 0])#,translation = [0,0,h_module*i])
        bellowNode.addObject('TriangleCollisionModel', moving='0', simulated='1')
        if act_flag == 0 :
            bellowNode.addObject('SurfacePressureActuator', name='SPC', template = 'Vec3d',triangles='@chambreAMesh'+str(i+1)+'.triangles',minPressure = self.min_pression,maxPressure = self.max_pression)#,maxPressureVariation = 20)#,valueType=self.value_type)
        elif  act_flag == 1 :
            bellowNode.addObject('SurfacePressureConstraint', name='SPC', triangles='@chambreAMesh'+str(i+1)+'.triangles', value=self.init_pressure_value,minPressure = self.min_pression,maxPressure = self.max_pression, valueType=self.value_type)#,maxPressureVariation = 20)#,
        # bellowNode.addObject('SurfacePressureModel', name='SPC_model', minPressure = 0,maxPressure = 20)
        # bellowNode.addObject('AdaptiveBeamMapping', interpolation='@../BeamInterpolation', input='@../DOFs', output='@./chambreA'+str(i+1) )
        self.i_cavity = self.i_cavity + 1;
        if self.i_cavity == self.nb_cavity :
            self.i_cavity = 0
        return bellowNode

    def createModule(self,parent,name_c,i,model): 
        """
        Import the FEM model from the .vtk file
        """
        module = parent.addChild('kaiser_robot'+str(i+1))
        # module.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.1, rayleighMass=0.1)
        # module.addObject('SparseLDLSolver', name='directSolver' , template="CompressedRowSparseMatrixd")
        # module.addObject('MeshSTLLoader', filename='mesh/solid_STIFF_FLOP02_attach_s.stl', flipNormals='0', triangulate='true', name='loader' )#, translation=[4., 0,-3.275], rotation=[0,0,90])
        module.addObject('MeshVTKLoader', name='loader', filename=model,translation = [0,0,self.h_module*i], rotation=[0, 0 , 0]) # 
        module.addObject('MeshTopology', src='@loader', name='container')
        module.addObject('MechanicalObject', name='tetras', template='Vec3', showObject=True, showObjectScale=1,rotation=[0, 0 , 0])#rotation=[0, 90 , 0])#,translation = [0,0,h_module*i])
        module.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=self.coef_poi,  youngModulus=self.YM_soft_part) # stable youngModulus = 500 / réel ? = 103
        module.addObject('UniformMass', totalMass=self.masse_module)
        # module.addObject('RestShapeSpringsForceField', points='0', stiffness=1e12, angularStiffness=1e12)

        return module

    def createRobot(self,parent,name,out_flag,act_flag):
        """
        Call createModule() and createCavity() to import .vtk and .stl file to create the robot.

        Add also the rigidification of subpart of the module (if rigid_bool == 1)
        """
        for i in range(self.nb_module):
            
            module_model_path = define_mesh_path(self.module_model,out_flag)
            chamber_model_path = define_mesh_path(self.chamber_model,out_flag)

            name = 'module'
            module = self.createModule(parent,name,i,module_model_path)
            # module.addObject('AdaptiveBeamMapping', interpolation='@../BeamInterpolation', input='@../DOFs', output='@./tetras' )

           ### choisir strict a False ou True
            module.addObject('BoxROI', name='boxROI_base'+str(i+1), box=[self.h_module*i-1, -8, -8, self.h_module*i+self.rigid_base + 1, 8, 8], drawBoxes=False, strict=False,drawTetrahedra = False) # si autom complète, mettre 8 dépendant des dimensions du robot
            module.addObject('BoxROI', name='boxROI_top'+str(i+1), box=[self.h_module*(i+1)-self.rigid_top, -8, -8, self.h_module*(i+1)+ 1, 8, 8 ], drawBoxes=False, strict=False,drawTetrahedra = False) # utilisé initialement pour rendre rigide une des parois du robot => ici le sommet du module

            if self.rigid_bool == 1 :
                print("-- Application de la rigidification des extrémités des modules --")
                modelSubTopo = module.addChild('modelSubTopo')
                modelSubTopo.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra='@boxROI_base'+str(i+1)+'.tetrahedraInROI', name='container')
                modelSubTopo.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra='@boxROI_top'+str(i+1)+'.tetrahedraInROI', name='container')
                modelSubTopo.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=self.coef_poi,  youngModulus=self.YM_stiff_part)

            for j in range(self.nb_cavity):
                name = 'Bellow' + str(j+1)
                # parent.addObject('MechanicalMatrixMapper',template='Rigid3d,Rigid3d', object1='@tetras', object2='@tetras', nodeToParse='@./'+name+str(i+1))  # je ne projete que dans la structure globale => mechanical object
                bellowNode1 = self.createCavity(module,name,i,chamber_model_path,act_flag)
                # bellowNode1.addObject('BarycentricMapping', mapForces=False, mapMasses=False)
                # bellowNode1.addObject('IdentityMapping', mapForces=False, mapMasses=False)


        return module