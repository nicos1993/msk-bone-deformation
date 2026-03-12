# import libraries
import numpy as np
import opensim as osim
import os
import utils.saveDeformedModel
import utils.applyAngulationToJoints

#---------------  WARNNING --------------------
# This script applies angulation to specified joints in the OpenSim model.
# Ensure the joint names are correct!
# Will likely not work if the parent frame is not positioned at the joint centre,
# as the angulation is applied to the parent frame of the joint.
#----------------------------------------------

#---------------  MAIN SETTINGS ---------------
# Model to deform
modelFileName = './models_Miller/model_31d84m.osim'

# joints to apply angulation to
joints_to_angulate = 'hip_r','knee_r','patellofemoral_r','ankle_r'

# axis of angulation
angulationAxis = 'x'

# angulation to be applied (in degrees)
angulationDeg = 20

# where the adjusted models will be saved
altered_models_folder = './models_Miller'
#----------------------------------------------

# import model
osimModel = osim.Model(modelFileName)

angulation_model_suffix = ('_Angulation_' + str(angulationDeg))

osimModel = utils.applyAngulationToJoints(osimModel, joints_to_angulate, angulationAxis, angulationDeg)

# save output model
if not os.path.exists(altered_models_folder):
    os.mkdir(altered_models_folder)

_, name_ext = os.path.split(modelFileName)
name, ext = os.path.splitext(name_ext)
deformed_model_name = name + angulation_model_suffix + ext
output_model_path = os.path.join(altered_models_folder, deformed_model_name)
osimModel.setName(osimModel.getName()+angulation_model_suffix)

# save model
utils.saveDeformedModel(osimModel, output_model_path)