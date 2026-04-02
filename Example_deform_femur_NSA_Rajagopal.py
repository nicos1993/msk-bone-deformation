#-------------------------------------------------------------------------#
#    Example script for applying neck-shaft angle (NSA) deformation 
#    to the femur in the Rajagopal2015 model
# ----------------------------------------------------------------------- #

import opensim as osim
import numpy as np
import os
import sys

# Import utilities
import utils.applyValgusVarusToJoints
import utils.applyValgusVarusToVTPGeometry
import utils.applyValgusVarusToMuscleAttachments
import utils.applyValgusVarusToMarkers
import utils.saveDeformedModel
import utils.getJointCentresForBone
import utils.createNeckShaftProfile

#---------------  MAIN SETTINGS ---------------
# Model to deform
modelFileName = './examples_Rajagopal2015/Rajagopal2015.osim'

# where the bone geometries are stored
OpenSim_Geometry_folder = './examples_Rajagopal2015/Geometry'

# body to deform
bone_to_deform = 'femur_r'

# axis of deformation (frontal plane)
valgusAxis = 'x'

# define the rotational profile at the joint centres of the bone of
# interest: NSAProfilePointsDeg = [ proximalNSA distalNSA ];
# Positive = valgus (outward), Negative = varus (inward)
NSAProfilePointsDeg = (15, 0)

# decide if you want to apply NSA to joint as well as other objects.
apply_nsa_to_joints = 'yes'

# where the deformed models will be saved
altered_models_folder = './examples_Rajagopal2015'
#----------------------------------------------

# import model
osimModel = osim.Model(modelFileName)

# compute bone length
Pprox, Pdist, total_L, V = utils.getJointCentresForBone.getJointCentresForBone(osimModel, bone_to_deform)

# define length corresponding to NSA points
LengthProfilePoints = np.vstack((Pprox, Pdist))

# compute NSA profile
nsa_angle_func_rad, nsa_doc_string = utils.createNeckShaftProfile.createNeckShaftProfile(LengthProfilePoints, NSAProfilePointsDeg, valgusAxis)

# suffix used for saving geometries
bone_short = bone_to_deform[0:3] + bone_to_deform[-2:]
deformed_model_suffix = ('_NSA' + bone_short[0].upper() + bone_short[1:] + '_' + nsa_doc_string)

# if you want you can apply NSA to joints
if apply_nsa_to_joints == 'yes':
    osimModel = utils.applyValgusVarusToJoints.applyValgusVarusToJoints(osimModel, bone_to_deform, valgusAxis, nsa_angle_func_rad)

# deforming muscle attachments
osimModel = utils.applyValgusVarusToMuscleAttachments.applyValgusVarusToMuscleAttachments(osimModel, bone_to_deform, valgusAxis, nsa_angle_func_rad)

# if there are markers rotate them
osimModel = utils.applyValgusVarusToMarkers.applyValgusVarusToMarkers(osimModel, bone_to_deform, valgusAxis, nsa_angle_func_rad)

# deform the bone geometries of the generic model
osimModel = utils.applyValgusVarusToVTPGeometry.applyValgusVarusToVTPGeometry(osimModel, bone_to_deform, valgusAxis, 
                                            nsa_angle_func_rad, nsa_doc_string,
                                            OpenSim_Geometry_folder)

# save output model
if not os.path.exists(altered_models_folder):
    os.mkdir(altered_models_folder)

_, name_ext = os.path.split(modelFileName)
name, ext = os.path.splitext(name_ext)
deformed_model_name = name + deformed_model_suffix + ext
output_model_path = os.path.join(altered_models_folder, deformed_model_name)
osimModel.setName(osimModel.getName() + deformed_model_suffix)

# save model
utils.saveDeformedModel.saveDeformedModel(osimModel, output_model_path)