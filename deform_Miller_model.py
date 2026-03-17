# import libraries
import numpy as np
import opensim as osim
import os
import utils.getJointCentresForBone
import utils.createTorsionProfile
import utils.applyTorsionToJoints
import utils.applyTorsionToVTPBoneGeom
import utils.applyTorsionToMuscleAttachments
import utils.applyTorsionToMarkers
import utils.saveDeformedModel

#--------------- Model permutations ---------------

# default tibia torsion:
def_tt = 29

# default femur anteversion:
def_fa = -14

# define offsets to give desired torsion/anteversion values
offset_tt = np.array([-17, -9, 11, 24])
offset_fa = np.array([13, -14, -24, -34])

#---------------  MAIN SETTINGS ---------------
# Model to deform
modelFileName = './models_Miller/model_31d84m.osim'

# where the bone geometries are stored
OpenSim_Geometry_folder = './models_Miller/Geometry'

# body to deform
bone_to_deform = 'femur_r'

# axis of deformation
torsionAxis = 'y'

# check if deforming tibia or femur to assign the correct offsets to create models
if bone_to_deform == 'tibia_r':
    offsets = offset_tt
    reference = def_tt
    apply_torsion_to_joints = 'yes'

elif bone_to_deform == 'femur_r':
    offsets = offset_fa
    reference = def_fa
    apply_torsion_to_joints = 'no'

for k in range(offsets.size):

    # desired deformation to apply
    offset = offsets[k]

    # define the rotational profile at the joint centres of the bone of
    # interest: TorsionProfilePointsDeg = [ proximalTorsion DistalTorsion ];
    # check if it is tibia/femur
    if bone_to_deform == 'tibia_r':
        TorsionProfilePointsDeg = (0,  - offset)
    elif bone_to_deform == 'femur_r':
        TorsionProfilePointsDeg = (offset, 0)

    # decide if you want to apply torsion to joint as well as other objects.
    # E.g. choose no for investigating the effect of femoral anteversion in a
    # leg with straight alignment.
    # Choose yes for modelling a CP child with deformation of bone resulting in
    # joint rotation, meaning the kinematic model is altered.
    # apply_torsion_to_joints = 'yes'

    # where the deformed models will be saved
    altered_models_folder = './models_Miller'
    #----------------------------------------------

    # import model
    osimModel = osim.Model(modelFileName)

    # compute bone length
    Pprox, Pdist, total_L, V = utils.getJointCentresForBone(osimModel, bone_to_deform)

    # define length corresponding to torsion points
    LengthProfilePoints = np.vstack((Pprox,Pdist))

    # compute torsion profile
    torsion_angle_func_rad, torsion_doc_string = utils.createTorsionProfile(LengthProfilePoints, TorsionProfilePointsDeg, torsionAxis)

    # suffix used for saving geometries
    bone_short = bone_to_deform[0:3]+bone_to_deform[-2:]
    deformed_model_suffix = ('_Tors'+bone_short[0].upper()+bone_short[1:]+'_'+torsion_doc_string)

    # if you want you can apply torsion to joints
    if apply_torsion_to_joints == 'yes':
        osimModel = utils.applyTorsionToJoints(osimModel, bone_to_deform, torsionAxis, torsion_angle_func_rad)

    # deforming muscle attachments
    osimModel = utils.applyTorsionToMuscleAttachments(osimModel, bone_to_deform, torsionAxis, torsion_angle_func_rad)

    # if there are markers rotate them
    osimModel = utils.applyTorsionToMarkers(osimModel, bone_to_deform, torsionAxis, torsion_angle_func_rad)

    # deform the bone geometries of the generic model
    osimModel = utils.applyTorsionToVTPBoneGeom(osimModel, bone_to_deform, torsionAxis, 
                                                torsion_angle_func_rad, torsion_doc_string,
                                                OpenSim_Geometry_folder)

    # save output model
    if not os.path.exists(altered_models_folder):
        os.mkdir(altered_models_folder)

    _, name_ext = os.path.split(modelFileName)
    name, ext = os.path.splitext(name_ext)
    deformed_model_name = name + deformed_model_suffix + ext
    output_model_path = os.path.join(altered_models_folder, deformed_model_name)
    osimModel.setName(osimModel.getName()+deformed_model_suffix)

    # save model
    utils.saveDeformedModel(osimModel, output_model_path)