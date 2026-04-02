#-------------------------------------------------------------------------#
#    Copyright (c) 2026 Haralabidis N.                                    #
#    Author:   Nicos Haralabidis,  2026                                   #
#    email:    N.Haralabidis@unsw.edu.au                                  #
# ----------------------------------------------------------------------- #
# Function to apply neck-shaft angle (NSA) deformation to joints

import opensim as osim
import numpy as np
from .getAxisRotMat import getAxisRotMat
from .getBodyJoint import getBodyJoint
from .getDistalJointNames import getDistalJointNames
from .getOpenSimVersion import getOpenSimVersion
from .orientation2MatRot import orientation2MatRot
from .computeSpatialTransformTranslations import computeSpatialTransformTranslations

def applyValgusVarusToJoints(osimModel, bone_to_deform, valgusAxis, nsa_angle_func_rad):
    """
    Applies neck-shaft angle (varus/valgus) deformation to joint orientations.
    
    Parameters:
    -----------
    osimModel : osim.Model
        OpenSim model
    bone_to_deform : str
        Name of body to deform (e.g., 'femur_r')
    valgusAxis : str
        Axis for frontal plane deformation ('x' for medial-lateral)
    nsa_angle_func_rad : function
        Function returning NSA angle (radians) at position along bone
    
    Returns:
    --------
    osimModel : osim.Model
        Model with updated joint orientations
    """
    
    # get rotation matrix function for given axis
    aRotMatFunc, axis_ind = getAxisRotMat(valgusAxis)
    
    print('------------------')
    print(' ADJUSTING JOINTS ')
    print('------------------')
    
    OpenSimVersion, _ = getOpenSimVersion()
    
    # rotate the proximal joint
    if OpenSimVersion < 4.0:
        # OpenSim 3.3
        proxJoint = osimModel.getBodySet.get(bone_to_deform).getJoint()
        
        orientation = osim.Vec3(0)
        location    = osim.Vec3(0)
        
        proxJoint.getOrientation(orientation)
        proxJoint.getLocation(location)
    else:
        # OpenSim 4.x
        proxJoint = getBodyJoint(osimModel, bone_to_deform, 0)
        
        location    = proxJoint.get_frames(1).get_translation()
        orientation = proxJoint.get_frames(1).get_orientation()
    
    print('*', proxJoint.getName(), '('+proxJoint.getConcreteClassName()+')')
    print('', bone_to_deform, 'is CHILD.')
    print('    orientation in child   : ', f"{orientation.get(0):.2f} {orientation.get(1):.2f} {orientation.get(2):.2f}")
    print('    location in child      : ', f"{location.get(0):.2f} {location.get(1):.2f} {location.get(2):.2f}" )
    
    # compute the NSA deformation matrix for proximal joint
    XYZ_location_vec =  np.array([location.get(0), location.get(1), location.get(2)])
    
    ref_loc_prox = XYZ_location_vec[axis_ind]
    
    # apply NSA angle at proximal joint location
    nsa_angle = nsa_angle_func_rad(XYZ_location_vec[axis_ind] - ref_loc_prox)
    nsa_RotMat = aRotMatFunc(nsa_angle[0])
    print('    NSA of ', str(nsa_angle[0]*180/np.pi), ' deg applied.')
    
    # compute new location in child
    new_loc = XYZ_location_vec @ nsa_RotMat.T
    newLocation = osim.Vec3(new_loc[0], new_loc[1], new_loc[2])
    
    # compute new orientation in child
    XYZ_orient_vec = np.array([orientation.get(0), orientation.get(1), orientation.get(2)])
    
    jointRotMat = orientation2MatRot(XYZ_orient_vec)
    
    # apply NSA deformation to joint orientation
    new_jointRotMat = nsa_RotMat @ jointRotMat
    
    # convert back to XYZ angles
    new_XYZ_orient_vec = computeXYZAngleSeq(new_jointRotMat)
    newOrientation = osim.Vec3(new_XYZ_orient_vec[0], new_XYZ_orient_vec[1], new_XYZ_orient_vec[2])
    
    # update proximal joint
    if OpenSimVersion < 4.0:
        # OpenSim 3.3
        proxJoint.setLocation(newLocation)
        proxJoint.setOrientation(newOrientation)
    else:
        # OpenSim 4.x
        proxJoint.get_frames(1).set_translation(newLocation)
        proxJoint.get_frames(1).set_orientation(newOrientation)
    
    return osimModel

def computeXYZAngleSeq(rotMat):
    """
    Computes XYZ angle sequence from rotation matrix.
    """
    # Extract angles from rotation matrix
    x = np.arctan2(rotMat[2, 1], rotMat[2, 2])
    y = np.arcsin(-rotMat[2, 0])
    z = np.arctan2(rotMat[1, 0], rotMat[0, 0])
    
    return np.array([x, y, z])