import opensim as osim

from .getDistalJointNames import getDistalJointNames
from .getAxisRotMat import getAxisRotMat
from .getBodyJoint import getBodyJoint
from .getOpenSimVersion import getOpenSimVersion
from .orientation2MatRot import orientation2MatRot
from .computeXYZAngleSeq import computeXYZAngleSeq
from .computeSpatialTransformTranslations import computeSpatialTransformTranslations
from .orientationRodrigues import orientationRodrigues
import numpy as np

def applyTorsionToJoints(osimModel, bone_to_deform, aStringAxis, torsion_angle_func_rad):

    # get rotation matrix as implicit function for given deformation
    aRotMatFunc, axis_ind = getAxisRotMat(aStringAxis)

    print('------------------')
    print(' ADJUSTING JOINTS ')
    print('------------------')

    OpenSimVersion, _ = getOpenSimVersion()

    # rotate the proximal joint
    # OpenSim 3.3
    if OpenSimVersion<4.0:
        proxJoint = osimModel.getBodySet.get(bone_to_deform).getJoint()
        
        # initialise orientation and location (in body of interest)
        orientation = osim.Vec3(0)
        location    = osim.Vec3(0)
        
        # extract proximal joint params
        proxJoint.getOrientation(orientation)
        proxJoint.getLocation(location)
    else:
        # OpenSim 4.x
        proxJoint = getBodyJoint(osimModel, bone_to_deform, 0)
        
        # extract proximal joint params
        location    = proxJoint.get_frames(1).get_translation()
        orientation = proxJoint.get_frames(1).get_orientation()

    print('*', proxJoint.getName(), '('+proxJoint.getConcreteClassName()+')')
    print('', bone_to_deform, 'is CHILD.')
    print('    orientation in child   : ', f"{orientation.get(0):.2f} {orientation.get(1):.2f} {orientation.get(2):.2f}")
    print('    location in child      : ', f"{location.get(0):.2f} {location.get(1):.2f} {location.get(2):.2f}" )
    
    # compute the torsion matrix for proximal joint
    XYZ_location_vec =  np.array([location.get(0), location.get(1), location.get(2)])
    
    ref_loc_prox = XYZ_location_vec[axis_ind]

    # NOTE: no need to check for CustomJoint here, as the child is not affected
    # by the SpatialTransform, which moves the child wrt the parent.
    tors_angle = torsion_angle_func_rad(XYZ_location_vec[axis_ind] - ref_loc_prox) # subtract the offset if none zero => otherwise some 'torsion' at the proxJoint location which I don't think is intended...
    torsion_RotMat = aRotMatFunc(tors_angle[0])
    print('    torsion of ', str(tors_angle[0]*180/np.pi), ' deg applied.')

    # compute new location in child
    new_loc = XYZ_location_vec @ torsion_RotMat.T
    newLocation = osim.Vec3(new_loc[0], new_loc[1], new_loc[2])

    # compute new orientation in child
    XYZ_orient_vec = np.array([orientation.get(0), orientation.get(1), orientation.get(2)])
    
    jointRotMat = orientation2MatRot(XYZ_orient_vec)
    newJointRotMat =  jointRotMat @ torsion_RotMat
    new_Orientation  = computeXYZAngleSeq(newJointRotMat)
    newOrientation = osim.Vec3(new_Orientation[0], new_Orientation[1], new_Orientation[2])

    R_Rod = orientationRodrigues(jointRotMat, axis_ind, tors_angle) 
    R_final = jointRotMat @ R_Rod
    new_Orientation = computeXYZAngleSeq(R_final)
    newOrientation = osim.Vec3(new_Orientation[0], new_Orientation[1], new_Orientation[2])

    # assign params
    # OpenSim 3.3
    if OpenSimVersion<4.0:
        proxJoint.setOrientation(newOrientation)
        proxJoint.setLocation(newLocation)
    else:
        # OpenSim 4.x
        proxJoint.get_frames(1).set_orientation(newOrientation)
        proxJoint.get_frames(1).set_translation(newLocation)

    # update distal joints
    jointNameSet = getDistalJointNames(osimModel, bone_to_deform)
    
    for nj in range(len(jointNameSet)):

        # initialise
        orientation = osim.Vec3(0)
        location    = osim.Vec3(0)

        # get current joint
        cur_joint_name = jointNameSet[nj]
        curDistJoint = osimModel.getJointSet().get(cur_joint_name)

        # extract joint params
        if OpenSimVersion<4.0:
            curDistJoint.getOrientationInParent(orientation)
            curDistJoint.getLocationInParent(location)
        else:
            orientation = curDistJoint.get_frames(0).get_orientation()
            location    = curDistJoint.get_frames(0).get_translation()

        print('*', curDistJoint.getName(), '('+curDistJoint.getConcreteClassName()+')')
        print('   ', bone_to_deform, ' is PARENT.')
        print('    orientation in parent : ', f"{orientation.get(0):.2f} {orientation.get(1):.2f} {orientation.get(2):.2f}")
        print('    location in parent    : ', f"{location.get(0):.2f} {location.get(1):.2f} {location.get(2):.2f}" )

        # compute the torsion matrix
        XYZ_location_vec =  np.array([location.get(0), location.get(1), location.get(2)])
        
        # take into account the spatialTransform
        jointOffset = np.array([0, 0, 0])
        if curDistJoint.getConcreteClassName() == 'CustomJoint':
            # offset from the spatial transform
            # this is in parent, which is the bone of interest
            jointOffset = computeSpatialTransformTranslations(osimModel, curDistJoint)
            print('    spatialTransf-transl   : ', f"{jointOffset[0]:.2f} {jointOffset[1]:.2f} {jointOffset[2]:.2f}")
            print('    location in parent (initSystem) : ', f"{jointOffset[0]:.2f} {jointOffset[1]:.2f} {jointOffset[2]:.2f}")
        
        # if CustomJoint add the translation from the CustomJoint
        XYZ_location_torsion = XYZ_location_vec+jointOffset

        # actually compute the matrix
        tors_angle = torsion_angle_func_rad(XYZ_location_torsion[axis_ind] - ref_loc_prox) # subtract the offset if none zero => otherwise 'torsion' at the distalJoint location does not match desired 'torsion' bound
        torsion_RotMat = aRotMatFunc(tors_angle[0])
        print('    torsion of ', str(tors_angle[0]*180/np.pi), ' deg applied.')

        # compute new location in parent
        new_Loc =  XYZ_location_vec @ torsion_RotMat.T
        newLocationInParent = osim.Vec3(new_Loc[0], new_Loc[1], new_Loc[2])

        # compute new orientation in parent
        XYZ_orient_vec = np.array([orientation.get(0), orientation.get(1), orientation.get(2)])
        jointRotMat = orientation2MatRot(XYZ_orient_vec)
        
        newJointRotMat =  jointRotMat @ torsion_RotMat
        new_OrientationInPar_old  = computeXYZAngleSeq(newJointRotMat)
        newOrientationInParent_old = osim.Vec3(new_OrientationInPar_old[0], new_OrientationInPar_old[1], new_OrientationInPar_old[2])

        # get the axis of rotation in the local frame of the body
        #body_vert_in_local = jointRotMat[axis_ind,:]
        #tors_angle
        # separate the components of the axis of rotation in the local frame of the body 
        #ux, uy, uz = body_vert_in_local[0], body_vert_in_local[1], body_vert_in_local[2]
        # identity matrix for Rodrigues' formula
        #I = np.eye(3)
        # skew-symmetric matrix for Rodrigues' formula
        #K = np.array([
        #    [0, -uz, uy],
        #    [uz, 0, -ux],
        #    [-uy, ux, 0]
        #    ])
        # compute the rotation matrix using Rodrigues' formula
        #R_rod = I + np.sin(tors_angle) * K + (1 - np.cos(tors_angle)) * (K @ K)
        
        R_Rod = orientationRodrigues(jointRotMat, axis_ind, tors_angle)
        
        R_final = jointRotMat @ R_Rod
        new_OrientationInPar = computeXYZAngleSeq(R_final)
        newOrientationInParent = osim.Vec3(new_OrientationInPar[0], new_OrientationInPar[1], new_OrientationInPar[2])


        # assign new parameters
        if OpenSimVersion<4.0:
            curDistJoint.setOrientationInParent(newOrientationInParent)
            curDistJoint.setLocationInParent(newLocationInParent)
        else:
            curDistJoint.get_frames(0).set_orientation(newOrientationInParent)
            curDistJoint.get_frames(0).set_translation(newLocationInParent)

    return osimModel