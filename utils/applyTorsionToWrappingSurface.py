import opensim as osim
from .getAxisRotMat import getAxisRotMat
from .orientation2MatRot import orientation2MatRot
from .computeXYZAngleSeq import computeXYZAngleSeq
import numpy as np

def applyTorsionToWrappingSurface(osimModel, aSegmentName, aTorsionAxisString, torsion_angle_func_rad):

    print('------------------------------')
    print(' ADJUSTING WRAPPING SURFACES ')
    print('------------------------------')

    # check if segment is included in the model
    if osimModel.getBodySet().getIndex(aSegmentName)<0:
        raise ValueError('The specified segment is not included in the OpenSim model')
    
    # converting the axis in the index used later
    RotMat, axis_ind = getAxisRotMat(aTorsionAxisString)

    # body of interest
    cur_body = osimModel.getBodySet().get(aSegmentName)

    # wrapping surfaces size
    N_wrap_surfaces = cur_body.getWrapObjectSet().getSize()

    ntm = 1
    processed_wrap_surfaces= []

    # loop through the wrapping surfaces
    for n_wrap_surfaces in range(N_wrap_surfaces):
        
        # current wrapping surfaces
        curr_wrap = cur_body.getWrapObject(cur_body.getWrapObjectSet().get(n_wrap_surfaces).getName())

        # keep track
        processed_wrap_surfaces.append(cur_body.getWrapObjectSet().get(n_wrap_surfaces).getName())
        ntm=ntm+1

        # current wrapping translation
        wrapSurfLocVec3 =  curr_wrap.get_translation()
        wrapSurfLocCoords = np.array([wrapSurfLocVec3.get(0),wrapSurfLocVec3.get(1),wrapSurfLocVec3.get(2)])
        
        # compute torsion metric for the wrap surfaces
        TorsRotMat = RotMat(torsion_angle_func_rad(wrapSurfLocCoords[axis_ind])[0])

        # compute new wrap surfaces coordinates
        new_wrapSurfLocCoords = (TorsRotMat @ wrapSurfLocCoords.T).T

        # setting the wrap surfaces translation as Vec3
        new_wrapSurfLocCoords_v3 = osim.Vec3(new_wrapSurfLocCoords[0], new_wrapSurfLocCoords[1], new_wrapSurfLocCoords[2])
        curr_wrap.set_translation(new_wrapSurfLocCoords_v3)

        # current wrapping rotation
        wrapSurfRotVec3 =  curr_wrap.get_xyz_body_rotation()
        wrapSurfRotCoords = np.array([wrapSurfRotVec3.get(0),wrapSurfRotVec3.get(1),wrapSurfRotVec3.get(2)])

        jointRotMat = orientation2MatRot(wrapSurfRotCoords)
        newJointRotMat =  jointRotMat @ TorsRotMat
        new_Orientation  = computeXYZAngleSeq(newJointRotMat)
        new_wrapSurfRot = osim.Vec3(new_Orientation[0], new_Orientation[1], new_Orientation[2])

        # setting the wrap surfaces rotation as Vec3
        curr_wrap.set_xyz_body_rotation(new_wrapSurfRot)

    print('Processed ' + str(ntm-1) + ' wrapping surfaces:')
    print(processed_wrap_surfaces)

    return osimModel