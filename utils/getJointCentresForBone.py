from webbrowser import get
import opensim as osim
import numpy as np
from .getOpenSimVersion import getOpenSimVersion
from .getBodyJoint import getBodyJoint
from .getDistalJointNames import getDistalJointNames
from .computeSpatialTransformTranslations import computeSpatialTransformTranslations

def getJointCentresForBone(osimModel, bone_to_deform):

    # body of interest
    cur_body = osimModel.getBodySet().get(bone_to_deform)

    print('---------------------')
    print(' COMPUTE BONE LENGTH ')
    print('---------------------')
    print('Bone to deform: ' + bone_to_deform)

    # get proximal joint centre 
    # body joint extraction
    
    OpenSimVersion, _ = getOpenSimVersion()

    if OpenSimVersion<4.0:
        # OpenSim 3.3
        proxJoint = cur_body.getJoint()
        # location in child
        prox_loc = cur_body.getJoint().get_location()
    else:
        # OpenSim 4.x
        proxJoint = getBodyJoint(osimModel, bone_to_deform, 1)
        prox_loc = proxJoint.get_frames(1).get_translation()

    # transform in np.array
    prox_P = np.array([prox_loc.get(0), prox_loc.get(1), prox_loc.get(2)])
    # NOTE: no need to check for CustomJoint here, as the child is not affected
    # by the SpatialTransform, which moves the child wrt the parent.

    # get distal joint(s) centre(s)
    # here the body of interest is parent
    jointNameSet = getDistalJointNames(osimModel, bone_to_deform)

    L = []
    dist_P = np.zeros((len(jointNameSet), 3))

    for nj in range(len(jointNameSet)):
    
        # get current joint
        cur_joint_name = jointNameSet[nj]
        distJoint = osimModel.getJointSet().get(cur_joint_name)
    
        # location in parent
        # OpenSim 3.3
        if OpenSimVersion<4.0 :
            dist_loc = distJoint.get_location_in_parent()
        else:
        # OpenSim 4.x
            dist_loc = distJoint.get_frames(0).get_translation()
    
        # offset from the spatial transform (in local body)
        # take into account the spatialTransform
        jointOffset = (0, 0, 0)
        if distJoint.getConcreteClassName() == 'CustomJoint':
            localJointOffset = computeSpatialTransformTranslations(osimModel, distJoint)
            jointOffsetV3 = osim.Vec3(localJointOffset[0], localJointOffset[1], localJointOffset[2])
            jointOffset = (jointOffsetV3.get(0), jointOffsetV3.get(1), jointOffsetV3.get(2))
    
#         # move to body of interest
# #       osimModel.getSimbodyEngine().transformPosition(si, distJoint.getBody(), jointOffsetV3, cur_body, jointOffsetV3)
    
        # sum the contributions
        current_dist_P = np.array([dist_loc.get(0), dist_loc.get(1), dist_loc.get(2)])
        dist_P[nj, :] = current_dist_P + np.array(jointOffset)
        # lengths
        L.append(np.linalg.norm(prox_P - dist_P[nj, :]))

    # compute length
    # in case of multiple joint centre take the further, so all joints are
    # transformed, if needed.
    # Example: tibiofemoral and patellofemoral joints.

    bone_length, max_ind = np.max(L), np.argmax(L)

    # distal point
    dist_P = dist_P[max_ind, :]

    # compute axis versor
    V = (dist_P-prox_P)/bone_length

    # display output
    print(['Proximal joint name  : ', proxJoint.getName()])
    print(f"Proximal joint centre: {prox_P[0]:.2f} {prox_P[1]:.2f} {prox_P[2]:.2f}")
    print(['Distal joint name    : ', jointNameSet[max_ind]])
    print(f"Distal joint centre: {dist_P[0]:.2f} {dist_P[1]:.2f} {dist_P[2]:.2f}")
    print(f"Total length of bone : {bone_length:.2f} m")

    return prox_P, dist_P, bone_length, V