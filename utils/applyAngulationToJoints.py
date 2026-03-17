import opensim as osim
import numpy as np

from .orientationRodrigues import orientationRodrigues
from .getOpenSimVersion import getOpenSimVersion
from .getAxisRotMat import getAxisRotMat
from .orientation2MatRot import orientation2MatRot
from .computeXYZAngleSeq import computeXYZAngleSeq


def applyAngulationToJoints(osimModel, joints_to_angulate, angulationAxis, angulationDeg):

    # get OpenSim version
    OpenSimVersion, _ = getOpenSimVersion()

    # get rotation matrix as implicit function for given deformation
    aRotMatFunc, axis_ind = getAxisRotMat(angulationAxis)

    print('--------------------------------')
    print(' ADJUSTING ANGULATION OF JOINTS ')
    print('--------------------------------')

    for joint_name in joints_to_angulate:

        if joint_name in ['hip_r','ankle_r']:
            angulationDeg_apply = - (angulationDeg / 2) # for hip and ankle, we apply half the angulation to each of the two joints applied in opposite directions
        elif joint_name in ['hip_l','ankle_l']:
            angulationDeg_apply = (angulationDeg / 2)
        elif joint_name in ['knee_r','patellofemoral_r']:
            angulationDeg_apply = angulationDeg
        elif joint_name in ['knee_l','patellofemoral_l']:
            angulationDeg_apply = -angulationDeg
        
        joint = osimModel.getJointSet().get(joint_name)

        # get joint
        if OpenSimVersion<4.0:
            
            # initialise orientation (in body of interest)
            orientation = osim.Vec3(0)

            # extract proximal joint params
            joint.getOrientation(orientation)

        else:
            # OpenSim 4.x
            orientation = joint.get_frames(0).get_orientation()

        print('*', joint.getName(), '('+joint.getConcreteClassName()+')')
        print('    orientation in parent   : ', f"{orientation.get(0):.2f} {orientation.get(1):.2f} {orientation.get(2):.2f}")

        torsion_RotMat = aRotMatFunc(np.deg2rad(angulationDeg_apply))
        print('    angulation of ', str(angulationDeg_apply), ' deg applied.')

        # compute new orientation in child
        XYZ_orient_vec = np.array([orientation.get(0), orientation.get(1), orientation.get(2)])
        
        jointRotMat = orientation2MatRot(XYZ_orient_vec)
        newJointRotMat =  jointRotMat @ torsion_RotMat
        new_Orientation_temp  = computeXYZAngleSeq(newJointRotMat)
        newOrientation_temp = osim.Vec3(new_Orientation_temp[0], new_Orientation_temp[1], new_Orientation_temp[2])

        R_Rod = orientationRodrigues(jointRotMat, axis_ind, np.deg2rad(angulationDeg_apply)) 
        R_final = jointRotMat @ R_Rod
        new_Orientation = computeXYZAngleSeq(R_final)
        newOrientation = osim.Vec3(new_Orientation[0], new_Orientation[1], new_Orientation[2])

        if OpenSimVersion<4.0:
            joint.setOrientation(newOrientation)
        else:
            joint.get_frames(0).set_orientation(newOrientation)

    return osimModel