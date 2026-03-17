#-------------------------------------------------------------------------%
#    Copyright (c) 2026 Haralabidis N.                                    %
#    Author:   Nicos Haralabidis,  2026                                   %
#    email:    N.Haralabidis@unsw.edu.au                                  %
#-------------------------------------------------------------------------%
import numpy as np

def orientationRodrigues(jointRotMat, axis_ind, tors_angle):
    # 

    # jointRotMat describes the orientation of a frame relative to its body frame

    # get the axis of rotation in the local frame of jointRotMat
    rot_axis_local = jointRotMat[axis_ind,:] 

    # separate the components
    ux, uy, uz = rot_axis_local[0], rot_axis_local[1], rot_axis_local[2]
    
    # create identity matrix for Rodrigues' formula
    I = np.eye(3)
    # skew-symmetric matrix for Rodrigues' formula from axis components
    K = np.array([
        [0, -uz, uy],
        [uz, 0, -ux],
        [-uy, ux, 0]
        ])
    
    # compute the rotation matrix using Rodrigues' formula
    R_rod = I + np.sin(tors_angle) * K + (1 - np.cos(tors_angle)) * (K @ K)

    return R_rod