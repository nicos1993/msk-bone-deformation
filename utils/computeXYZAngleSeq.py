# COMPUTEXYZANGLESEQ Convert a rotation matrix in the orientation vector
# used in OpenSim (X-Y-Z axes rotation order).
#
#   orientation = computeXYZAngleSeq(aRotMat)
#
# Inputs:
#   aRotMat - a rotation matrix, normally obtained writing as columns the
#       axes of the body reference system, expressed in global reference
#       system.
#
# Outputs:
#   orientation - the sequence of angles used in OpenSim to define the
#       joint orientation. Sequence of rotation is X-Y-Z.
#
#-------------------------------------------------------------------------#
#  Author:   Luca Modenese, 2021
#  Copyright 2021 Luca Modenese
#-------------------------------------------------------------------------#
#
# FUNCTION FROM msk-STAPLE toolbox.
#
#-------------------------------------------------------------------------#
import numpy as np

def computeXYZAngleSeq(aRotMat):

    # fixed body sequence of angles from rot mat usable for orientation in
    # OpenSim
    beta  = np.atan2(aRotMat[0,2],                   np.sqrt(aRotMat[0,0]**2.0+aRotMat[0,1]**2.0))
    alpha = np.atan2(-aRotMat[1,2]/np.cos(beta),        aRotMat[2,2]/np.cos(beta))
    gamma = np.atan2(-aRotMat[0, 1]/np.cos(beta),       aRotMat[0,0]/np.cos(beta))

    orientation = np.array([alpha, beta, gamma])

    return orientation