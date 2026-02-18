#-------------------------------------------------------------------------%
#    Copyright (c) 2021 Modenese L.                                       %
#    Author:   Luca Modenese,  2021                                       %
#    email:    l.modenese@imperial.ac.uk                                  %
# ----------------------------------------------------------------------- %
#
# FUNCTION FROM msk-STAPLE toolbox.
#
#-------------------------------------------------------------------------%
import numpy as np

def orientation2MatRot(XYZ_orientation):
    # Transforms Euler XYZ body-fixed rotation angles used to express the orientation
    # in OpenSim model in their rotation matrix

    # compute all parts
    c = np.cos(XYZ_orientation)
    s = np.sin(XYZ_orientation)
    # assign to elements of the matrix
    c1,c2,c3 = c[0],c[1],c[2]
    s1,s2,s3 = s[0],s[1],s[2]
    # matrix for XYZ fixed-body rotation (see
    # https://en.wikipedia.org/wiki/Euler_angles)
    RotMat = np.array([[c2*c3,               -c2*s3,          s2],
                 [c1*s3+c3*s1*s2,   c1*c3-s1*s2*s3,   -c2*s1],
                 [s1*s3-c1*c3*s2,   c3*s1+c1*s2*s3,    c1*c2]])

    return RotMat