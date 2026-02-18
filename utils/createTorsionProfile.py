import numpy as np
from .getAxisRotMat import getAxisRotMat

def torsionProfile(L, LengthProfilePoints, torsion_points):

    torsion_points_rad = np.array(torsion_points) / (180 / np.pi)

    tors_grad = np.diff(torsion_points_rad) / np.diff(LengthProfilePoints)

    torsion_angle = torsion_points_rad[0] + tors_grad * L
    return torsion_angle

def createTorsionProfile(LengthProfilePoints, TorsionProfilePointsDeg, torsionAxis):

    print('--------------------------')
    print(' CREATING TORSION PROFILE ')
    print('--------------------------')

    # get axis indices
    _, axis_ind = getAxisRotMat(torsionAxis)

    print('Axis of torsion: ', torsionAxis.upper())
    print('Profile (Tors Point, Coordinate)')

    # pointProfile on axis of interest
    axis_LengthProfilePoints = LengthProfilePoints[:, axis_ind]

    for nup in range(len(TorsionProfilePointsDeg)):
        print(f"{TorsionProfilePointsDeg[nup]}   deg     |------> {axis_LengthProfilePoints[nup]}")

    # create implicit function for calculating torsion at a certain quote
    torsion_angle_func_rad = lambda L: torsionProfile(L, axis_LengthProfilePoints, TorsionProfilePointsDeg)

    # round degrees of torsion at joints
    torsion_bounds_deg = np.round(np.array([torsion_angle_func_rad(axis_LengthProfilePoints[0]), torsion_angle_func_rad(axis_LengthProfilePoints[1])])*(180/np.pi))

    # strings to use for naming models
    torsion_doc_string = f"Prox{str(int(torsion_bounds_deg[0][0]))}Dist{str(int(torsion_bounds_deg[1][0]))}Deg"

    return torsion_angle_func_rad, torsion_doc_string