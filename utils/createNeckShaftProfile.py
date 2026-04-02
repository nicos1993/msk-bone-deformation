#-------------------------------------------------------------------------#
#    Copyright (c) 2026 Haralabidis N.                                    #
#    Author:   Nicos Haralabidis,  2026                                   #
#    email:    N.Haralabidis@unsw.edu.au                                  #
# ----------------------------------------------------------------------- #
# Function to create a neck-shaft angle (NSA) profile for femoral 
# deformation (varus/valgus bending in the frontal plane)

import numpy as np
from .getAxisRotMat import getAxisRotMat

def nsaProfile(L, LengthProfilePoints, nsa_points):
    """
    Computes the neck-shaft angle at a given position along the bone.
    
    Parameters:
    -----------
    L : float
        Position along bone axis
    LengthProfilePoints : array
        Proximal and distal positions along bone
    nsa_points : tuple
        NSA angles at proximal and distal points (in degrees)
    
    Returns:
    --------
    float : NSA angle in radians at position L
    """
    
    nsa_points_rad = np.array(nsa_points) / (180 / np.pi)
    
    nsa_grad = np.diff(nsa_points_rad) / np.diff(LengthProfilePoints)
    
    nsa_angle = nsa_points_rad[0] + nsa_grad * L
    return nsa_angle

def createNeckShaftProfile(LengthProfilePoints, NSAProfilePointsDeg, valgusAxis='x'):
    """
    Creates a continuous NSA profile function along the femoral length.
    
    Neck-shaft angle (NSA) is the angle between the femoral neck and shaft
    measured in the frontal plane. Positive values indicate valgus 
    (outward bending), negative values indicate varus (inward bending).
    
    Parameters:
    -----------
    LengthProfilePoints : ndarray, shape (2, 3)
        Joint center coordinates [proximal; distal]
    NSAProfilePointsDeg : tuple
        (proximalNSA, distalNSA) in degrees
    valgusAxis : str
        Axis for frontal plane deformation, typically 'x'
    
    Returns:
    --------
    nsa_angle_func_rad : function
        Function that returns NSA angle (radians) at any position along bone
    nsa_doc_string : str
        String for naming deformed files
    """
    
    print('--------------------------')
    print(' CREATING NSA PROFILE ')
    print('--------------------------')
    
    # get axis indices
    _, axis_ind = getAxisRotMat(valgusAxis)
    
    print('Axis of valgus/varus deformation: ', valgusAxis.upper())
    print('Profile (NSA Point, Coordinate)')
    
    # pointProfile on axis of interest
    axis_LengthProfilePoints = LengthProfilePoints[:, axis_ind]
    
    for nup in range(len(NSAProfilePointsDeg)):
        print(f"{NSAProfilePointsDeg[nup]:+.1f} deg     |------> {axis_LengthProfilePoints[nup]:.1f}")
    
    # create implicit function for calculating NSA at a certain quote
    nsa_angle_func_rad = lambda L: nsaProfile(L, axis_LengthProfilePoints, NSAProfilePointsDeg)
    
    # round degrees of NSA at joints
    nsa_bounds_deg = np.round(np.array([nsa_angle_func_rad(axis_LengthProfilePoints[0]), 
                                        nsa_angle_func_rad(axis_LengthProfilePoints[1])])*(180/np.pi))
    
    # strings to use for naming models
    nsa_doc_string = f"Prox{str(int(nsa_bounds_deg[0][0]))}Dist{str(int(nsa_bounds_deg[1][0]))}Deg"
    
    return nsa_angle_func_rad, nsa_doc_string