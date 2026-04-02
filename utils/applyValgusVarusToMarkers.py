#-------------------------------------------------------------------------#
#    Copyright (c) 2026 Haralabidis N.                                    #
#    Author:   Nicos Haralabidis,  2026                                   #
#    email:    N.Haralabidis@unsw.edu.au                                  #
# ----------------------------------------------------------------------- #
# Function to apply neck-shaft angle (NSA) deformation to markers

import opensim as osim
import numpy as np
from .getAxisRotMat import getAxisRotMat
from .getOpenSimVersion import getOpenSimVersion

def applyValgusVarusToMarkers(osimModel, aSegmentName, valgusAxis, nsa_angle_func_rad):
    """
    Applies neck-shaft angle (varus/valgus) deformation to marker positions.
    
    Parameters:
    -----------
    osimModel : osim.Model
        OpenSim model
    aSegmentName : str
        Name of body with markers (e.g., 'femur_r')
    valgusAxis : str
        Axis for frontal plane deformation
    nsa_angle_func_rad : function
        Function returning NSA angle (radians) at position along bone
    
    Returns:
    --------
    osimModel : osim.Model
        Model with updated marker positions
    """
    
    print('--------------------------')
    print(' ADJUSTING MARKERS ')
    print('--------------------------')
    
    # check if segment is included in the model
    if osimModel.getBodySet().getIndex(aSegmentName) < 0:
        raise ValueError('The specified segment is not included in the OpenSim model')
    
    # converting the axis in the index used later
    RotMat, axis_ind = getAxisRotMat(valgusAxis)
    
    OpenSimVersion, _ = getOpenSimVersion()
    
    # extracting markerset
    MarkerSet = osimModel.getMarkerSet()
    N_mark = MarkerSet.getSize()
    processed_markers = []
    ntm = 1
    
    # loop through the markers
    for n_mark in range(N_mark):
        
        # current marker
        curr_Mark = MarkerSet.get(n_mark)
        
        # Body attached to marker
        attachBodyName = curr_Mark.getBody().getName()
        
        if attachBodyName == aSegmentName:
            
            # keep track
            if curr_Mark.getName() not in processed_markers:
                processed_markers.append(curr_Mark.getName())
                ntm = ntm + 1
            
            # marker coordinates
            if OpenSimVersion < 4.0:  # OpenSim 3.3
                markerLocVec3 = curr_Mark.getLocation()
            else:  # OpenSim 4.x
                markerLocVec3 = curr_Mark.get_location()
            
            # convert to numpy array
            markerLocCoords = np.array([markerLocVec3.get(0), 
                                        markerLocVec3.get(1), 
                                        markerLocVec3.get(2)])
            
            # compute NSA deformation matrix for the marker
            NSARotMat = RotMat(nsa_angle_func_rad(markerLocCoords[axis_ind])[0])
            
            # compute new marker coordinates
            new_markerLocCoords = (NSARotMat @ markerLocCoords.T).T
            
            if OpenSimVersion < 4.0:  # OpenSim 3.3
                curr_Mark.setLocation(osim.Vec3(new_markerLocCoords[0], 
                                                 new_markerLocCoords[1], 
                                                 new_markerLocCoords[2]))
            else:  # OpenSim 4.x
                curr_Mark.set_location(osim.Vec3(new_markerLocCoords[0], 
                                                  new_markerLocCoords[1], 
                                                  new_markerLocCoords[2]))
    
    if ntm > 1:
        print(f'Processed {ntm-1} markers:')
        print_str = ''
        for nd in range(len(processed_markers)):
            print_str += f"{processed_markers[nd]+ '   '}"
            if np.mod(nd+1, 4) == 0:
                print(print_str)
                print_str = ''
        if print_str:
            print(print_str)
    else:
        print('No markers found on specified segment.')
    
    return osimModel