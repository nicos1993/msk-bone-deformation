import opensim as osim
import numpy as np

from .getOpenSimVersion import getOpenSimVersion
from .getAxisRotMat import getAxisRotMat

def applyTorsionToMarkers(osimModel, aSegmentName, aTorsionAxisString, torsion_angle_func_rad):

    # check if segment is included in the model
    if osimModel.getBodySet().getIndex(aSegmentName)<0:
        raise ValueError('The specified segment is not included in the OpenSim model')

    print('-------------------')
    print(' ADJUSTING MARKERS ')
    print('-------------------')

    # converting the axis in the index used later
    RotMat, axis_ind = getAxisRotMat(aTorsionAxisString)

    # extracting MarkerSet
    markers = osimModel.getMarkerSet()
    N_markers = markers.getSize()

    OpenSimVersion, _ = getOpenSimVersion()

    # loop through the muscles
    for n_marker in range(N_markers):
        
        # current muscles
        curr_marker = markers.get(n_marker)
            
        # Body attached to each point of the PathPointSet
        if OpenSimVersion<4.0: #OpenSim 3.3
            attachBodyName = curr_marker.getBodyName()
        else: #OpenSim 4.x
            attachBodyName = curr_marker.getParentFrame().getName()

        if attachBodyName == aSegmentName:

            print('processing ', curr_marker.getName())

            # point coordinates
            if OpenSimVersion<4.0: #OpenSim 3.3
                markerLocVec3 =  curr_marker.getOffset()
            else: #OpenSim 4.x
                markerLocVec3 =  curr_marker.get_location()

            # convert to np.array
            markerLocCoords = np.array([markerLocVec3.get(0),markerLocVec3.get(1),markerLocVec3.get(2)])
            
            # compute torsion metric for the attachment point
            TorsRotMat = RotMat(torsion_angle_func_rad(markerLocCoords[axis_ind])[0])

            # compute new muscle attachment coordinates
            new_markerLocCoords = (TorsRotMat @ markerLocCoords.T).T

            # transform to OpenSim Vec3
            newOffset = osim.Vec3(new_markerLocCoords[0], new_markerLocCoords[1], new_markerLocCoords[2])

            # setting the torsioned marker offset
            if OpenSimVersion<4.0: #OpenSim 3.3
                curr_marker.setOffset(newOffset)
            else: #OpenSim 4.x
                curr_marker.set_location(newOffset)

    return osimModel