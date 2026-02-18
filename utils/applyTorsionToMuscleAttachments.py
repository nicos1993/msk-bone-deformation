import opensim as osim
import numpy as np
from utils.getAxisRotMat import getAxisRotMat
from .getOpenSimVersion import getOpenSimVersion
import math

def applyTorsionToMuscleAttachments(osimModel, aSegmentName, aTorsionAxisString, torsion_angle_func_rad):

    # default: deform viapoints (legacy option)
    deformViapoint = 'yes'

    print('------------------------------')
    print(' ADJUSTING MUSCLE ATTACHMENTS ')
    print('------------------------------')

    # check if segment is included in the model
    if osimModel.getBodySet().getIndex(aSegmentName)<0:
        raise ValueError('The specified segment is not included in the OpenSim model')
    
    # converting the axis in the index used later
    RotMat, axis_ind = getAxisRotMat(aTorsionAxisString)

    OpenSimVersion, _ = getOpenSimVersion()

    # extracting muscleset
    Muscles = osimModel.getMuscles()
    N_mus = Muscles.getSize()
    processed_muscles = []
    ntm = 1

    # state now required
    state = osimModel.initSystem()

    # loop through the muscles
    for n_mus in range(N_mus):
        
        # current muscles
        curr_Mus = Muscles.get(n_mus)
        
        # extracting the path
        currentPathPointSet = curr_Mus.getGeometryPath().getPathPointSet()
        
        # number of points
        N_p = currentPathPointSet.getSize()

        # looping through the points of the PathPointSet
        for n_p in range(N_p):

            # skip the point if viapoints are not be deformed
            if deformViapoint == 'no' and (n_p != 0 or n_p != N_p-1):
                continue

            # Body attached to each point of the PathPointSet
            attachBodyName = currentPathPointSet.get(n_p).getBody().getName()

            if attachBodyName == aSegmentName:

                # keep track 
                if curr_Mus.getName() not in processed_muscles:
                    processed_muscles.append(curr_Mus.getName())
                    ntm = ntm + 1

                # point coordinates
                # OpenSim 3.3
                if OpenSimVersion<4.0: 
                    musAttachLocVec3 =  currentPathPointSet.get(n_p).getLocation()
                else:
                    # OpenSim 4.x
                    musAttachLocVec3 =  currentPathPointSet.get(n_p).getLocation(state)
                
                curr_pathpoint_class  = currentPathPointSet.get(n_p).getConcreteClassName()

                if curr_pathpoint_class == 'PathPoint' or curr_pathpoint_class == 'ConditionalPathPoint':

                    # convert to Matlab var
                    musAttachLocCoords = np.array([musAttachLocVec3.get(0), musAttachLocVec3.get(1), musAttachLocVec3.get(2)])

                    # compute torsion metric for the attachment point
                    TorsRotMat = RotMat(torsion_angle_func_rad(musAttachLocCoords[axis_ind])[0])

                    # compute new muscle attachment coordinates
                    new_musAttachLocCoords = (TorsRotMat @ musAttachLocCoords.T).T

                    if OpenSimVersion<4.0: #OpenSim 3.3
                        currentPathPointSet.get(n_p).setLocationCoord(0,new_musAttachLocCoords[0])
                        currentPathPointSet.get(n_p).setLocationCoord(1,new_musAttachLocCoords[1])
                        currentPathPointSet.get(n_p).setLocationCoord(2,new_musAttachLocCoords[2])
                    else:   #OpenSim 4.x
                            # getPathPoint returns an AbstractPathPointSet. Requires
                            # downcasting
                        currentPathPoint = getattr(osim, curr_pathpoint_class).safeDownCast(currentPathPointSet.get(n_p))
                        # setting the muscle PathPointSet as Vec3
                        new_musAttachLocCoords_vec3 = osim.Vec3(new_musAttachLocCoords[0], new_musAttachLocCoords[1], new_musAttachLocCoords[2])
                        currentPathPoint.setLocation(new_musAttachLocCoords_vec3)

                elif curr_pathpoint_class == 'MovingPathPoint':

                    currentPathPoint = osim.MovingPathPoint.safeDownCast(currentPathPointSet.get(n_p))

                    # extract the pqthpoints
                    px = currentPathPoint.get_x_location()
                    py = currentPathPoint.get_y_location()
                    pz = currentPathPoint.get_z_location()
                    
                    # extract functions
                    fx = osim.SimmSpline.safeDownCast(px)
                    fy = osim.SimmSpline.safeDownCast(py)
                    fz = osim.SimmSpline.safeDownCast(pz)
                    coord_set = ['x','y','z']

                    for nc in range(3):
                        cur_coord = coord_set[nc]
                        # extract joint angles (x) from coordinate of interest
                        Npp = eval('f'+cur_coord+'.getX().getSize()')
                        Xpoints = eval('f'+cur_coord+'.getX()')

                        for npp in range(Npp):
                            # curr joint angle
                            cur_angle = Xpoints.get(npp)
                            # compute point coordinates at that joint angle
                            Px = fx.calcValue(osim.Vector(1,cur_angle))
                            Py = fy.calcValue(osim.Vector(1,cur_angle))
                            Pz = fz.calcValue(osim.Vector(1,cur_angle))
                            # build a point
                            musAttachLocCoords = np.array([Px, Py, Pz])
                            # compute torsion metric for the attachment point
                            TorsRotMat = RotMat(torsion_angle_func_rad(musAttachLocCoords[axis_ind])[0])
                            # compute new muscle attachment coordinates
                            new_musAttachLocCoords = (TorsRotMat @ musAttachLocCoords.T).T
                            # assign to spline
                            eval('f'+cur_coord+'.setY(npp, new_musAttachLocCoords[nc])')

    print(f'Processed {ntm-1} muscles:')
    print_str = ''
    for nd in range(1,len(processed_muscles)+1):
        #print(np.mod(nd, math.floor((ntm-1)/2 + 0.5)))
        if np.mod(nd, math.floor((ntm-1)/2 + 0.5))==0:
            print(print_str)
            print_str = ''
        print_str += f"{processed_muscles[nd-1]+ '   '}"
    
    # remaining muscles
    print_str += f"{processed_muscles[nd-1]+ '   '}"

    return osimModel