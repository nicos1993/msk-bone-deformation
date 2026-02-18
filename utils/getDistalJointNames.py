import opensim as osim
from .getOpenSimVersion import getOpenSimVersion
# given a body, return all joints of which that body is parent.
def getDistalJointNames(osimModel, bodyName):

    # extract all joints
    modelJointSet = osimModel.getJointSet()
    N_j = modelJointSet.getSize()

    # counter of distal joints
    n_d = 0

    OpenSimVersion, _ = getOpenSimVersion()

    distalJointSetNames = []

    for n_j in range(N_j):

        # get parent body name for each joint
        # OpenSim 3.3
        if OpenSimVersion<4.0:
            jointParentName = modelJointSet.get(n_j).getParentBody().getName()
        else:
            # OpenSim 4.x
            jointParentName = modelJointSet.get(n_j).getParentFrame().findBaseFrame().getName()

        # when matching with bodyName save name
        if jointParentName == bodyName:
            distalJointSetNames.append(modelJointSet.get(n_j).getName())
            n_d = n_d + 1        

    return distalJointSetNames