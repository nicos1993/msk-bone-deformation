import opensim as osim
# --------------------------------------------------------------------------
# Function replacing the old getJoint() method that we all loved in OpenSim
# 3.3 but currently non available in OpenSim 4.x.
# Returns the joint for which the specified body would be the child.
# --------------------------------------------------------------------------
def getBodyJoint(*args):

    n_inputs = len(args)

    osimModel = args[0]
    aBodyName = args[1]
    debug_printout = 0 if n_inputs < 3 else args[2]

    # default
    bodyJoint = []

    jointName = []

    if aBodyName == 'ground':
        return
    
    # check if body is included in the model
    if osimModel.getBodySet().getIndex(aBodyName)<0:
        raise Exception('getBodyJoint.py The specified body ', aBodyName, ' is not included in the OpenSim model')
    
    # get jointset
    jointSet = osimModel.getJointSet()

    # loop through jointset
    nj = 0

    for n_joint in range(jointSet.getSize()):

        # get cur joint
        cur_joint = jointSet.get(n_joint)

        # child frame from joint
        child_frame = cur_joint.getChildFrame()

        # link back to base frame: this could be a body
        body_of_frame = child_frame.findBaseFrame()

        # get base frame name
        possible_body_name = body_of_frame.getName()

        if osimModel.getBodySet().getIndex(possible_body_name) >= 0 and aBodyName == possible_body_name:

            # save the joints with the specified body as Child
            jointName.append(cur_joint.getName())

            if debug_printout:
                print(aBodyName + ' is parent frame on joint: ', jointName[nj])

            nj = nj + 1
            continue

    # return a string if there is only one body
    if len(jointName) == 1:
        jointName = jointName[0]
        bodyJoint = osimModel.getJointSet().get(jointName)
    else:
        raise Exception('getBodyJoint.py More than one joint connected to body of interest ', aBodyName, '. This function is design to work as getJoint() in OpenSim 3.3')

    return bodyJoint