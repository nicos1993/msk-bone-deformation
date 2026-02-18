import opensim as osim

def computeSpatialTransformTranslations(osimModel, aCustomJoint):

    # double check if the joint is effectively a CustomJoint
    if aCustomJoint.getConcreteClassName() == 'CustomJoint':
        
        # initialize
        si = osimModel.initSystem()
        
        # get the Spatial Transform
        customJ = osim.CustomJoint.safeDownCast(aCustomJoint)
        
        # get the translations at state si
        # spatial position of Child in Parent as a function of coordinates.
        jointSpatialTransf = customJ.getSpatialTransform()
        t1 = jointSpatialTransf.get_translation1().getValue(si)
        t2 = jointSpatialTransf.get_translation2().getValue(si)
        t3 = jointSpatialTransf.get_translation3().getValue(si)
        
        # ignoring rotations for now
    #     r1 = jointSpatialTransf.get_rotation1().getValue(si)
    #     r2 = jointSpatialTransf.get_rotation2().getValue(si)
    #     r3 = jointSpatialTransf.get_rotation3().getValue(si)
        
        # export the translation vector
        SpatialTransformTrans = (t1,t2,t3)
        
    else:
        print('The provided joint is not a CustomJoint. No SpatialTransform offset.')
        SpatialTransformTrans = (0, 0, 0)

    return SpatialTransformTrans