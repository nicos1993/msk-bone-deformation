#-------------------------------------------------------------------------#
#    Copyright (c) 2026 Haralabidis N.                                    #
#    Author:   Nicos Haralabidis,  2026                                   #
#    email:    N.Haralabidis@unsw.edu.au                                  #
# ----------------------------------------------------------------------- #
# Function to apply neck-shaft angle (NSA) deformation to VTP bone geometries

from .getAxisRotMat import getAxisRotMat
from .getOpenSimVersion import getOpenSimVersion
from .applyTorsionToVTPBoneGeom import (getBodyVTPFileNames, readVTPfile_v2, 
                                        writeDeformedVTPGeometry, 
                                        assignDeformedVTPFileNamesToBody)
import opensim as osim
import numpy as np
import os

def applyValgusVarusToVTPGeometry(*args):
    """
    Applies neck-shaft angle (varus/valgus) deformation to VTP bone geometries.
    
    Parameters:
    -----------
    osimModel : osim.Model
        OpenSim model
    bone_to_deform : str
        Name of body to deform (e.g., 'femur_r')
    valgusAxis : str
        Axis for frontal plane deformation
    nsa_angle_func_rad : function
        Function returning NSA angle (radians) at position along bone
    nsa_doc_string : str
        String for naming deformed geometries
    OpenSim_Geometry_folder : str, optional
        Path to OpenSim geometry folder
    
    Returns:
    --------
    osimModel : osim.Model
        Model with updated geometry references
    """
    
    n_inputs = len(args)
    
    osimModel = args[0]
    bone_to_deform = args[1]
    valgusAxis = args[2]
    nsa_angle_func_rad = args[3]
    nsa_doc_string = args[4]
    
    if n_inputs < 6:
        _, osim_version_string = getOpenSimVersion()
        OSGeometry_folder = 'C:/OpenSim '+osim_version_string+'/Geometry/'
    else:
        OSGeometry_folder = args[5]
    
    print('--------------------------')
    print(' ADJUSTING VTP GEOMETRIES ')
    print('--------------------------')
    
    # get VTP files for bone of interest
    print('Geometry folder: ', OSGeometry_folder)
    print('VTP files attached to ', bone_to_deform,':')
    vtpNameSet = getBodyVTPFileNames(osimModel, bone_to_deform)
    
    # converting the axis in the index used later
    RotMat, axis_ind = getAxisRotMat(valgusAxis)
    
    for n_vtp in range(len(vtpNameSet)):
        
        # current geometry
        curr_vtp = vtpNameSet[n_vtp]
        
        # print vtp file
        print('* ', curr_vtp)
        
        # full path to current vtp file
        vtp_file = os.path.join(OSGeometry_folder, curr_vtp)
        
        # reads the original vtp file
        print('   - reading VTP file')
        normals, points = readVTPfile_v2(vtp_file)
        
        # Deforms points and normals with NSA profile
        print('   - applying varus/valgus deformation to points and normals')
        
        # initialize
        new_points = np.zeros([points.shape[0], 3])
        new_normals = np.zeros([normals.shape[0], 3])
        
        for n in range(points.shape[0]):
            # compute NSA deformation matrix at point location
            NSARotMat = RotMat(nsa_angle_func_rad(points[n, axis_ind])[0])
            
            # Apply NSA deformation to points and normals
            new_points[n, :] = (NSARotMat @ points[n, :].T).T
            new_normals[n, :] = (NSARotMat @ normals[n, :].T).T
        
        # writes the deformed geometry
        deformed_vtp_suffix = ('_NSA'+valgusAxis.upper()+nsa_doc_string)
        print('   - writing deformed VTP file')
        writeDeformedVTPGeometry(vtp_file, new_normals, new_points, deformed_vtp_suffix)
    
    # assign to model
    osimModel, _ = assignDeformedVTPFileNamesToBody(osimModel, bone_to_deform, deformed_vtp_suffix)
    
    return osimModel