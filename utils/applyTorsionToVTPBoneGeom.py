#-------------------------------------------------------------------------#
#    Copyright (c) 2021 Modenese L.                                       #
#    Author:   Luca Modenese,  2021                                       #
#    email:    l.modenese@imperial.ac.uk                                  #
# ----------------------------------------------------------------------- #
# function to deform the vtp bone geometries according to the specified
# torsional profile.

from copy import error
from utils.getAxisRotMat import getAxisRotMat
from .getOpenSimVersion import getOpenSimVersion
import opensim as osim
import numpy as np
import os

def getBodyVTPFileNames(aOsimModel, aBodyName):

    # check if body is included in the model
    if aOsimModel.getBodySet().getIndex(aBodyName)<0:
        raise ValueError('The specified segment is not included in the OpenSim model')
    
    OpenSimVersion, _ = getOpenSimVersion()

    vtpNameSet = []

    # OpenSim 3.3
    if OpenSimVersion<4.0:
        # gets GeometrySet, where the display properties are located
        bodyGeometrySet = aOsimModel.getBodySet().get(aBodyName).getDisplayer().getGeometrySet()
        # Gets the element of the geometrySet
        N_vtp = bodyGeometrySet.getSize()
        # Loops and saved the names of the VTP geometry files
        for n_vtp in range(N_vtp):
            cur_geom = bodyGeometrySet.get(n_vtp)
            vtpNameSet.append(str(cur_geom.getGeometryFile()))
        
    else:
        body = aOsimModel.getBodySet().get(aBodyName)
        # get number of meshes
        N_vtp = body.getPropertyByName('attached_geometry').size()
        for n_vtp in range(N_vtp):
            cur_geom = body.get_attached_geometry(n_vtp)
            # transform to Mesh
            currentMesh = osim.Mesh.safeDownCast(cur_geom)
             # extract file
            vtpNameSet.append(str(currentMesh.get_mesh_file()))
        
    return vtpNameSet    
        
def readVTPfile_v2(vtp_file):

    try:
        with open(vtp_file, 'r') as f:
            lines = f.readlines()
    except FileNotFoundError:
        raise IOError('VTPfile not loaded')

    normals = []
    points = []
    
    for line in lines:
        try:
            data = [float(x) for x in line.strip().split()]
            if len(data) == 3:
                # it is a normal if norm is close to one.
                # the code assumes that normals are listed before points.
                if (abs(np.linalg.norm(data)-1)<0.000001) and len(points) == 0:
                    normals.append(data)
                # otherwise is a point
                elif len(points) < len(normals):
                    points.append(data)
                else:
                    # if not point or normal then exit for loop, This avoids going
                    # through topology.
                    break
        except ValueError:
            # line does not contain 3 floats, so we ignore it
            continue
            
    return np.array(normals), np.array(points)

# this function updates the normals and points of a vtp file with some new
# normals and points given as input
# writes the deformed file in the same folder where the original vtp is
# locates
def writeDeformedVTPGeometry(vtp_file, new_normals, new_points, deformed_vtp_suffix):

    path, name_ext = os.path.split(vtp_file)
    name, _ = os.path.splitext(name_ext)
    
    deformed_vtp_file = os.path.join(path, f"{name}{deformed_vtp_suffix}.vtp")

    try:
        with open(vtp_file, 'r') as f_in, open(deformed_vtp_file, 'w') as f_out:
            
            n_norm = 0
            n_p = 0
            
            for line in f_in:
                write_line = line
                try:
                    data = [float(x) for x in line.strip().split()]
                    if len(data) == 3:
                        # Check for normals
                        if (abs(np.linalg.norm(data)-1)<0.000001) and n_norm < len(new_normals):
                            tline = new_normals[n_norm,:]
                            write_line = f'\t\t\t{tline[0]:-6.6f} {tline[1]:-6.6f} {tline[2]:-6.6f}\r\n'
                            n_norm += 1
                        # Check for points
                        elif n_p < len(new_points) and (abs(np.linalg.norm(data)-1)>=0.000001):
                            tline = new_points[n_p,:]
                            write_line = f'\t\t\t{tline[0]:-6.6f} {tline[1]:-6.6f} {tline[2]:-6.6f}\r\n'
                            n_p += 1
                except (ValueError, IndexError):
                    # This line doesn't contain 3 floats, write it as is.
                    pass
                
                f_out.write(write_line)

    except FileNotFoundError:
        raise IOError('Vtp file not loaded or deformed file could not be created')

    print(f"   - saved as '{deformed_vtp_file}' in geometry folder.")

    return

def assignDeformedVTPFileNamesToBody(aOsimModel, aBodyName, suffix):

    # check if body is included in the model
    if aOsimModel.getBodySet().getIndex(aBodyName)<0:
        raise ValueError('The specified segment is not included in the OpenSim model')
    
    OpenSimVersion, _ = getOpenSimVersion()

    # OpenSim 3.3
    if OpenSimVersion<4.0:
        # gets GeometrySet, where the display properties are located
        bodyGeometrySet = aOsimModel.getBodySet().get(aBodyName).getDisplayer().getGeometrySet()
    
        # Gets the element of the geometrySet
        N_vtp = bodyGeometrySet.getSize()

        newVTPNames = []
        
        # Loops and updates the names of the VTP geometry files
        for n_vtp in range(N_vtp):
            cur_geom = bodyGeometrySet.get(n_vtp)
            # original name
            origName = cur_geom.getGeometryFile()
            # update the vtp file name
            updVTPName = origName[:-4] + suffix + '.vtp'
            # sets new file name for Geometry
            cur_geom.setGeometryFile(updVTPName)
            # stores name
            newVTPNames.append(updVTPName)
            # clear
            del origName 
        
    else:
        body = aOsimModel.getBodySet().get(aBodyName)
        # get number of meshes
        N_vtp = body.getPropertyByName('attached_geometry').size()

        newVTPNames = []

        # Loops and updates the names of the VTP geometry files
        for n_vtp in range(N_vtp):
        
            cur_geom = body.get_attached_geometry(n_vtp)
        
            # transform to Mesh
            currentMesh = osim.Mesh.safeDownCast(cur_geom)
        
            # original name
            origName = currentMesh.get_mesh_file()

            # update the vtp file name
            updVTPName = origName[:-4] + suffix + '.vtp'
            
            # sets new file name for Geometry
            currentMesh.set_mesh_file(updVTPName)
            
            # stores name
            newVTPNames.append(updVTPName)
            
            # clear
            del origName

    return aOsimModel, newVTPNames


def applyTorsionToVTPBoneGeom(*args):

    n_inputs = len(args)

    osimModel = args[0]
    bone_to_deform = args[1]
    torsionAxis = args[2]
    torsion_angle_func_rad = args[3]
    torsion_doc_string = args[4]

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
    RotMat, axis_ind = getAxisRotMat(torsionAxis)

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

        # Deforms points and normals
        print('   - applying torsion to points and normals')
        
        # initialize (required for multiple vtp vones)
        # thanks to Axel Koussou!
        new_points= np.zeros([points.shape[0],3])
        new_normals= np.zeros([normals.shape[0],3])

        for n in range(points.shape[0]):
            # compute torsion matrix
            TorsRotMat = RotMat(torsion_angle_func_rad(points[n,axis_ind])[0])
            
            # New points and axis
            new_points[n,:] = (TorsRotMat @ points[n,:].T).T
            new_normals[n,:] = (TorsRotMat @ normals[n,:].T).T

        # writes the deformed geometry
        deformed_vtp_suffix = ('_Torsion'+torsionAxis.upper()+torsion_doc_string)
        print('   - writing deformed VTP file')
        writeDeformedVTPGeometry(vtp_file, new_normals, new_points, deformed_vtp_suffix)
    
    # assign to model
    osimModel, _ = assignDeformedVTPFileNamesToBody(osimModel, bone_to_deform, deformed_vtp_suffix)

    return osimModel