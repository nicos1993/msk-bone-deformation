import opensim as osim
# ----------------------------------------------------------------------- %
# This script returns the OpenSim version
# ----------------------------------------------------------------------- %
def getOpenSimVersion():

    # read env variable
    try:
        # method available only from 4.x
        os_version = osim.GetVersion()
        
        # use the last file separator to get the OpenSim installation folder name
        version = os_version[0:3]
        
        # get the string for the opensim version
        osim_version_string = version
        
        # transform in float
        osim_version_float = float(osim_version_string)
        
        return osim_version_float, osim_version_string

    except:
        
        # GetVersion is not available in earlier OpenSim versions
        osim_version_float = 3.3
        osim_version_string = '3.3'
        return osim_version_float, osim_version_string
