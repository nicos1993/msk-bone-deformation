import numpy as np
def getAxisRotMat(aStringAxis):

    # uniform the input
    stringAxis = aStringAxis.lower()

    # get the corresponding rotation matrix as implicit function
    match stringAxis:
        case 'x':
            aRotMat = lambda a: np.array(
                          [[1, 0, 0],
                           [0, np.cos(a), -np.sin(a)],
                           [0, np.sin(a), np.cos(a)]])
            axis_ind = 0
        case 'y':
            aRotMat = lambda a: np.array(
                          [[np.cos(a), 0, np.sin(a)],
                           [0, 1, 0],
                           [-np.sin(a), 0, np.cos(a)]])
            axis_ind = 1
        case 'z':
            aRotMat = lambda a: np.array(
                          [[np.cos(a), -np.sin(a), 0],
                           [np.sin(a), np.cos(a), 0],
                           [0, 0, 1]])
            axis_ind = 2
        case _:
            raise Exception('Please specify a rotation axis as ''x'', ''y'' or ''z''.')

    return aRotMat, axis_ind