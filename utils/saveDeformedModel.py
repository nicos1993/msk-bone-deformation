import opensim as osim

def saveDeformedModel(osimModel, output_model_path):

    print('-----------------------')
    print(' SAVING DEFORMED MODEL ')
    print('-----------------------')

    # update credits
    osimModel.setAuthors('Created by the Python deformation tool developed by Luca Modenese (2021) See original model file for related information.')
    # print model
    osimModel.printToXML(output_model_path)
    # inform user
    print(f'model saved as {output_model_path}.')
    print('Done.')

    return