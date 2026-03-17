import opensim as osim
import os

# list models_Miller models to find the one you want to modify, and then update the path in the line below
models_dir = 'models_Miller'

model_files = [f for f in os.listdir(models_dir) if ('TorsTib' in f and 'TorsFem' in f) and f.endswith('.osim')]

for k in range(len(model_files)):

    model_file = model_files[k]
    model = osim.Model(os.path.join(models_dir, model_file))

    torso = model.updBodySet().get('torso')
    name = 'head_offset_frame'
    offsetFrame = osim.PhysicalOffsetFrame(name, torso, osim.Transform())
    offsetFrame.set_translation(osim.Vec3(-0.00287273, 0.594847, 0)) # somewhat arbitrary translation to place the frame at the head, but this can be adjusted as needed
    offsetFrame.set_orientation(osim.Vec3(0, 0, 0))
    torso.addComponent(offsetFrame)

    model.finalizeConnections()
    model.initSystem()

    framePath = osim.StdVectorString()
    framePath.append('/bodyset/torso/head_offset_frame')

    osim.OpenSenseUtilities.addModelIMUs(model, framePath)

    # new model file name
    new_model_file = model_file.replace('.osim', '_headFrame.osim')

    model.initSystem()
    model.printToXML(os.path.join(models_dir, new_model_file))

    # outputPath = osim.StdVectorString()
    # outputPath.append('.*accelerometer_signal')

    # # set the path and name of a trajectory file that contains the states and controls for a full stride of walking, which will be used to analyze the accelerometer signals in the head offset frame
    # trajectory = osim.MocoTrajectory('misc_files/name_the_output_file_here_full_stride_t050_n51_y092_sym_all.sto')

    # signals = osim.analyzeVec3(model, trajectory.exportToStatesTable(), trajectory.exportToControlsTable(), outputPath)
    # signals.setColumnLabels(framePath)
    # signals.updMatrix().setToZero()

    # # check if the false accelerometer signal file exists
    # file_name = 'misc_files/false_accelerometer_signals.sto'
    # if not os.path.exists(file_name):

    #     # write the false accelerometer signals to a file, which can be used as the reference for an acceleration tracking goal in a Moco problem
    #     osim.STOFileAdapterVec3.write(signals, file_name)