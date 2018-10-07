# general
add_subdirectory(playground)
add_subdirectory(TemplateFilter)
add_subdirectory(OpenCVTemplate)

# tensorflow
if (THIS_IS_A_CAR)
	add_subdirectory(tensorflowPlayground)
	add_subdirectory(TensorFlowTemplate)
endif(THIS_IS_A_CAR)
# zeromq
add_subdirectory(LITD_ZMQ_ImageTemplate)
add_subdirectory(LITD_ZMQ_SensorTemplate)
add_subdirectory(LITD_ZMQ_YoloTemplate)
add_subdirectory(LITD_ZMQ_TrajectoryTemplate)
