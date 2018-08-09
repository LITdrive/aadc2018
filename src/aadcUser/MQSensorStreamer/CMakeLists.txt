cmake_minimum_required(VERSION 3.10.0)

project(mq_sensor_streamer)

# find libzmq
if(UNIX)
	set(ZMQ_DIR "/opt/libzmq")
else(UNIX)
	set(ZMQ_DIR "C:/SDK/libzmq")
endif(UNIX)

set(ZMQ_LIBRARY_DIRS "${ZMQ_DIR}/build/lib")
set(ZMQ_INCLUDE_DIRS "${ZMQ_DIR}/include")

FIND_LIBRARY(ZMQ_LIBS NAMES
		libzmq.a
		PATHS
		"${ZMQ_LIBRARY_DIRS}"
		)

if (ZMQ_LIBS)
	message(STATUS "Found ZeroMQ. ZeroMQ lib dir is: ${ZMQ_LIBRARY_DIRS}")
	set(AADC_ZMQ_FOUND TRUE)
else (ZMQ_LIBS)
	message(FATAL_ERROR "ZeroMQ libs not found under ${ZMQ_LIBRARY_DIRS}")
endif (ZMQ_LIBS)

include_directories(${ZMQ_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

set (SOURCEFILES    	stdafx.h
		zmq.hpp
		zmq_addon.hpp
		MQSensorStreamer.h
		MQSensorStreamer.cpp)

adtf_add_filter(${PROJECT_NAME}
				${SOURCEFILES}
)

# Specify where it should be installed to
adtf_install_target(${PROJECT_NAME} bin)

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
 )

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${ZMQ_LIBS} LINK_PUBLIC ${OpenCV_LIBS})

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER user)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER user)
