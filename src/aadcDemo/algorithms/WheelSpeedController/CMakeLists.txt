cmake_minimum_required(VERSION 3.10.0)

project(wheel_speed_controller)

include_directories(${OpenCV_INCLUDE_DIRS})

set (SOURCEFILES    	stdafx.h 
						WheelSpeedController.h
						WheelSpeedController.cpp)

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

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER algorithms)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER algorithms)
