cmake_minimum_required(VERSION 3.10.0)

project(LITD_StanleyControl)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/home/aadc/share/adtf/include/*)

set (SOURCEFILES    	stdafx.h 
			            LITD_StanleyControl.h
			            LITD_StanleyControl.cpp
                        LITD_VirtualPoint.h
                        LITD_VirtualPoint.cpp
                        math_utilities.h)

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
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER user)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER user)
