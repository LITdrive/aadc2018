cmake_minimum_required(VERSION 3.10.0)

project(LITD_LaneGenerator)

include_directories(${OpenCV_INCLUDE_DIRS})

if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=gnu++0x)
endif()

set (SOURCEFILES  LITD_VirtualPoint.cpp
                  LITD_MapElementStraight.cpp
                  LITD_MapElementDoubleStraight.cpp
                  LITD_MapElementCurve.cpp
                  LITD_MapElementDoubleCurve.cpp
                  LITD_Map.cpp
                  LITD_LaneGenerator.cpp
                  )

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
