cmake_minimum_required(VERSION 3.10.0)

project(LITD_LaneGenerator)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS_HEADER_ONLY})

if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=gnu++0x)
endif()

set (SOURCEFILES  LITD_LaneGenerator.cpp
		  LITD_LaneGenerator.h
		  LITD_Map.cpp
		  LITD_MapElementCrossing.cpp
		  LITD_MapElementCrossing.h
		  LITD_MapElementCurve.cpp
		  LITD_MapElementCurve.h
		  LITD_MapElementDoubleCurve.cpp
		  LITD_MapElementDoubleCurve.h
		  LITD_MapElementDoubleStraight.cpp
		  LITD_MapElementDoubleStraight.h
		  LITD_MapElement.h
		  LITD_MapElementStraight.cpp
		  LITD_MapElementStraight.h
		  LITD_Map.h
		  LITD_VirtualPoint.cpp
		  LITD_VirtualPoint.h
		  math_utilities.h
		  stdafx.h
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
