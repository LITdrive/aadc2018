# cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Define project name
project(LITD_LaneGenerator)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them

if(CMAKE_VERSION VERSION_LESS "2.8.11")
  # Add OpenCV headers location to your include paths
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=gnu++0x)
endif()


include_directories(${OpenCV_INCLUDE_DIRS})


# Declare the executable target built from your sources
#add_executable(${PROJECT_NAME} controlTest.cpp model_predictive_control.h model_predictive_control.cpp)
add_executable(${PROJECT_NAME} #map_generator_test.cpp 
                               LITD_ControlTest.cpp
                               LITD_VirtualPoint.cpp
                               LITD_MapElementStraight.cpp
                               LITD_MapElementDoubleStraight.cpp
                               LITD_MapElementCurve.cpp
                               LITD_MapElementDoubleCurve.cpp
                               LITD_Map.cpp
                               )

install(TARGETS ${PROJECT_NAME} DESTINATION bin/application CONFIGURATIONS RelWithDebInfo Release)

install(TARGETS ${PROJECT_NAME} DESTINATION bin/debug/application CONFIGURATIONS Debug)

# Link your application with OpenCV libraries
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
