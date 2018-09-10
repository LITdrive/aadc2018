# cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Define project name
project(tensorflowPlayground)


set(CMAKE_CXX_STANDARD 11)


# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them

# Add tensorflow headers location to your include paths
include_directories(${Protobuf_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${TENSORFLOW_INCLUDE_DIRS})



# Declare the executable target built from your sources
add_executable(${PROJECT_NAME} tensorflowPlayground.cpp)


install(TARGETS ${PROJECT_NAME} DESTINATION bin/application CONFIGURATIONS RelWithDebInfo Release)

install(TARGETS ${PROJECT_NAME} DESTINATION bin/debug/application CONFIGURATIONS Debug)

# Link your application with tensorflow libraries
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${Protobuf_LIBRARIES} ${TENSORFLOW_LIBS} )
