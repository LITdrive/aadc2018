cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

cmake_policy(SET CMP0002 NEW)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0011 NEW)

# The project and cmake vars must only be defined in case that only the base
# filter should be build.
if(NOT AADC_BUILD_ALL_IN_ONE)
    # define a project name
    project(aadc_demo)

    # CMAKE_BUILD_TYPE is empty on Unix systems, but needed by ADTF. Otherwise the ADTF libs
    # can't be found and linked.
    if(UNIX AND NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
                "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
                FORCE)
    endif(UNIX AND NOT CMAKE_BUILD_TYPE)



endif(NOT AADC_BUILD_ALL_IN_ONE)

#-------General------------

set(AADC_FOUND FALSE)
set(AADC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../)
find_package(AADC REQUIRED)
if (AADC_FOUND)
   message(STATUS "AADC Config File found")
else(AADC_FOUND)
   message(FATAL_ERROR "-- AADC not found")
endif (AADC_FOUND)

find_package(ADTF 3.3.0 REQUIRED)
message(STATUS "Found ADTF: ${ADTF_VERSION}")

#--------------add private macros-----------------
include(${AADC_DIR}/AADC_PRIVATE.cmake)
include(${AADC_DIR}/AADCConfigVersion.cmake)
#--------------sources----------------------------

# include the include directory from aadcBase

include_directories(${AADC_DIR}/include)
include_directories(${CMAKE_INSTALL_INCLUDE})

add_subdirectory(${AADC_DIR}/description description)
add_subdirectory(datafilter)
add_subdirectory(helper)
add_subdirectory(jury)
add_subdirectory(algorithms)
add_subdirectory(sensor)
add_subdirectory(markers)





