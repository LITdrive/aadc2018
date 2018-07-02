
cmake_minimum_required(VERSION 3.10.0)
project(laser_scanner)

if (NOT TARGET adtf::filtersdk)
	find_package(ADTF COMPONENTS filtersdk)
endif()
	
if (WIN32)
	set (ARCH_DIR win32)
	set (ARCH_THREAD winthread)
else(WIN32)
	set (ARCH_DIR linux)
	set (ARCH_THREAD thread)
endif(WIN32)

set (SOURCEFILES        
    stdafx.h 
	LaserScanner.h
	LaserScanner.cpp
	${RPLIDAR_ROOT}/src/rplidar_driver_serial.h
    ${RPLIDAR_ROOT}/src/hal/thread.h
    ${RPLIDAR_ROOT}/src/arch/${ARCH_DIR}/net_serial.cpp
    ${RPLIDAR_ROOT}/src/arch/${ARCH_DIR}/${ARCH_THREAD}.hpp
    ${RPLIDAR_ROOT}/src/arch/${ARCH_DIR}/timer.cpp
	)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ../../include
	${RPLIDAR_INCLUDE_DIRS}
	${RPLIDAR_ROOT}/src
    )
	
	
if(WIN32) 
    #install Release/RelWithDebInfo
	
    #install Debug

endif(WIN32)
	
#we use the special ADTF CMake command to add an shared object/DLL which has an extension called .adtfplugin
adtf_add_streaming_service(${PROJECT_NAME} ${SOURCEFILES})

#install the program you need 
adtf_install_target(${PROJECT_NAME} bin)

adtf_create_plugindescription(
	TARGET
		${PROJECT_NAME}
	PLUGIN_SUBDIR
		bin
	DEPENDENT_PLUGINS
)
	
target_link_libraries(${PROJECT_NAME}
	LINK_PUBLIC
    ${RPLIDAR_LIBS}
)


#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER sensor)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER sensor)
