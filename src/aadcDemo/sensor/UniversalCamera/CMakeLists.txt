
cmake_minimum_required(VERSION 3.10.0)
project(universal_camera)

if (NOT TARGET adtf::filtersdk)
	find_package(ADTF COMPONENTS filtersdk)
endif()
	
set (SOURCEFILES        
    stdafx.h 
	UniversalCamera.h
	UniversalCamera.cpp)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ../../include
    SYSTEM ${OpenCV_INCLUDE_DIRS}
    )
	

 
#install dlls for opencv to binary folder 
if(WIN32)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}d.dll" DESTINATION ${CMAKE_INSTALL_BINARY}/debug CONFIGURATIONS Debug)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS RelWithDebInfo Release)
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
	
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenCV_LIBS})

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER sensor)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER sensor)

