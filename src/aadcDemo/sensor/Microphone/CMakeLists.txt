cmake_minimum_required(VERSION 3.10.0)
project(microphone)

if (NOT TARGET adtf::ui)
    find_package(ADTF COMPONENTS ui)
endif()

#ind_package(adtfui REQUIRED)

set_property(GLOBAL PROPERTY AUTOGEN_TARGETS_FOLDER AutoMoc)

adtf_use_qt(Core Multimedia)


set (SOURCEFILES        
    stdafx.h 
	Microphone.h
        Microphone.cpp
	AudioIODevice.h
	AudioIODevice.cpp)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ../../include
    )
	
# Instruct CMake to run moc automatically when needed.
set (CMAKE_AUTOMOC ON)
set (CMAKE_AUTOUIC ON)
set (CMAKE_AUTORCC ON)	

#we use the special ADTF CMake command to add an shared object/DLL which has an extension called .adtfplugin
adtf_add_streaming_service(${PROJECT_NAME} ${SOURCEFILES})

#install the program you need 
adtf_install_target(${PROJECT_NAME} bin)

target_link_libraries(${PROJECT_NAME} PUBLIC Qt5::Core Qt5::Multimedia)

adtf_create_plugindescription(
	TARGET
		${PROJECT_NAME}
	PLUGIN_SUBDIR
		bin
	DEPENDENT_PLUGINS
		"adtf_xsystem" 
)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER sensor)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER sensor)
