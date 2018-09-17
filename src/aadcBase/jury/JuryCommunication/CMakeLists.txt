cmake_minimum_required(VERSION 3.10.0)
project(jury_communication_tcp)

if(NOT TARGET adtf::systemsdk)
    find_package(ADTF COMPONENTS systemsdk)
endif()

set (CATEGORY   jury)
set (SOURCES    TcpSource.h
				TcpSource.cpp
				TcpSink.h
				TcpSink.cpp)

#we use the special ADTF CMake command to add an shared object/DLL which has an extension called .adtfplugin
adtf_add_streaming_service(${PROJECT_NAME} ${SOURCES})

adtf_install_target(${PROJECT_NAME} bin)

#this is to add a folder within the ADTF examples Solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER ${CATEGORY})

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
    DEPENDENT_PLUGINS
        adtf_kernel
)

