cmake_minimum_required(VERSION 3.10.0)

project(zeromq_service)

if (NOT TARGET adtf::systemsdk)
    find_package(ADTF COMPONENTS systemsdk)
endif()

include_directories(${ZMQ_INCLUDE_DIRS})

set (SOURCEFILES    stdafx.h
                    zeromq_service_intf.h
                    zeromq_service.h
					zeromq_service.cpp)

adtf_add_system_service(${PROJECT_NAME}
                        ${SOURCEFILES})

adtf_install_target(${PROJECT_NAME} bin)

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
)

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${ZMQ_LIBS})

#this is to add a folder within the ADTF examples solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER services)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER services)
