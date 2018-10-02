cmake_minimum_required(VERSION 3.10.0)

project(litd_zmq_decision)

if (NOT TARGET adtf::ui)
    find_package(ADTF COMPONENTS filtersdk ui)
endif()

include_directories(${ZMQ_INCLUDE_DIRS})

set (HEADERS	ZmqDecision.h
				../utils/zeromq/ZmqBase.h
				)

set (SOURCES	ZmqDecision.cpp
				../utils/zeromq/ZmqBase.cpp
				)

adtf_add_filter(${PROJECT_NAME}
				${HEADERS}
				${SOURCES}
				)

target_include_directories(${PROJECT_NAME} PUBLIC .)

#install the program you need
adtf_install_target(${PROJECT_NAME} bin)

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
)

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${ZMQ_LIBS})

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER zeromq)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER zeromq)