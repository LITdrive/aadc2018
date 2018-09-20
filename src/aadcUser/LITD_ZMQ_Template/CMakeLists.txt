cmake_minimum_required(VERSION 3.10.0)

project(litd_zmq_template)

if (NOT TARGET adtf::ui)
    find_package(ADTF COMPONENTS filtersdk ui)
endif()

include_directories(${ZMQ_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR})

set (HEADERS	stdafx.h
				ZmqTemplate.h
				../utils/zeromq/ZmqBase.h
				)

set (SOURCES	stdafx.h
				ZmqTemplate.cpp
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

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${ZMQ_LIBS} LINK_PUBLIC ${Boost_LIBRARIES})

if(WIN32)
    #install Release/RelWithDebInfo
    install(FILES "${Boost_LIBRARY_DIRS}/boost_system-vc140-mt-x64-1_66.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo)
    install(FILES "${Boost_LIBRARY_DIRS}/boost_chrono-vc140-mt-x64-1_66.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo)
    install(FILES "${Boost_LIBRARY_DIRS}/boost_thread-vc140-mt-x64-1_66.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo)
    install(FILES "${Boost_LIBRARY_DIRS}/boost_date_time-vc140-mt-x64-1_66.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo)
    #install Debug
    install(FILES "${Boost_LIBRARY_DIRS}/boost_system-vc140-mt-gd-x64-1_66.dll" DESTINATION ${CMAKE_INSTALL_BINARY}/debug CONFIGURATIONS Debug)
    install(FILES "${Boost_LIBRARY_DIRS}/boost_chrono-vc140-mt-gd-x64-1_66.dll" DESTINATION ${CMAKE_INSTALL_BINARY}/debug CONFIGURATIONS Debug)
    install(FILES "${Boost_LIBRARY_DIRS}/boost_thread-vc140-mt-gd-x64-1_66.dll" DESTINATION ${CMAKE_INSTALL_BINARY}/debug CONFIGURATIONS Debug)
    install(FILES "${Boost_LIBRARY_DIRS}/boost_date_time-vc140-mt-gd-x64-1_66.dll" DESTINATION ${CMAKE_INSTALL_BINARY}/debug CONFIGURATIONS Debug)
endif(WIN32)

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER zeromq)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER zeromq)