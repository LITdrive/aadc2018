cmake_minimum_required(VERSION 3.10.0)

project(LITD_StanleyControl)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIRS_HEADER_ONLY})

set (SOURCEFILES    	stdafx.h
                        math_utilities.h
                        LITD_StanleyControl.h
			            LITD_StanleyControl.cpp
                        LITD_VirtualPoint.h
                        LITD_VirtualPoint.cpp
                        ../utils/properties/FilePropertiesObserver.h
						../utils/properties/FilePropertiesObserver.cpp
                                  )

adtf_add_filter(${PROJECT_NAME}
				${SOURCEFILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC .)

# Specify where it should be installed to
adtf_install_target(${PROJECT_NAME} bin)

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
 )

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
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER user)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER user)
