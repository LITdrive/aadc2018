cmake_minimum_required(VERSION 3.10.0)

project(litd_objectdetection_filter)

include_directories(${Protobuf_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${TENSORFLOW_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

set (SOURCEFILES    	stdafx.h 
						LITD_ObjectDetection.h
						LITD_ObjectDetection.cpp
                        YOLOHandler.h
                        YOLOHandler.cpp)

adtf_add_filter(${PROJECT_NAME}
				${SOURCEFILES}
)

# Specify where it should be installed to
adtf_install_target(${PROJECT_NAME} bin)

#install dlls for opencv to binary folder 
if(WIN32)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}d.dll" DESTINATION ${CMAKE_INSTALL_BINARY}/debug CONFIGURATIONS Debug)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS RelWithDebInfo Release)
endif(WIN32)

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

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
 ) 

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenCV_LIBS} LINK_PUBLIC ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${Protobuf_LIBRARIES} ${TENSORFLOW_LIBS} )

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER user)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER user)
