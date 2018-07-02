project(object_detection)    

include_directories( ${OpenCV_INCLUDE_DIRS} )

adtf_add_filter(${PROJECT_NAME}
    stdafx.h
    ObjectDetection.cpp
    ObjectDetection.h
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS} )

adtf_install_plugin(${PROJECT_NAME} bin)

#install dlls for opencv to binary folder 
if(WIN32)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}d.dll" 
    DESTINATION ${CMAKE_INSTALL_BINARY}/debug CONFIGURATIONS Debug)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}.dll"
    DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS RelWithDebInfo Release)
endif(WIN32)

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin")        

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER algorithms)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER algorithms)