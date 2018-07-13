cmake_minimum_required(VERSION 3.10.0)

project(camera_calibration)

if (NOT TARGET adtf::ui)
    find_package(ADTF COMPONENTS filtersdk ui)
endif()

find_package(adtfui REQUIRED)

#define new targets for qt automoc generated projects (since cmake 3.9 autogen projects are created separately)
set_property(GLOBAL PROPERTY AUTOGEN_TARGETS_FOLDER AutoMoc)

adtf_use_qt(Widgets Gui)

INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
include_directories(${OpenCV_INCLUDE_DIRS})


set (HEADERS    stdafx.h
						calibrationSettings.h
					    CameraCalibrationFilter.h
						CameraCalibrationWidget.h )
set (SOURCES
					  CameraCalibrationFilter.cpp
					  CameraCalibrationWidget.cpp)


# Instruct CMake to run moc automatically when needed.
set (CMAKE_AUTOMOC ON)
set (CMAKE_AUTOUIC ON)
set (CMAKE_AUTORCC ON)

#uncomment if CMAKE_AUTOMOC/UIC is disabled
#ui/MainWindow.ui 
#set (UI_SOURCE hello.ui)

adtf_add_filter(${PROJECT_NAME}
				${HEADERS}
				#${UI_SOURCE}
				${SOURCES}
				)

set_property(TARGET ${PROJECT_NAME} PROPERTY AUTOGEN_TARGET_DEPENDS ${HEADERS})

target_link_libraries(${PROJECT_NAME} PUBLIC adtf::ui Qt5::Core Qt5::Gui Qt5::Widgets ${Boost_LIBRARIES} )
target_include_directories(${PROJECT_NAME} PUBLIC .)

#install the program you need    
adtf_install_target(${PROJECT_NAME} bin)

#install dlls for opencv to binary folder 
if(WIN32)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}d.dll" DESTINATION ${CMAKE_INSTALL_BINARY}/debug CONFIGURATIONS Debug)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS RelWithDebInfo Release)
endif(WIN32)

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
    DEPENDENT_PLUGINS
        "adtf_xsystem")
		
		



target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenCV_LIBS})

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER sensor)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER sensor)