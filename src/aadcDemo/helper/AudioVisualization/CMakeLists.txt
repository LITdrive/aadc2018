cmake_minimum_required(VERSION 3.10.0)

project(audio_visualization)

if (NOT TARGET adtf::ui)
    find_package(ADTF COMPONENTS filtersdk ui)
endif()

find_package(adtfui REQUIRED)

#define new targets for qt automoc generated projects (since cmake 3.9 autogen projects are created separately)
set_property(GLOBAL PROPERTY AUTOGEN_TARGETS_FOLDER AutoMoc)

adtf_use_qt(Widgets Gui Core Charts)

set (HEADERS    stdafx.h
                AudioVisualization.h
)
set (SOURCES
                AudioVisualization.cpp
)




# Instruct CMake to run moc automatically when needed.
set (CMAKE_AUTOMOC ON)
set (CMAKE_AUTOUIC ON)
set (CMAKE_AUTORCC ON)

adtf_add_filter(${PROJECT_NAME}
    ${HEADERS}
    ${SOURCES}
)

set_property(TARGET ${PROJECT_NAME} PROPERTY AUTOGEN_TARGET_DEPENDS ${HEADERS})

target_link_libraries(${PROJECT_NAME} PUBLIC adtf::ui Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Charts )
target_include_directories(${PROJECT_NAME} PUBLIC .)

if(WIN32)
    install(FILES "${QT_DIR}/bin/Qt5Chartsd.dll" 
    DESTINATION ${CMAKE_INSTALL_BINARY}/debug CONFIGURATIONS Debug)
    install(FILES "${QT_DIR}/bin/Qt5Charts.dll" 
    DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS RelWithDebInfo Release)
endif(WIN32)

#install the program you need    
adtf_install_target(${PROJECT_NAME} bin)

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
    DEPENDENT_PLUGINS
        "adtf_xsystem")

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER helper)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER helper)