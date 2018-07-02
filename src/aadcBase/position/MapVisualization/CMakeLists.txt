cmake_minimum_required(VERSION 3.10.0)

project(map_visualization)

if (NOT TARGET adtf::ui)
    find_package(ADTF COMPONENTS filtersdk ui)
endif()

adtf_use_qt(Widgets Gui Core)

include_directories(mapReader)

set (SOURCEFILES cMapVisualization.h
                 cMapVisualization.cpp
                 displaywidget.cpp
                 displaywidget.h
                 stdafx.h
                 mapReader/openDriveReader.h
                 mapReader/openDriveReader.cpp
                 mapReader/tinyxml2.h
                 mapReader/tinyxml2.cpp 
)



adtf_add_filter(${PROJECT_NAME}
                ${SOURCEFILES}
)

target_link_libraries(${PROJECT_NAME} PUBLIC adtf::ui Qt5::Widgets)

adtf_disable_qt_warnings(${PROJECT_NAME})

adtf_install_target(${PROJECT_NAME} bin)

#this is to add a folder within the ADTF examples solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER position)

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
    DEPENDENT_PLUGINS
        "adtf_xsystem"
)
