cmake_minimum_required(VERSION 2.8.11)

project(jury_application)

adtf_use_qt(Widgets Network Xml)

if(WIN32)
  set(CMAKE_WIN32_EXECUTABLE ON)
endif()

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set_property(GLOBAL PROPERTY AUTOGEN_TARGETS_FOLDER AutoMoc)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Find the QtWidgets library
find_package(Qt5Widgets REQUIRED PATHS ${QT_DIR}/lib/cmake/Qt5Widgets)
find_package(Qt5Network REQUIRED PATHS ${QT_DIR}/lib/cmake/Qt5Network)
find_package(Qt5Xml REQUIRED PATHS ${QT_DIR}/lib/cmake/Qt5Xml)

# Tell CMake to create the helloworld executable
add_executable(${PROJECT_NAME} main.cpp JuryApp.cpp JuryApp.h mainwindow.ui)

# Use the Widgets module from Qt 5.
target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::Network Qt5::Xml)

install(TARGETS ${PROJECT_NAME} DESTINATION bin/application CONFIGURATIONS RelWithDebInfo Release)

install(TARGETS ${PROJECT_NAME} DESTINATION bin/debug/application CONFIGURATIONS Debug)

#install dlls for qt to binary folder 
if(WIN32)
    install(FILES "${QT_DIR}/bin/Qt5Gui.dll" DESTINATION bin/application CONFIGURATIONS RelWithDebInfo Release)
    install(FILES "${QT_DIR}/bin/Qt5Core.dll" DESTINATION bin/application CONFIGURATIONS RelWithDebInfo Release)
    install(FILES "${QT_DIR}/bin/Qt5Xml.dll" DESTINATION bin/application CONFIGURATIONS RelWithDebInfo Release)
    install(FILES "${QT_DIR}/bin/Qt5Widgets.dll" DESTINATION bin/application CONFIGURATIONS RelWithDebInfo Release)
    install(FILES "${QT_DIR}/bin/Qt5Network.dll" DESTINATION bin/application CONFIGURATIONS RelWithDebInfo Release)
    
    install(FILES "${QT_DIR}/bin/Qt5Guid.dll" DESTINATION bin/debug/application CONFIGURATIONS Debug)
    install(FILES "${QT_DIR}/bin/Qt5Cored.dll" DESTINATION bin/debug/application CONFIGURATIONS Debug)
    install(FILES "${QT_DIR}/bin/Qt5Xmld.dll" DESTINATION bin/debug/application CONFIGURATIONS Debug)
    install(FILES "${QT_DIR}/bin/Qt5Widgetsd.dll" DESTINATION bin/debug/application CONFIGURATIONS Debug)
    install(FILES "${QT_DIR}/bin/Qt5Networkd.dll" DESTINATION bin/debug/application CONFIGURATIONS Debug)
endif(WIN32)

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER jury)
