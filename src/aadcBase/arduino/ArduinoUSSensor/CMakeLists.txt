cmake_minimum_required(VERSION 3.10.0)
project(arduino_us_sensors)

if (NOT TARGET adtf::filtersdk)
    find_package(ADTF COMPONENTS filtersdk)
endif()


include_directories(
    ${Boost_INCLUDE_DIR} 
)
set (CATEGORY   arduino/streaming_sources)
set (SOURCES    ArduinoUSSensor.h
				ArduinoUSSensor.cpp
				../ArduinoBase.h
                ../ArduinoBase.cpp
                ../helper/SerialDeviceBoost.h
                ../helper/SerialDeviceBoost.cpp
                ../helper/arduino_com_client.h
                ../helper/arduino_com_client.cpp
                ../helper/arduino_com_helpers.h)



#we use the special ADTF CMake command to add an shared object/DLL which has an extension called .adtfplugin
adtf_add_streaming_service(${PROJECT_NAME} ${SOURCES})

#install the program you need 
adtf_install_target(${PROJECT_NAME} bin)

#this is to add a folder within the ADTF examples solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER ${CATEGORY})

target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})

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

#TODO Here we fail at the moment:
adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
    MERGE_DESCRIPTION
        "arduino_us_sensors.plugindescription"
    DEPENDENT_PLUGINS
)
