cmake_minimum_required(VERSION 2.8)
project(utils_properties)

include_directories(
    ${Boost_INCLUDE_DIR} 
)

set (SOURCEFILES	FilePropertiesObserver.cpp
					FilePropertiesObserver.h
					FilePropertiesObserverTest.cpp)

add_executable(${PROJECT_NAME} ${SOURCEFILES})

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

set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER utils)
