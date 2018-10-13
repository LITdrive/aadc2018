cmake_minimum_required(VERSION 3.10.0)

project(litd_fft_filter)

#-------------------------------------------------------
#-------FFTW--------------------------------------------
#-------------------------------------------------------

if(UNIX)
	set(FFTW_DIR "/opt/fftw/fftw-3.3.8")
	set(FFTW_LIBRARY_DIRS "${FFTW_DIR}/build")
else(UNIX)
	set(FFTW_DIR "C:/SDK/fftw/fftw-3.3.5-dll64")
	set(FFTW_LIBRARY_DIRS ${FFTW_DIR})
endif(UNIX)

if(UNIX)
	set(FFTW_INCLUDE_DIRS "${FFTW_DIR}/api")
else(UNIX)
	set(FFTW_INCLUDE_DIRS ${FFTW_DIR})
endif(UNIX)

FIND_LIBRARY(FFTW_LIBS NAMES
	fftw3
	libfftw3-3
	PATHS
	"${FFTW_LIBRARY_DIRS}"
	)

if (FFTW_LIBS)
	message(STATUS "Found FFTW. FFTW lib dir is: ${FFTW_LIBRARY_DIRS}")
	set(AADC_FFTW_FOUND TRUE)
else (FFTW_LIBS)
	message(FATAL_ERROR "FFTW libs not found under ${FFTW_LIBRARY_DIRS} (follow instructions in the README!)")
endif (FFTW_LIBS)

#-------------------------------------------------------
#-------------------------------------------------------
#-------------------------------------------------------

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${FFTW_INCLUDE_DIRS})

set (SOURCEFILES    	stdafx.h 
						FFTFilter.h
						FFTFilter.cpp)

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

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
 ) 

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenCV_LIBS} LINK_PUBLIC ${FFTW_LIBS})

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER user)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER user)
