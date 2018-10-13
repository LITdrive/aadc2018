# execute just once
if(AADC_FOUND)
    message(STATUS "AADC config file already found.")
    return()
endif(AADC_FOUND)

set(AADC_OPENCV_FOUND FALSE)
set(AADC_PYLON_FOUND FALSE)
set(AADC_BOOST_FOUND FALSE)

# check if this is a build on one of the aadc cars
cmake_host_system_information(RESULT HOSTNAME_VAR QUERY HOSTNAME)
set (THIS_IS_A_CAR FALSE)
if ((${HOSTNAME_VAR} STREQUAL "annika") OR (${HOSTNAME_VAR} STREQUAL "bob") OR (${HOSTNAME_VAR} STREQUAL "charlie"))
	set (THIS_IS_A_CAR TRUE)
endif()

#-------------------------------------------------------
#-------ADTF Dir----------------------------------------
#-------------------------------------------------------
if (WIN32)
	#either use env var or set dir here
	if ("$ENV{ADTF3_DIR}" STREQUAL "")
		set (ADTF_DIR "C:/SDK/adtf/3.3.3")
	    message(STATUS "ADTF3_DIR not set use default path: ${ADTF_DIR}")
	else ("$ENV{ADTF3_DIR}" STREQUAL "")
		set (ADTF_DIR "$ENV{ADTF3_DIR}")
		message(STATUS "ADTF3_DIR set use this path: ${ADTF_DIR}")
	endif ()
	set (ADTF_ALLOW_UNSUPPORTED_COMPILER ON)
else(WIN32)
	set (ADTF_DIR "/opt/ADTF/3.3.3")
endif(WIN32)

#-------------------------------------------------------
#-------QT Dir------------------------------------------
#-------------------------------------------------------
if (WIN32)
	set (QT_DIR "C:/SDK/qt/5.9.5/msvc2015_64")    
else(WIN32)
	set (QT_DIR "/opt/qt/5.9.0/5.9/gcc_64")
endif(WIN32)

#-------------------------------------------------------
#-------Eigen (header-only)-----------------------------
#-------------------------------------------------------
set (EIGEN3_INCLUDE_DIRS_HEADER_ONLY "${AADC_DIR}/include/Eigen")

#-------------------------------------------------------
#------- OpenCV 3.4.2 ----------------------------------
#-------------------------------------------------------
find_package(OpenCV REQUIRED  PATHS 
		"/opt/opencv/3.4.1"		
		"C:/SDK/opencv/3.4.1")
		

if(NOT OpenCV_LIBS)
    message(FATAL_ERROR "OpenCV lib not found. Please specify the OPENCV_DIR")
else(NOT OpenCV_LIBS)
	set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)
    message(STATUS "OpenCV lib found. OpenCV Version is ${OpenCV_VERSION}")
    set(AADC_OPENCV_FOUND TRUE)	
    message(STATUS "OpenCV Include dir is ${OpenCV_INCLUDE_DIRS}")
	# UNIX: do not forget to add the path to LD_LIBRARY_PATH: add /opt/opencv/3.4.1/lib to file /etc/ld.so.conf.d/opencv.conf
endif(NOT OpenCV_LIBS)
           
#------------------------------------------------------
#-------boost------------------------------------------
#------------------------------------------------------
if (WIN32)
	set (BOOST_ROOT "C:/SDK/boost/1.66.0")    
else(WIN32)
	set (BOOST_ROOT "/opt/boost/1.66.0")
endif(WIN32)

set (Boost_USE_STATIC_LIBS OFF CACHE BOOL "use static libraries from Boost")
set (Boost_USE_MULTITHREADED ON)

find_package(Boost 1.58 REQUIRED COMPONENTS system filesystem thread)
			
if(Boost_FOUND)
	if (WIN32)
	  # disable autolinking in boost
	  add_definitions( -DBOOST_ALL_NO_LIB )

	  # force all boost libraries to dynamic link (we already disabled
	  # autolinking, so I don't know why we need this, but we do!)
	  add_definitions( -DBOOST_ALL_DYN_LINK )
	endif()
	message(STATUS "Found Boost. Boost Version is: ${Boost_VERSION}" )
	message(STATUS "Boost Include dir is ${Boost_INCLUDE_DIR}")
	message(STATUS "Boost Lib dir is ${Boost_LIBRARY_DIRS}")
	#message(STATUS "Boost Libs are ${Boost_LIBRARIES}")
	set(AADC_BOOST_FOUND TRUE)
	# UNIX: do not forget to add the path to LD_LIBRARY_PATH: add /opt/boost/1.66.0/lib to file /etc/ld.so.conf.d/boost.conf
else(Boost_FOUND)
	message(FATAL_ERROR "Boost not found." )		
endif(Boost_FOUND)

#------------------------------------------------------------------	
#--------lib protobuf------------------------------------------
#------------------------------------------------------------------	

if (THIS_IS_A_CAR)
if(UNIX)
	set(protobuf_MODULE_COMPATIBLE TRUE)	
	FIND_PACKAGE(Protobuf 3.5.0 REQUIRED)

	if (Protobuf_FOUND)
		message(STATUS "Found Protobuf lib with version ${Protobuf_VERSION}, libs are: ${Protobuf_LIBRARIES}; Include Dir is ${Protobuf_INCLUDE_DIRS}")
	else (Protobuf_FOUND)
		message(FATAL_ERROR "Protobuf lib not found")
	endif (Protobuf_FOUND)
else(UNIX)
	message(STATUS "Skipping Protobuf lib on Windows.")
endif(UNIX)
endif(THIS_IS_A_CAR)

#------------------------------------------------------------------	
#--------lib eigen------------------------------------------
#------------------------------------------------------------------	

if (THIS_IS_A_CAR)
if(UNIX)	
	FIND_PACKAGE(Eigen3 3.3.4 REQUIRED PATHS "/opt/eigen/3.3.4")

	if (Eigen3_FOUND)
		message(STATUS "Found Eigen ${EIGEN3_VERSION_STRING} ; Include Dir is ${EIGEN3_INCLUDE_DIRS}")
	else (Eigen3_FOUND)
		message(FATAL_ERROR "Eigen not found")
	endif (Eigen3_FOUND)
else(UNIX)
	message(STATUS "Skipping Eigen lib on Windows.")
endif(UNIX)
endif(THIS_IS_A_CAR)

#------------------------------------------------------------------	
#--------lib tensorflow------------------------------------------
#------------------------------------------------------------------	

if (THIS_IS_A_CAR)
if(UNIX)
	set(TENSORFLOW_DIR "/opt/tensorflow/1.8.0")
	set(TENSORFLOW_INCLUDE_DIRS "${TENSORFLOW_DIR}/include")


	FIND_LIBRARY(TENSORFLOW_LIB NAMES 
		tensorflow_cc
		PATHS 
		"${TENSORFLOW_DIR}/lib"
		)

	FIND_LIBRARY(TENSORFLOW_FRAMEWORK NAMES 
		tensorflow_framework
		PATHS 
		"${TENSORFLOW_DIR}/lib"
		)

	set(TENSORFLOW_LIBS "${TENSORFLOW_LIB};${TENSORFLOW_FRAMEWORK}")	
		
		
	if (TENSORFLOW_LIBS)
		message(STATUS "Found Tensorflow. Tensorflow libs are: ${TENSORFLOW_LIBS}; Include Dir is ${TENSORFLOW_INCLUDE_DIRS}")
	else (TENSORFLOW_LIBS)
		message(FATAL_ERROR "Tensorflow lib not found under ${TENSORFLOW_DIR}")
	endif (TENSORFLOW_LIBS)

else(UNIX)
	message(STATUS "Skipping Tensorflow libs on Windows.")
endif(UNIX)
endif(THIS_IS_A_CAR)
       

#-----------------------------------------------------------
#--------lib pylon------------------------------------------
#-----------------------------------------------------------

if(UNIX)
	set(PYLON_DIR "/opt/pylon/5.0.12/pylon5")
	set(PYLON_LIBRARY_DIRS "${PYLON_DIR}/lib64")
	set(PYLON_INCLUDE_DIRS "${PYLON_DIR}/include")
else(UNIX)
	set(PYLON_DIR "C:/SDK/pylon/5.0.12")
	set(PYLON_LIBRARY_DIRS "${PYLON_DIR}/Development/lib/x64")
	set(PYLON_INCLUDE_DIRS "${PYLON_DIR}/Development/include")
	set(PYLON_BINARY_DIRS "${PYLON_DIR}/Runtime/x64")
	# UNIX: do not forget to add the path to LD_LIBRARY_PATH: add /opt/pylon/5.0.12/pylon5/lib64/ to file /etc/ld.so.conf.d/pylon.conf	
endif(UNIX)
	
FIND_LIBRARY(PYLON_UTILITY_LIBS NAMES 
		pylonutility
		PylonUtility_MD_VC120_v5_0
		PATHS 
		"${PYLON_LIBRARY_DIRS}"
		)

FIND_LIBRARY(PYLON_BASE_LIBS NAMES 
		pylonbase
		PylonBase_MD_VC120_v5_0
		PATHS 
		"${PYLON_LIBRARY_DIRS}"
		)

FIND_LIBRARY(GENAPI_LIBS NAMES 
		GenApi_MD_VC120_v3_0_Basler_pylon_v5_0
		GenApi_gcc_v3_0_Basler_pylon_v5_0
		PATHS 
		"${PYLON_LIBRARY_DIRS}"
		)
					
FIND_LIBRARY(GCBase_LIBS NAMES 
	GCBase_MD_VC120_v3_0_Basler_pylon_v5_0
	GCBase_gcc_v3_0_Basler_pylon_v5_0
	PATHS 
	"${PYLON_LIBRARY_DIRS}"
	)
	
FIND_LIBRARY(PYLON_C NAMES 
	pylonc
	PylonC_MD_VC120
	PATHS 
	"${PYLON_LIBRARY_DIRS}"
	)

set(PYLON_LIBS "${PYLON_UTILITY_LIBS};${PYLON_BASE_LIBS};${GENAPI_LIBS};${GCBase_LIBS};${PYLON_C}")	
		
		
if (PYLON_LIBS)
	message(STATUS "Found pylon. Pylon lib dir is: ${PYLON_LIBRARY_DIRS}")
	set(AADC_PYLON_FOUND TRUE)
else (PYLON_LIBS)
	message(FATAL_ERROR "Pylon libs not found under ${PYLON_LIBRARY_DIRS}")
endif (PYLON_LIBS)

#-------------------------------------------------------
#-------RPLidar-----------------------------------------
#-------------------------------------------------------
if (WIN32)
	set (RPLIDAR_ROOT "C:/SDK/rplidar/1.6.0")    
else(WIN32)
	set (RPLIDAR_ROOT "/opt/rplidar/1.6.0")
endif(WIN32)

find_path(RPLIDAR_INCLUDE_DIRS NAMES rplidar.h sdkcommon.h
	PATHS ${RPLIDAR_ROOT}/include
)

FIND_LIBRARY(RPLIDAR_LIBS_RELEASE NAMES 
	librplidar_sdk.a
	rplidar_driver.lib
	PATHS 
	${RPLIDAR_ROOT}/bin/Release
	)
	
FIND_LIBRARY(RPLIDAR_LIBS_DEBUG NAMES 
librplidar_sdk.a
rplidar_driver.lib
PATHS 
${RPLIDAR_ROOT}/bin/Debug
)

SET(RPLIDAR_LIBS
  debug ${RPLIDAR_LIBS_DEBUG}
  optimized ${RPLIDAR_LIBS_RELEASE}
  )

if(NOT RPLIDAR_INCLUDE_DIRS OR NOT RPLIDAR_LIBS)
    	message(FATAL_ERROR "RPLidar lib not found. Please specify the RPLIDAR_ROOT. RPLidar root is ${RPLIDAR_ROOT}, Lib is ${RPLIDAR_LIBS}")
else(NOT RPLIDAR_INCLUDE_DIRS OR NOT RPLIDAR_LIBS)
	set(AADC_RPLIDAR_FOUND TRUE)
	message(STATUS "RPLidar lib found. RPLidar root is ${RPLIDAR_ROOT}, Lib is ${RPLIDAR_LIBS}")
endif(NOT RPLIDAR_INCLUDE_DIRS OR NOT RPLIDAR_LIBS)

#-------------------------------------------------------
#-------ZeroMQ------------------------------------------
#-------------------------------------------------------

if(UNIX)
	set(ZMQ_DIR "/opt/libzmq")
	set(ZMQ_LIBRARY_DIRS "${ZMQ_DIR}/build/lib")
else(UNIX)
	set(ZMQ_DIR "C:/SDK/libzmq")
	set(ZMQ_LIBRARY_DIRS "${ZMQ_DIR}/build/lib/Release")
endif(UNIX)

set(ZMQ_INCLUDE_DIRS "${ZMQ_DIR}/include")

FIND_LIBRARY(ZMQ_LIBS NAMES
		libzmq.a
		libzmq-v140-mt-4_3_1
		PATHS
		"${ZMQ_LIBRARY_DIRS}"
		)

if (ZMQ_LIBS)
	message(STATUS "Found ZeroMQ. ZeroMQ lib dir is: ${ZMQ_LIBRARY_DIRS}")
	set(AADC_ZMQ_FOUND TRUE)
else (ZMQ_LIBS)
	message(FATAL_ERROR "ZeroMQ libs not found under ${ZMQ_LIBRARY_DIRS} (follow instructions in the README!)")
endif (ZMQ_LIBS)

#------------------------------------------------------------------	
#--------Tools for Doc---------------------------------------------
#------------------------------------------------------------------

if(WIN32)
	set(TOOLS_DIR                 "C:/Tools" CACHE PATH "" FORCE)
	set(ADTF_MD5SUM_EXECUTABLE    ${TOOLS_DIR}/cygwin/bin/md5sum.exe CACHE PATH "" FORCE)
	set(DOXYGEN_EXECUTABLE        ${TOOLS_DIR}/doxygen/bin/doxygen.exe CACHE PATH "" FORCE)
	set(DOXYGEN_DOT_EXECUTABLE    ${TOOLS_DIR}/Graphviz/bin CACHE PATH "" FORCE)
	set(MSCGEN_PATH               ${TOOLS_DIR}/Msc-generator CACHE PATH "" FORCE)
	set(HTML_HELP_COMPILER        ${TOOLS_DIR}/HTMLHelpWorkshop/1.3.0/hhc.exe CACHE PATH "" FORCE)
	set(HTML_HELP_INCLUDE_PATH    ${TOOLS_DIR}/HTMLHelpWorkshop/1.3.0/include CACHE PATH "" FORCE)
	set(HTML_HELP_LIBRARY         ${TOOLS_DIR}/HTMLHelpWorkshop/1.3.0/lib/htmlhelp.lib CACHE PATH "" FORCE)
	set(EXACT_MD5SUM_EXECUTABLE   ${TOOLS_DIR}/cygwin/1.7.16/bin/md5sum.exe CACHE PATH "" FORCE)
	set(EXACT_NSIS_EXECUTABLE     ${TOOLS_DIR}/NSIS/2.46.0/makensis.exe CACHE PATH "" FORCE)
endif(WIN32)
#------------------------------------------------------------------
#--------compiler settings-----------------------------------------
#------------------------------------------------------------------

# under windows the size of the pre-compiled-header are limited so we have to extend it manually
if (WIN32)
	# this doesn't work for a MinGW setup
    #SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Zm140" )
else (WIN32)	
    SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wno-unknown-pragmas" )
endif(WIN32)

#------------------------------------------------------------------	
#--------Install Prefix--------------------------------------------
#------------------------------------------------------------------	
#set the install prefix if not already done.
set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../../_install")
if(MSVC10)
	set(CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/win64_vc100" CACHE PATH "The install directory" FORCE)     
elseif(CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/linux64" CACHE PATH "The install directory" FORCE)   
endif()
set(CMAKE_INSTALL_BINARY "${CMAKE_INSTALL_PREFIX}/bin" CACHE PATH "The install directory" FORCE)
set(CMAKE_INSTALL_APPS "${CMAKE_INSTALL_PREFIX}/apps" CACHE PATH "The install directory" FORCE)
set(CMAKE_INSTALL_DOC "${CMAKE_INSTALL_PREFIX}/doc" CACHE PATH "The install directory" FORCE)
set(CMAKE_INSTALL_INCLUDE "${CMAKE_INSTALL_PREFIX}/include" CACHE PATH "The install directory" FORCE)
set(CMAKE_INSTALL_LIBRARY "${CMAKE_INSTALL_PREFIX}/lib" CACHE PATH "The install directory" FORCE)
set(CMAKE_INSTALL_DESCRIPTION "${CMAKE_INSTALL_PREFIX}/description" CACHE PATH "The install directory" FORCE)
set(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT FALSE)
message(STATUS "Install dir is ${CMAKE_INSTALL_PREFIX}")

if (WIN32)
	#enable folders in VS
	set_property(GLOBAL PROPERTY USE_FOLDERS true)
endif(WIN32)
#------------------------------------------------------------------

if(NOT AADC_OPENCV_FOUND OR NOT AADC_BOOST_FOUND OR NOT AADC_PYLON_FOUND)
	message(FATAL_ERROR "At least one of the required libraries is not found")
endif(NOT AADC_OPENCV_FOUND OR NOT AADC_BOOST_FOUND OR NOT AADC_PYLON_FOUND)

