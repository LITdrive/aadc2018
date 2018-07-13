set(DOC_BUILD_STRING "${CMAKE_PROJECT_NAME}-${PACKAGE_VERSION}-adtf${ADTF_VERSION}")


# Creates target DOC
add_toolbox_doc(${CMAKE_CURRENT_SOURCE_DIR}/../../doc/doxygen/doxygen.cfg ./ TARGET_NAME DOC)
   

install(
	FILES
		change_log.txt
		${CMAKE_CURRENT_BINARY_DIR}/html/${DOC_BUILD_STRING}.chm
	DESTINATION
		${CMAKE_INSTALL_DOC}
	OPTIONAL
)


