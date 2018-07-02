function(add_toolbox_doc CONFIG OUTPUT_DIR)
    find_package(Doxygen REQUIRED)

    if (NOT DOXYGEN_DOT_FOUND)
        message(WARNING "Unable to find the dot tool, please set DOXYGEN_DOT_EXECUTABLE variable accordingly")
    endif (NOT DOXYGEN_DOT_FOUND)

    if(WIN32)
        find_path(MSCGEN_PATH mscgen.exe PATHS ${DOXYGEN_DIR}/bin)
        if(NOT MSCGEN_PATH)
            message(FATAL_ERROR "Unable to find the mscgen tool, please set MSCGEN_PATH variable accordingly")
        endif(NOT MSCGEN_PATH)
    else(WIN32)
        set(MSCGEN_PATH "")
    endif(WIN32)

    if(ADTF_INTERNAL_DOC)
        set(ADTF_EXCLUDE_PRIVATE EXCLUDE_PRIVATE)
        set(ADTF_INTERNAL_DOCS YES)
        message("Internal documentation enabled")
    else(ADTF_INTERNAL_DOC)
        set(ADTF_EXCLUDE_PRIVATE EXCLUDE)
        set(ADTF_INTERNAL_DOCS NO)
    endif(ADTF_INTERNAL_DOC)

    set(REL_OUTPUT_DIR ${OUTPUT_DIR})
    set(OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/${OUTPUT_DIR})
    set(NEW_CONFIG ${CMAKE_CURRENT_BINARY_DIR}/doxygen.cfg)
    configure_file(${CONFIG} ${NEW_CONFIG})

    #add_custom_target(DOC ALL.....
    add_custom_target(DOC ${DOXYGEN_EXECUTABLE} ${NEW_CONFIG}
        DEPENDS ${NEW_CONFIG}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )

    if(WIN32)
        find_package(HTMLHelp REQUIRED)

        #add_custom_command(TARGET DOC POST_BUILD
        #    COMMAND ${HTML_HELP_COMPILER}
        #    ARGS ${REL_OUTPUT_DIR}/index.hhp
        #    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        #)
		
		ADD_CUSTOM_TARGET(CHM_DOC 
			DEPENDS ${DOC_BUILD_STRING}.chm
			)
			
	    set_property(TARGET CHM_DOC PROPERTY FOLDER process)
		
	    ADD_CUSTOM_COMMAND(
	       OUTPUT ${DOC_BUILD_STRING}.chm
	       COMMAND ${CMAKE_COMMAND} -DHTML_HELP_COMPILER=${HTML_HELP_COMPILER} -P ${CMAKE_CURRENT_SOURCE_DIR}/hhc.cmake
	       POST_BUILD
	       WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	       )
		   
    endif(WIN32)

endfunction(add_toolbox_doc CONFIG OUTPUT)

#credits go to http://www.cmake.org/pipermail/cmake/2009-February/027014.html
MACRO (DATE RESULT)
    IF (WIN32)
        EXECUTE_PROCESS(COMMAND "date" "/T" OUTPUT_VARIABLE ${RESULT})
        string(REGEX REPLACE "(..)/(..)/..(..).*" "\\1\\2\\3"
${RESULT} ${${RESULT}})
    ELSEIF(UNIX)
        EXECUTE_PROCESS(COMMAND "date" "+%d/%m/%Y" OUTPUT_VARIABLE ${RESULT})
        string(REGEX REPLACE "(..)/(..)/..(..).*" "\\1\\2\\3"
${RESULT} ${${RESULT}})
    ELSE (WIN32)
        MESSAGE(SEND_ERROR "date not implemented")
        SET(${RESULT} 000000)
    ENDIF (WIN32)
ENDMACRO (DATE)

MACRO(a_post_build_install NAME)
        add_custom_command(TARGET ${NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -DCMAKE_INSTALL_CONFIG_NAME=${CMAKE_CFG_INTDIR} -P cmake_install.cmake)
ENDMACRO(a_post_build_install NAME)
