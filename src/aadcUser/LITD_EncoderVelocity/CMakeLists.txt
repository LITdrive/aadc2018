cmake_minimum_required(VERSION 3.10.0)

project(litd_encoder_velocity)

set (SOURCEFILES    	stdafx.h 
						LITD_EncoderVelocity.h
                        LITD_EncoderVelocity.cpp
                        EncoderVelocity.h
						EncoderVelocity.cpp)

adtf_add_filter(${PROJECT_NAME}
				${SOURCEFILES}
)

# Specify where it should be installed to
adtf_install_target(${PROJECT_NAME} bin)

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
 )

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER user)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER user)
