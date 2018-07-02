/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

#pragma once
#include "stdafx.h"

#define CID_LASER_SCANNER_STREAMING_SOURCE "laser_scanner.streaming_source.demo.aadc.cid"
#define LABEL_LASER_SCANNER_STREAMING_SOURCE "Laser Scanner"

#if WIN32 //check all com ports...
#define SERIAL_DEVICE_DEFAULT "\\.\com31"
#else
#define SERIAL_DEVICE_DEFAULT "/dev/ttyUSB0"
#endif

/*! this is the main class for the laserscanner module */
class cLaserScanner : public adtf::streaming::cSampleStreamingSource
{
public:
    ADTF_CLASS_ID_NAME(cLaserScanner,
                       CID_LASER_SCANNER_STREAMING_SOURCE,
                       LABEL_LASER_SCANNER_STREAMING_SOURCE);

    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::services::IReferenceClock), REQUIRE_INTERFACE(adtf::services::IKernel));

private:
 
    /*! The property baud rate */
    property_variable<tInt64> m_propBaudRate = 115200;
    /*! The property serial device */
    property_variable<cString> m_propSerialDevice = cString(SERIAL_DEVICE_DEFAULT);
    
    /*! The output writer */
    cSampleWriter m_oOut;

    /*! the clock. */
    object_ptr<adtf::services::IReferenceClock> m_pClock;
    
    /*! The grabbing thread */
    adtf::system::kernel_thread_looper m_oThread;

    /*! The lidar driver */
    RPlidarDriver * m_pdrv;

    /*! the lidar measurements */
    rplidar_response_measurement_node_t m_nodes[LASER_SCANNER_COUNT];
public:
    /*! Default constructor. */
    cLaserScanner();

    tResult Construct() override;
    tResult Init() override;
    tResult Shutdown() override;
    tResult StartStreaming() override;
    tResult StopStreaming() override;

private:

    /*!
     * Gets scan data.
     *
     * \param [in,out]  scan    The scan.
     *
     * \return  Standard Result Code.
     */
    tResult GetScanData(std::vector<tPolarCoordiante>& scan);

    /*!
     * Initializes the lidar.
     *
     * \return  Standard Result Code.
     */
    tResult InitLidar();

    /*!
     * Deinit lidar.
     *
     * \return  Standard Result Code.
     */
    tResult DeinitLidar();

    /*!
     * Timer function.
     *
     * \return  Standard Result Code.
     */
    tVoid ThreadFunc();

    /*!
     * Transmits the given scan.
     *
     * \param   scan    The scan.
     *
     * \return  Standard Result Code.
     */
    tResult Transmit(const std::vector<tPolarCoordiante>& scan);

    /*!
     * Creates a new frame.
     *
     * \return  The new new frame.
     */
    object_ptr<ISample> CreateNewFrame();

    /*! A ddl polar coordinate identifier. */
    struct ddlPolarCoordinateId
    {
        tSize radius;
        tSize angle;
    } m_ddlPolarCoordinateId;

    /*! The polar coordinate sample factory */
    adtf::mediadescription::cSampleCodecFactory m_polarCoordinateSampleFactory;

    /*! A ddl laser scanner data identifier. */
    struct ddlLaserScannerDataId
    {
        tSize size;
        tSize scanArray;
    } m_ddlLSDataId;

    /*! The ls structure sample factory */
    adtf::mediadescription::cSampleCodecFactory m_LSStructSampleFactory;

};