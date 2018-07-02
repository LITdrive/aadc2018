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

#include "stdafx.h"

#include <aadc_structs.h>


using namespace adtf::util;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::system;

#include "LaserScanner.h"

using namespace rp::standalone::rplidar;

ADTF_PLUGIN(LABEL_LASER_SCANNER_STREAMING_SOURCE, cLaserScanner);

cLaserScanner::cLaserScanner()
{
    //Register Properties
    RegisterPropertyVariable("serial_device", m_propSerialDevice);
}

tResult cLaserScanner::Construct()
{
    RETURN_IF_FAILED(cSampleStreamingSource::Construct());

    object_ptr<IStreamType> pTypePolarCoord;
    if (ERR_NOERROR == adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPolarCoordiante", pTypePolarCoord, m_polarCoordinateSampleFactory))
    {
        //// find the indexes of the element for faster access in the process method.
        LogNamedMessage("Found mediadescription for tPolarCoordiante!");
        //get all the member indices

        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_polarCoordinateSampleFactory, "f32Radius", m_ddlPolarCoordinateId.radius));
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_polarCoordinateSampleFactory, "f32Angle", m_ddlPolarCoordinateId.angle));
    }
    else
    {
        LOG_INFO("No mediadescription for tPolarCoordiante found!");
    }
    object_ptr<IStreamType> pTypeLSData;
    if (ERR_NOERROR == adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tLaserScannerData", pTypeLSData, m_LSStructSampleFactory))
    {
        //// find the indexes of the element for faster access in the process method.
        LogNamedMessage("Found mediadescription for tLaserScannerData!");
        //get all the member indices
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_LSStructSampleFactory, "ui32Size", m_ddlLSDataId.size));
        RETURN_IF_FAILED(adtf_ddl::access_element::find_array_index(m_LSStructSampleFactory, "tScanArray", m_ddlLSDataId.scanArray));
    }
    else
    {
        LOG_INFO("No mediadescription for tLaserScannerData found!");
    }

    //Register Output Pin
    RETURN_IF_FAILED(create_pin(*this, m_oOut, "output", pTypeLSData));

    RETURN_NOERROR;
}

tResult cLaserScanner::Init()
{
    RETURN_IF_FAILED(cSampleStreamingSource::Init());
    
    //we init the lidar here to check if it is working
    RETURN_IF_FAILED(InitLidar());

    RETURN_IF_FAILED(DeinitLidar());

    RETURN_NOERROR;
}

tResult cLaserScanner::Shutdown()
{
    
    return cSampleStreamingSource::Shutdown();
}

tResult cLaserScanner::StartStreaming()
{
    RETURN_IF_FAILED(InitLidar());
    
    //start the timer to grab the scans
    RETURN_IF_FAILED(cSampleStreamingSource::StartStreaming());

    //start the scanner
    u_result res = m_pdrv->startMotor();
    if (res != RESULT_OK)
    {
        RETURN_ERROR_DESC(ERR_BAD_DEVICE, cString("Could not start RPLidar"));
    }

    //set usetypical scan for double outputrate
    res = m_pdrv->startScan(true, true);
    if (res != RESULT_OK)
    {
        RETURN_ERROR_DESC(ERR_BAD_DEVICE, cString("Could not start RPLidar"));
    }

    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    m_oThread = kernel_thread_looper(cString(get_named_graph_object_full_name(*this) + "::grabbing"),
                            &cLaserScanner::ThreadFunc, this);
    if (!m_oThread.Joinable())
    {
        RETURN_ERROR_DESC(ERR_UNEXPECTED, "Unable to create read kernel thread");
    }


    RETURN_NOERROR;
}

tResult cLaserScanner::StopStreaming()
{
    m_oThread = kernel_thread_looper();

    m_pdrv->stop();
    m_pdrv->stopMotor();

    RETURN_IF_FAILED(DeinitLidar());

    return cSampleStreamingSource::StopStreaming();
}

tResult cLaserScanner::GetScanData(std::vector<tPolarCoordiante>& scan)
{
    //set max count
    size_t count = LASER_SCANNER_COUNT;
    if (m_pdrv->grabScanData(m_nodes, count) == RESULT_OK)
    {
        //now count holds the actual received scan points
        //reorder the scan data
        u_result res = m_pdrv->ascendScanData(m_nodes, count);
        if (res == RESULT_OK)
        {
            tPolarCoordiante scanPoint;
            for (int i = static_cast<int>(count) -1; i >= 0; i--)
            {
                //convert angle to degree               
                scanPoint.f32Angle = (m_nodes[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
                //convert radius to mm
                scanPoint.f32Radius = m_nodes[i].distance_q2 / 4.0f;
                //only take +-90 deg in forward direction
                if (scanPoint.f32Angle <= LASER_SCANNER_CUT_ANGLE_MAX || scanPoint.f32Angle >= LASER_SCANNER_CUT_ANGLE_MIN)
                {
                    //invert angle because scanner is mounted upside down
                    scanPoint.f32Angle =  360.0f - scanPoint.f32Angle;
                    scan.push_back(scanPoint);
                }
                //LOG_INFO(cString::Format("received scan data at index: %d/%d, angle: %f, dist: %f, is %s out of range, old angle %f", i, count, scanPoint.f32Angle, scanPoint.f32Radius, outOfRange ? "" : " not ", angleOld));
            }
        }
        else
        {
            RETURN_ERROR(ERR_FAILED);
        }
    }
    else
    {
        RETURN_ERROR(ERR_DEVICE_NOT_READY);
    }
    RETURN_NOERROR;
}

tResult cLaserScanner::InitLidar()
{
    m_pdrv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    RETURN_IF_POINTER_NULL(m_pdrv);

    u_result res = m_pdrv->connect(cString(m_propSerialDevice).GetPtr(), static_cast<uint32_t>(m_propBaudRate));
    if (res != RESULT_OK)
    {
        LOG_ERROR(cString::Format("Error in rplidar::connect, ErrCode: 0x%08x", res));
        RETURN_ERROR_DESC(ERR_BAD_DEVICE, cString("Could not connect RPLidar"));
    }
    else
    {
        rplidar_response_device_info_t devinfo;
        res = m_pdrv->getDeviceInfo(devinfo);

        if (res != RESULT_OK)
        {
            LOG_ERROR(cString::Format("Error in rplidar::connect, ErrCode: 0x%08x", res));
            RETURN_ERROR_DESC(ERR_BAD_DEVICE, cString("Could not connect RPLidar"));
        }

        rplidar_response_device_health_t healthinfo;
        res = m_pdrv->getHealth(healthinfo);

        if (res != RESULT_OK)
        {
            LOG_ERROR(cString::Format("Error in rplidar::connect, %d\n", healthinfo.status));
            RETURN_ERROR_DESC(ERR_BAD_DEVICE, cString("Could not connect RPLidar"));
        }
    }

    RETURN_NOERROR;
}

tResult cLaserScanner::DeinitLidar()
{
    RPlidarDriver::DisposeDriver(m_pdrv);

    RETURN_NOERROR;
}

tVoid cLaserScanner::ThreadFunc()
{
    std::vector<tPolarCoordiante> scan;
    GetScanData(scan);

    Transmit(scan);
}

tResult cLaserScanner::Transmit(const std::vector<tPolarCoordiante>& scan)
{
    object_ptr<ISample> pSample;
    RETURN_IF_FAILED(alloc_sample(pSample, m_pClock->GetStreamTime()))
    {
        {
            auto oCodec = m_LSStructSampleFactory.MakeCodecFor(pSample);

            oCodec.SetElementValue(m_ddlLSDataId.size, scan.size());

            tPolarCoordiante* pCoordinates = reinterpret_cast<tPolarCoordiante*>(oCodec.GetElementAddress(m_ddlLSDataId.scanArray));

            //init array with zeros
            memset(pCoordinates, 0, LASER_SCANNER_COUNT * sizeof(tPolarCoordiante));

            tUInt32 nIdx = 0;
            for (const tPolarCoordiante& pPolarCoordinate : scan)
            {
                pCoordinates[nIdx].f32Angle = pPolarCoordinate.f32Angle;
                pCoordinates[nIdx++].f32Radius = pPolarCoordinate.f32Radius;
            }

        }
    }

    m_oOut << pSample << flush << trigger;

    RETURN_NOERROR;
}
