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
#define CID_UNIVERSAL_CAMERA_STREAMING_SOURCE "universal_camera.streaming_source.demo.aadc.cid"
#define LABEL_UNIVERSAL_CAMERA_STREAMING_SOURCE "Universal Camera"


/*! This is the main class for an universal camera. */
class cUniversalCamera : public adtf::streaming::cSampleStreamingSource
{
public:
    ADTF_CLASS_ID_NAME(cUniversalCamera,
        CID_UNIVERSAL_CAMERA_STREAMING_SOURCE,
        LABEL_UNIVERSAL_CAMERA_STREAMING_SOURCE);

    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::services::IReferenceClock), REQUIRE_INTERFACE(adtf::services::IKernel));

private:

    /*! Width of the stream */
    property_variable<tInt>   m_streamWidth = 1024;
    /*! Height of the stream */
    property_variable<tInt>   m_streamHeight = 768;
    /*! The stream channels */
    property_variable<tInt>   m_streamChannels = 3;
    /*! Identifier for the camera */
    property_variable<tInt>   m_cameraId = 0;
    /*! The camera flip horizontal */
    property_variable<tBool>  m_cameraFlipHorizontal = false;
    /*! The camera flip vertical */
    property_variable<tBool>  m_cameraFlipVertical = false;

    /*! output writer */
    cSampleWriter m_oWriter;

    /*! The grabbing thread */
    adtf::system::kernel_thread_looper m_oThread;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    /*! The video capture device */
    cv::VideoCapture m_videoCapture;

    /*! The output image format */
    adtf::streaming::tStreamImageFormat m_outputFormat;
public:
    cUniversalCamera();

    tResult Construct() override;
    tResult StartStreaming() override;
    tResult StopStreaming() override;

private:

    /*!
     * Thread function.
     *
     * \return  A tVoid.
     */
    tVoid ThreadFunc();


};