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
#include <stdafx.h>

using namespace adtf::util;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::system;

using namespace cv;

#include "UniversalCamera.h"
#include "ADTF3_OpenCV_helper.h"


ADTF_PLUGIN(LABEL_UNIVERSAL_CAMERA_STREAMING_SOURCE, cUniversalCamera);

cUniversalCamera::cUniversalCamera()
{
    //Register Properties
    RegisterPropertyVariable("streamWidth", m_streamWidth);
    RegisterPropertyVariable("streamHeight", m_streamHeight);
    RegisterPropertyVariable("streamChannels", m_streamChannels);
    RegisterPropertyVariable("flipHorizontal", m_cameraFlipHorizontal);
    RegisterPropertyVariable("flipVertical", m_cameraFlipVertical);
    RegisterPropertyVariable("ID", m_cameraId);
}

tResult cUniversalCamera::Construct()
{
    RETURN_IF_FAILED(cSampleStreamingSource::Construct());

    //Define stream output format
    object_ptr<IStreamType> pType = make_object_ptr<cStreamType>(stream_meta_type_image());
    set_property(*pType, stream_meta_type_image::FormatName, ADTF_IMAGE_FORMAT(BGR_24));
    set_property(*pType, stream_meta_type_image::PixelWidth, static_cast<tInt>(m_streamWidth));
    set_property(*pType, stream_meta_type_image::PixelHeight, static_cast<tInt>(m_streamHeight));
    set_property(*pType, stream_meta_type_image::MaxByteSize, m_streamWidth * m_streamHeight * m_streamChannels);

    //Register Output Pin
    RETURN_IF_FAILED(create_pin(*this, m_oWriter, "output", pType));

    RETURN_NOERROR;
}

tResult cUniversalCamera::StartStreaming()
{
    RETURN_IF_FAILED(cSampleStreamingSource::StartStreaming());

    tBool propertiesValid = tFalse;

    m_videoCapture.open(m_cameraId);

    if (m_videoCapture.isOpened())
    {
        propertiesValid = m_videoCapture.set(CV_CAP_PROP_FRAME_WIDTH, m_streamWidth);
        propertiesValid = m_videoCapture.set(CV_CAP_PROP_FRAME_HEIGHT, m_streamHeight);
    }
    if (!propertiesValid)
    {
        RETURN_ERROR_DESC(ERR_UNEXPECTED, "Camera capture properties could not be set");
    }

    //check the properties of the capture device and set type accordingly
    m_outputFormat.m_ui32Height = static_cast<tUInt32>(m_videoCapture.get(CV_CAP_PROP_FRAME_HEIGHT));
    m_outputFormat.m_ui32Width = static_cast<tUInt32>(m_videoCapture.get(CV_CAP_PROP_FRAME_WIDTH));
    m_outputFormat.m_strFormatName = ADTF_IMAGE_FORMAT(BGR_24);
    m_outputFormat.m_szMaxByteSize = m_outputFormat.m_ui32Height * m_outputFormat.m_ui32Width * m_streamChannels;

    adtf::ucom::object_ptr<IStreamType> pTypeOutput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeOutput, m_outputFormat);
    m_oWriter << pTypeOutput;

    LOG_WARNING(cString::Format("Image Format set to %i x %i", m_outputFormat.m_ui32Width, m_outputFormat.m_ui32Height));

    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    
    //start thread for grabbing
    m_oThread = kernel_thread_looper(cString(get_named_graph_object_full_name(*this) + "::grabbing"),
        &cUniversalCamera::ThreadFunc, this);
    if (!m_oThread.Joinable())
    {
        RETURN_ERROR_DESC(ERR_UNEXPECTED, "Unable to create read kernel thread");
    }

    RETURN_NOERROR;
}

tResult cUniversalCamera::StopStreaming()
{
    m_oThread = kernel_thread_looper();

    return cSampleStreamingSource::StopStreaming();
}

tVoid cUniversalCamera::ThreadFunc()
{
    // if device is open
    if (m_videoCapture.isOpened())
    {
        Mat cameraFrame;
        try
        {
            m_videoCapture.read(cameraFrame);
        }
        catch (cv::Exception& e)
        {
            const char* err_msg = e.what();
            LOG_ERROR(cString("OpenCV exception caught: ") + err_msg);
        }

        //if we have something to transmit
        if (!cameraFrame.empty())
        {
            cv::Mat flippedImage;
            if (!m_cameraFlipHorizontal && m_cameraFlipVertical)
            {
                cv::flip(cameraFrame, flippedImage, 0);
                cameraFrame = flippedImage;
            }
            else if (m_cameraFlipHorizontal && !m_cameraFlipVertical)
            {
                cv::flip(cameraFrame, flippedImage, 1);
                cameraFrame = flippedImage;
            }
            else if (m_cameraFlipHorizontal && m_cameraFlipVertical)
            {
                cv::flip(cameraFrame, flippedImage, -1);
                cameraFrame = flippedImage;
            }
            
            //update output format if matrix size does not fit to
            if (cameraFrame.total() * cameraFrame.elemSize() != m_outputFormat.m_szMaxByteSize)
            {
                setTypeFromMat(m_oWriter, cameraFrame, true);
            }

            // create and transmit sample to pin
            writeMatToPin(m_oWriter, cameraFrame, m_pClock->GetStreamTime());
        }
    }

}
