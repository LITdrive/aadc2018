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
#include "LITD_ObjectDetection.h"
#include "ADTF3_OpenCV_helper.h"
#include <boost/thread/thread.hpp>

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_LITD_OBJECT_DETECTION_THREAD_TRIGGERED_FILTER,
                                    "LITD ObjectDetection",
                                    cLITD_ObjectDetection,
                                    adtf::filter::thread_trigger(tTrue));

cLITD_ObjectDetection::cLITD_ObjectDetection()
{
    RegisterPropertyVariable("tensorflow model path", m_model_path);
	RegisterPropertyVariable("sleep time [msec]", m_sleep_time);

    //create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    const adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    object_ptr<IStreamType> pTypeTemplateData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tYOLONetOutput", pTypeTemplateData, m_YNOStructSampleFactory))
    {
        adtf_ddl::access_element::find_array_index(m_YNOStructSampleFactory, cString("f32NodeValue"), m_ddtYOLONetOutputStruct.nodeValues);
    }
    else
    {
        LOG_WARNING("No mediadescription for tYOLONetOutput found!");
    }

    //Register input pin
    Register(m_oReader, "input", pType);
    //Register output pin
    Register(m_oWriterLeft, "left", pTypeTemplateData);
    Register(m_oWriterCenter, "center", pTypeTemplateData);
    Register(m_oWriterRight, "right", pTypeTemplateData);

    //register callback for type changes
    m_oReader.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oReader, m_sImageFormat, *pType.Get());
    });
}

tResult cLITD_ObjectDetection::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    // check file path
    cFilename modelPath = m_model_path;
    adtf::services::ant::adtf_resolve_macros(modelPath);
    if (!cFileSystem::Exists(modelPath))
    {
        RETURN_ERROR_DESC(ERR_INVALID_FILE, cString::Format("YOLO model file could not be loaded from %s", modelPath.GetPtr()));
    }

    // load model
    Status load_graph_status = yolo_handler.load_graph(modelPath.GetPtr());
    if (!load_graph_status.ok()) {
        LOG_ERROR("YOLO Graph Status: %s", load_graph_status.ToString().c_str());
    } else {
        LOG_INFO("Successfully loaded YOLO network from %s", modelPath.GetPtr());
    }

    RETURN_NOERROR;
}

tResult cLITD_ObjectDetection::Process(tTimeStamp tmTimeOfTrigger)
{
    LOG_INFO("forward path called");
    while (!m_runner_reset_signal)
    {
        object_ptr<const ISample> pReadSample;
        Tensor output;
        int size_output_array = 21125;
        tFloat32 left_output_array[size_output_array];
        tFloat32 center_output_array[size_output_array];
        tFloat32 right_output_array[size_output_array];
        int flag = 0;

        if (IS_OK(m_oReader.GetLastSample(pReadSample)))
        {
            object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
            //lock read buffer
            if (IS_OK(pReadSample->Lock(pReadBuffer)))
            {
                //create a opencv matrix from the media sample buffer
                Mat inputImage = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
                                        CV_8UC3, const_cast<unsigned char*>(static_cast<const unsigned char*>(pReadBuffer->GetPtr())));


                //Do the image processing and copy to destination image buffer
                // TODO: use .data() instead
                output = yolo_handler.forward_path(inputImage);
                LOG_INFO("forward path done");
                auto output_flat = output.flat_outer_dims<float, 3>();

                for (int i = 0; i < size_output_array; i++) {
                    left_output_array[i] = output_flat(i);
                    center_output_array[i] = output_flat(i+21125);
                    right_output_array[i] = output_flat(i+42250);
                }
                flag = 1;
            }

            if (flag == 1) {

                object_ptr<ISample> pWriteSampleLeft;
                if (IS_OK(alloc_sample(pWriteSampleLeft)))
                {
                    auto oCodec = m_YNOStructSampleFactory.MakeCodecFor(pWriteSampleLeft);

                    tFloat32 *nodeValues = static_cast<tFloat32*>(oCodec.GetElementAddress(m_ddtYOLONetOutputStruct.nodeValues));
                    memcpy(nodeValues, left_output_array, sizeof left_output_array);
                }
                m_oWriterLeft << pWriteSampleLeft << flush << trigger;

                object_ptr<ISample> pWriteSampleCenter;
                if (IS_OK(alloc_sample(pWriteSampleCenter)))
                {
                    auto oCodec = m_YNOStructSampleFactory.MakeCodecFor(pWriteSampleCenter);

                    tFloat32 *nodeValues = static_cast<tFloat32*>(oCodec.GetElementAddress(m_ddtYOLONetOutputStruct.nodeValues));
                    memcpy(nodeValues, center_output_array, sizeof center_output_array);
                }
                m_oWriterCenter << pWriteSampleCenter << flush << trigger;

                object_ptr<ISample> pWriteSampleRight;
                if (IS_OK(alloc_sample(pWriteSampleRight)))
                {
                    auto oCodec = m_YNOStructSampleFactory.MakeCodecFor(pWriteSampleRight);

                    tFloat32 *nodeValues = static_cast<tFloat32*>(oCodec.GetElementAddress(m_ddtYOLONetOutputStruct.nodeValues));
                    memcpy(nodeValues, right_output_array, sizeof right_output_array);
                }
                m_oWriterRight << pWriteSampleRight << flush << trigger;

            }
        }

        // good night
        boost::this_thread::sleep(boost::posix_time::milliseconds(m_sleep_time));
    }

    RETURN_NOERROR;
}