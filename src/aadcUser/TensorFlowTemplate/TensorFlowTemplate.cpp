﻿/*********************************************************************
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
#include "TensorFlowTemplate.h"
#include "ADTF3_OpenCV_helper.h"

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_COPENCVTEMPLATE_DATA_TRIGGERED_FILTER,
                                    "TensorFlowTemplate_cf",
                                    cTensorFlowTemplate,
                                    adtf::filter::pin_trigger({ "input" }));

cTensorFlowTemplate::cTensorFlowTemplate()
{
    //create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    const adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    //Register input pin
    Register(m_oReader, "input", pType);
    //Register output pin
    Register(m_oWriter, "output", pType);

    //register callback for type changes
    m_oReader.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
                                    {
                                        return ChangeType(m_oReader, m_sImageFormat, *pType.Get(), m_oWriter);
                                    });

}

tResult cTensorFlowTemplate::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    RETURN_NOERROR;
}

tResult cTensorFlowTemplate::Process(tTimeStamp tmTimeOfTrigger)
{

    object_ptr<const ISample> pReadSample;
    Tensor tensorImage;

    while (IS_OK(m_oReader.GetNextSample(pReadSample)))
    {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer)))
        {
            //create a opencv matrix from the media sample buffer
            Mat inputImage = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
                                 CV_8UC3, const_cast<unsigned char*>(static_cast<const unsigned char*>(pReadBuffer->GetPtr())));

            tensorImage = this->ConvertToTensor(inputImage, m_sImageFormat.m_ui32Height, m_sImageFormat.m_ui32Width);
        }
    }

    RETURN_NOERROR;
}

Tensor cTensorFlowTemplate::ConvertToTensor(Mat cameraImg, int inputHeight, int inputWidth)
{
    Tensor inputImg(tensorflow::DT_FLOAT, tensorflow::TensorShape({1,inputHeight,inputWidth,3}));
    auto inputImageMapped = inputImg.tensor<float, 4>();
    auto start = std::chrono::system_clock::now();

    //Copy all the data over
    for (int y = 0; y < inputHeight; ++y) {
        const float* source_row = ((float*)cameraImg.data) + (y * inputWidth * 3);
        for (int x = 0; x < inputWidth; ++x) {
            const float* source_pixel = source_row + (x * 3);
            inputImageMapped(0, y, x, 0) = source_pixel[2];
            inputImageMapped(0, y, x, 1) = source_pixel[1];
            inputImageMapped(0, y, x, 2) = source_pixel[0];
        }
    }
    auto end = std::chrono::system_clock::now();

    return inputImg;
}