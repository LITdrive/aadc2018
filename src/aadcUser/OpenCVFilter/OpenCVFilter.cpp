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

#include <bitset>
#include "stdafx.h"
#include "OpenCVFilter.h"
#include "ADTF3_OpenCV_helper.h"
#include "ADTF3_helper.h"

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_OPENCVFILTER_DATA_TRIGGERED_FILTER,
                                    "OpenCV Filter",
                                    cOpenCVFilter,
                                    adtf::filter::pin_trigger({ "input" }));

cOpenCVFilter::cOpenCVFilter()
{

    //create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    const adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    //register input pin
    Register(m_oReader, "input", pType);
    //Register output pin
    Register(m_oWriter, "output", pType);

    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_inputSignalValueSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_inputSignalValueSampleFactory, "ui32ArduinoTimestamp", m_ddlSignalValueId.timeStamp);
        adtf_ddl::access_element::find_index(m_inputSignalValueSampleFactory, "f32Value", m_ddlSignalValueId.value);
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }

    //register output pins
    Register(m_oSpeedWriter, "speed", pTypeSignalValue);
    Register(m_oSteeringWriter, "steering", pTypeSignalValue);

    //register callback for type changes
    m_oReader.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oReader, m_sImageFormat, *pType.Get());
    });
}

tResult cOpenCVFilter::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    RETURN_NOERROR;
}

tResult cOpenCVFilter::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;
    tFloat speed = 0.0, steering=0.0;
    try{
        while (IS_OK(m_oReader.GetNextSample(pReadSample)))
        {
            object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
            //lock read buffer
            if (IS_OK(pReadSample->Lock(pReadBuffer)))
            {
                //create a opencv matrix from the media sample buffer
                Mat inputImage = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
                                       CV_8UC3, const_cast<unsigned char*>(static_cast<const unsigned char*>(pReadBuffer->GetPtr())));

                int redThreshould = 75;
                //left
                tInt left = 1;
                for(tInt i = 0;i< inputImage.size[0];i++){
                    for(tInt j = 0; j < inputImage.size[1]/2;j++){
                        int r = inputImage.at<unsigned char>(i,j*3);
                        int g = inputImage.at<unsigned char>(i,j*3+1);
                        int b = inputImage.at<unsigned char>(i,j*3+2);
                        if( r > b + g && r > redThreshould)
                            left++;
                    }
                }
                //right
                tInt right = 1;
                for(tInt i = 0;i < inputImage.size[0];i++){
                    for(tInt j = inputImage.size[1]/2; j < inputImage.size[0]; j++){
                        int r = inputImage.at<unsigned char>(i,j*3);
                        int g = inputImage.at<unsigned char>(i,j*3+1);
                        int b = inputImage.at<unsigned char>(i,j*3+2);
                        if( r > b + g && r > redThreshould)
                            right++;
                    }
                }
                speed = (0.025 - (left+right)*1.0/(inputImage.size[0]*inputImage.size[1]))*400;
                if(speed < -10)
                    speed=-10;
                if(speed > 10)
                    speed=10;
                steering = -100 + right*200.0/(left+right);
                //cout << speed;
                //cout << steering;
                //speed = left;
                //steering = right;
            }
            object_ptr<ISample> pSpeedSample;
            object_ptr<ISample> pSteeringSample;

            if (IS_OK(alloc_sample(pSteeringSample)))
            {

                auto oCodec = m_inputSignalValueSampleFactory.MakeCodecFor(pSteeringSample);

                RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, steering));
                RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.timeStamp, m_pClock->GetStreamTime()));

            }

            if (IS_OK(alloc_sample(pSpeedSample)))
            {

                auto oCodec2 = m_inputSignalValueSampleFactory.MakeCodecFor(pSpeedSample);
                RETURN_IF_FAILED(oCodec2.SetElementValue(m_ddlSignalValueId.value, speed));
                RETURN_IF_FAILED(oCodec2.SetElementValue(m_ddlSignalValueId.timeStamp, m_pClock->GetStreamTime()));
            }

            m_oSpeedWriter << pSpeedSample << flush << trigger;
            m_oSteeringWriter << pSteeringSample << flush << trigger;
        }
    }
    catch(const exception& e){
        cout << e.what();
    }

    catch(...){
        cout << "Exception occured";
    }
    //cout << "Call finished";
    RETURN_NOERROR;
}
