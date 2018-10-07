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
#include "FineLocalisation.h"
#include "ADTF3_OpenCV_helper.h"
#include "ADTF3_helper.h"
#include "FineLocator.h"


#define DEG2RAD M_PI/180

ADTF_PLUGIN(LABEL_FINE_LOCALISATION_FILTER, cFineLocalisation)

cFineLocalisation::cFineLocalisation()
{

    //create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    const adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    //Register input pin
    create_pin(*this, m_oReader, "inBirdsEye", pType);

    object_ptr<IStreamType> pTypePositionData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPosition", pTypePositionData, m_PositionSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32x", m_ddlPositionIndex.x);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32y", m_ddlPositionIndex.y);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32radius", m_ddlPositionIndex.radius);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32speed", m_ddlPositionIndex.speed);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32heading", m_ddlPositionIndex.heading);
    }
    else
    {
        LOG_WARNING("No mediadescription for tPosition found!");
    }
    object_ptr<const IStreamType> pConstTypePositionData = pTypePositionData;


    //Register input pin
    create_pin(*this, m_oPosReader, "inPosition", pConstTypePositionData);

    //Register output pin
    filter_create_pin(*this, m_oPosWriter, "outPosition", pConstTypePositionData);

    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value));
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }
    object_ptr<const IStreamType> pConstTypeSignalValue = pTypeSignalValue;

    filter_create_pin(*this, m_oConfWriter, "outConfidence", pConstTypeSignalValue);

    //register callback for type changes
    m_oReader.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oReader, m_sImageFormat, *pType.Get());
    });

    RegisterPropertyVariable("AffineMat@00", mat00);
    RegisterPropertyVariable("AffineMat@01", mat01);
    RegisterPropertyVariable("AffineMat@02", mat02);
    RegisterPropertyVariable("AffineMat@10", mat10);
    RegisterPropertyVariable("AffineMat@11", mat11);
    RegisterPropertyVariable("AffineMat@12", mat12);
    RegisterPropertyVariable("Path to Map", mapPath);
    RegisterPropertyVariable("search space size", propSearchSpaceSize);
    RegisterPropertyVariable("distance from back axle to Picture bottom [m]", axleToPicture);
    RegisterPropertyVariable("offset for Heading [°]", headingOffset);
    RegisterPropertyVariable("Angle Iteration Count", angleIterCnt);
    RegisterPropertyVariable("Angle Range Min [°]", angleRangeMin);
    RegisterPropertyVariable("Angle Range Max [°]", angleRangeMax);
    RegisterPropertyVariable("Image subsample rate", subSampleRate);



    create_inner_pipe(*this, cString::Format("%s_trigger", "inPosition"), "inPosition", [&](tTimeStamp tmTime) -> tResult
    {
        return ProcessPosition(tmTime);
    });

    create_inner_pipe(*this, cString::Format("%s_trigger", "inBirdsEye"), "inBirdsEye", [&](tTimeStamp tmTime) -> tResult
    {
        return ProcessImage(tmTime);
    });
}

tResult cFineLocalisation::Init(const tInitStage eStage)
{
    RETURN_IF_FAILED(cFilter::Init(eStage));
    if (eStage == StageFirst)
    {
        // press "Init"
        RETURN_IF_FAILED(Configure());
    }
    RETURN_NOERROR;
}

tResult cFineLocalisation::Configure()
{
    //TODO transfer data to new thread and start new thread
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    cFilename mapPathResolved = mapPath;
    adtf::services::ant::adtf_resolve_macros(mapPathResolved);
    locator.setMap(const_cast<char*>(mapPathResolved.GetPtr()));
    locator.setAngleSearchSpace(angleRangeMin, angleRangeMax, angleIterCnt);
    affineMat[0][0] = mat00;
    affineMat[0][1] = mat01;
    affineMat[0][2] = mat02;
    affineMat[1][0] = mat10;
    affineMat[1][1] = mat11;
    affineMat[1][2] = mat12;
    locator.setPixelMetricTransformer(PixelMetricTransformer(affineMat));
    locator.setSearchSpace(propSearchSpaceSize);
    sampleCnt = 0;
    RETURN_NOERROR;
}

tResult cFineLocalisation::ProcessPosition(tTimeStamp tmTimeOfTrigger){
    object_ptr<const ISample> pPosReadSample;

    if(IS_OK(m_oPosReader.GetLastSample(pPosReadSample))) {
        auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(*pPosReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.x, &x));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.y, &y));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.heading, &heading));
        //adjust heading for inversion of back camera
        heading = heading + headingOffset*DEG2RAD;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.speed, &speed));
        recievedPosition = true;

    } else {
        LOG_ERROR("!!Failed to read last Position Sample!!");
    }
    RETURN_NOERROR;
}

tResult cFineLocalisation::ProcessImage(tTimeStamp tmTimeOfTrigger){
    object_ptr<const ISample> pReadSample;

    while (IS_OK(m_oReader.GetNextSample(pReadSample)) && recievedPosition)
    {
        if(sampleCnt % subSampleRate == 0) {
            object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
            //lock read buffer
            if (IS_OK(pReadSample->Lock(pReadBuffer))) {
                //-------------------------- Send to new thread ----------------------------------
                //create a opencv matrix from the media sample buffer
                Mat bvImage = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height), CV_8UC3, const_cast<unsigned char *>(static_cast<const unsigned char *>(pReadBuffer->GetPtr())));
                //-------------------------- Start of new thread ----------------------------------
                //[dx, dy, headingOffset, confidence]
                auto start = std::chrono::system_clock::now();
                float *location = locator.localize(bvImage, heading, x, y, axleToPicture);
                auto end = std::chrono::system_clock::now();
                std::chrono::duration<double> diff = end - start;
                LOG_INFO("Localization took %e s", diff.count());
                if(location[3] > angleRangeMax || location[3] < angleRangeMin) LOG_ERROR("caclualted angle offset out of bounds: %f", location[3]);
                object_ptr<ISample> pWriteSample;
                RETURN_IF_FAILED(alloc_sample(pWriteSample, m_pClock->GetStreamTime()));
                {
                    auto oCodec = m_PositionSampleFactory.MakeCodecFor(pWriteSample);

                    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.x, x + location[0]));
                    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.y, y + location[1]));
                    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.heading, heading - headingOffset*DEG2RAD + location[2]));
                    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.speed, speed));
                }
                LOG_INFO("found deltas: %.2f %.2f %.4f",location[0], location[1], location[2]);
                transmitSignalValue(m_oConfWriter, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, location[3]);
                m_oPosWriter << pWriteSample << flush << trigger;

                //-------------------------- End of new thread ----------------------------------
            }
            sampleCnt = 0;
        }
        sampleCnt++;
    }
    RETURN_NOERROR;
}