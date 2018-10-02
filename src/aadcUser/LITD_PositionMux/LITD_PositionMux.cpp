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
#include "LITD_PositionMux.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_POSITIONMUX_DATA_TRIGGERED_FILTER,
    "LITD_PositionMux",
    cLITD_PositionMux,
    adtf::filter::pin_trigger({"in_pos0", "in_pos1", "in_pos2", "in_mux"}));


cLITD_PositionMux::cLITD_PositionMux()
{
    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory)) {
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value));
    } else {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }

    Register(m_oMuxReader,  "in_mux" , pTypeSignalValue);

    object_ptr<IStreamType> pTypePositionData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPosition", pTypePositionData, m_PositionSampleFactory)) {
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32x", m_ddlPositionIndex.x);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32y", m_ddlPositionIndex.y);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32radius", m_ddlPositionIndex.radius);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32speed", m_ddlPositionIndex.speed);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32heading", m_ddlPositionIndex.heading);
    } else {
        LOG_WARNING("No mediadescription for tPosition found!");
    }

    Register(m_oPosReader0, "in_pos0", pTypePositionData);
    Register(m_oPosReader1, "in_pos1", pTypePositionData);
    Register(m_oPosReader2, "in_pos2", pTypePositionData);
    Register(m_oPosWriter,  "out_pos", pTypePositionData);


    RegisterPropertyVariable("Default Mux State", defaultMuxState);
}


//implement the Configure function to read ALL Properties
tResult cLITD_PositionMux::Configure()
{
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}

tResult cLITD_PositionMux::sendPositionSample(float x, float y, float heading, float speed, float radius){
    object_ptr<ISample> pWriteSample;
    RETURN_IF_FAILED(alloc_sample(pWriteSample, m_pClock->GetStreamTime()));
    {
        auto oCodec = m_PositionSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.x, x));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.y, y));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.heading, heading));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.speed, speed));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.radius, radius));
    }
    m_oPosWriter << pWriteSample << flush << trigger;

    RETURN_NOERROR;
}

tResult cLITD_PositionMux::Process(tTimeStamp tmTimeOfTrigger)
{

    object_ptr<const ISample> pReadSample, pPosReadSample0, pPosReadSample1, pPosReadSample2;

    while (IS_OK(m_oMuxReader.GetNextSample(pReadSample)))
    {
        // store speed
        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pReadSample);
        tFloat32 f32MuxState = adtf_ddl::access_element::get_value(oDecoder, m_ddlSignalValueId.value);
        muxState = int(f32MuxState);
    }
    
    tFloat32 x, y, heading, speed, radius;
    while(IS_OK(m_oPosReader0.GetNextSample(pPosReadSample0))) {
        if(muxState != 0) continue;     // Skip if not current output
        auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(*pPosReadSample0);

        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.x, &x));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.y, &y));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.heading, &heading));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.speed, &speed));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.radius, &radius));

        sendPositionSample(x, y, heading, speed, radius);
    }

    while(IS_OK(m_oPosReader1.GetNextSample(pPosReadSample1))) {
        if(muxState != 1) continue;     // Skip if not current output
        auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(*pPosReadSample1);

        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.x, &x));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.y, &y));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.heading, &heading));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.speed, &speed));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.radius, &radius));

        sendPositionSample(x, y, heading, speed, radius);
    }

    while(IS_OK(m_oPosReader2.GetNextSample(pPosReadSample2))) {
        if(muxState != 2) continue;     // Skip if not current output
        auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(*pPosReadSample2);

        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.x, &x));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.y, &y));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.heading, &heading));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.speed, &speed));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.radius, &radius));

        sendPositionSample(x, y, heading, speed, radius);
    }

    RETURN_NOERROR;
}