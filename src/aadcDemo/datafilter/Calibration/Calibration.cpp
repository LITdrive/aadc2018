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
#include "Calibration.h"
#include "ADTF3_helper.h"

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CALIBRATION_DATA_TRIGGERED_FILTER,
    "Calibration",
    cCalibration,
    adtf::filter::pin_trigger({ "input_value" }));



cCalibration::cCalibration()
{
    //Register Properties
    RegisterPropertyVariable("Scale Factor", m_scaleFactor);

    //Get Media Descriptions
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
    Register(m_oInputValue, "input_value", pTypeSignalValue);
    Register(m_oOutputValue, "output_value", pTypeSignalValue);
}

tResult cCalibration::Configure()
{
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    RETURN_NOERROR;
}

tResult cCalibration::Process(tTimeStamp tmTimeOfTrigger)
{
    // Setpoint value speed
    object_ptr<const ISample> pInputValueSample;
    //write values with zero
    tFloat32 f32Value = 0;
    tUInt32  Ui32TimeStamp = 0;

    if (IS_OK(m_oInputValue.GetNextSample(pInputValueSample)))
    {

        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pInputValueSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &Ui32TimeStamp));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &f32Value));

        f32Value = (m_scaleFactor * f32Value);

    }

    RETURN_IF_FAILED(transmitSignalValue(m_oOutputValue, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, Ui32TimeStamp, m_ddlSignalValueId.value, f32Value));

    RETURN_NOERROR;
}
