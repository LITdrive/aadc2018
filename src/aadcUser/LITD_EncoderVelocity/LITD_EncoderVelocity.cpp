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
#include "LITD_EncoderVelocity.h"
#include <ADTF3_helper.h>

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_ENCODER_VELOCITY_FILTER,
	"LITD EncoderVelocity",
	cEncoderVelocity,
	adtf::filter::pin_trigger({"wheel_left"}));


cEncoderVelocity::cEncoderVelocity()
{
    RegisterPropertyVariable("wheel circumference [m]", m_f32wheelCircumference);
    RegisterPropertyVariable("pt1 filter constant", m_f32FilterConstantfirstOrder);
    RegisterPropertyVariable("median filter length", m_i32FilterLength);
    RegisterPropertyVariable("encoder steps per revolution", m_i32StepsPerRevolution);
    RegisterPropertyVariable("false: 0 is forward, true: 1 is forward", m_bForwardDirection);

	// wheel data
    object_ptr<IStreamType> pTypeWheelData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tWheelData", pTypeWheelData, m_WheelDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_WheelDataSampleFactory, "ui32ArduinoTimestamp", m_ddlWheelDataIndex.ArduinoTimestamp);
        adtf_ddl::access_element::find_index(m_WheelDataSampleFactory, "ui32WheelTach", m_ddlWheelDataIndex.WheelTach);
        adtf_ddl::access_element::find_index(m_WheelDataSampleFactory, "i8WheelDir", m_ddlWheelDataIndex.WheelDir);
    }
    else
    {
        LOG_INFO("No mediadescription for tWheelData found!");
    }

    Register(m_oInputWheelLeft, "wheel_left", pTypeWheelData);
    Register(m_oInputWheelRight, "wheel_right", pTypeWheelData);

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
	
    Register(m_oOutputCarSpeed, "vehicle_speed", pTypeSignalValue);
}

tResult cEncoderVelocity::Configure()
{
    RETURN_IF_FAILED(cTriggerFunction::Configure());
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

	m_encoderVelocity = new EncoderVelocity(m_f32wheelCircumference, m_i32StepsPerRevolution, m_bForwardDirection, m_i32FilterLength, m_f32FilterConstantfirstOrder);

    RETURN_NOERROR;
}

tResult cEncoderVelocity::Process(tTimeStamp tmTimeOfTrigger)
{
	object_ptr<const ISample> pSampleFromWheelLeft;

    while (IS_OK(m_oInputWheelLeft.GetNextSample(pSampleFromWheelLeft)))
	{
		tUInt32 tach = 0;
		tInt8 dir = 0;
		tUInt32 timestamp = 0;

		auto oDecoder = m_WheelDataSampleFactory.MakeDecoderFor(*pSampleFromWheelLeft);

		RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.ArduinoTimestamp, &timestamp));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.WheelTach, &tach));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.WheelDir, &dir));

		m_encoderVelocity->ProcessLeftSignal(timestamp, dir, tach);
	}

	object_ptr<const ISample> pSampleFromWheelRight;

	while (IS_OK(m_oInputWheelRight.GetNextSample(pSampleFromWheelRight)))
	{
		tUInt32 tach = 0;
		tInt8 dir = 0;
		tUInt32 timestamp = 0;

		auto oDecoder = m_WheelDataSampleFactory.MakeDecoderFor(*pSampleFromWheelRight);

		RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.ArduinoTimestamp, &timestamp));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.WheelTach, &tach));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.WheelDir, &dir));

		m_encoderVelocity->ProcessRightSignal(timestamp, dir, tach);
	}
	
	tFloat32 f32speed = m_encoderVelocity->GetVelocity();
    tUInt32 ui32arduinoTimestamp = 0;

	RETURN_IF_FAILED(transmitSignalValue(m_oOutputCarSpeed, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, ui32arduinoTimestamp, m_ddlSignalValueId.value, f32speed));

    RETURN_NOERROR;
}
