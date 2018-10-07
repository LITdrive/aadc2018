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
#include "LITD_SpeedInjector.h"
#include <aadc_structs.h>


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_SPEED_INJECTOR_FILTER,
	"LITD SpeedInjector",
	cSpeedInjector,
	adtf::filter::pin_trigger({ "position" }));


cSpeedInjector::cSpeedInjector()
{
	/* tPosition */
	object_ptr<IStreamType> pTypePositionData;
	if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPosition", pTypePositionData, m_PositionSampleFactory))
	{
		access_element::find_index(m_PositionSampleFactory, cString("f32x"), m_ddlPositionIndex.f32x);
		access_element::find_index(m_PositionSampleFactory, cString("f32y"), m_ddlPositionIndex.f32y);
		access_element::find_index(m_PositionSampleFactory, cString("f32radius"), m_ddlPositionIndex.f32radius);
		access_element::find_index(m_PositionSampleFactory, cString("f32speed"), m_ddlPositionIndex.f32speed);
		access_element::find_index(m_PositionSampleFactory, cString("f32heading"), m_ddlPositionIndex.f32heading);
	}
	else
	{
		LOG_WARNING("No mediadescription for tPosition found!");
	}

	/* tSignalValue */
	object_ptr<IStreamType> pTypeSignalValue;
	if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory))
	{
		access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp);
		access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value);
	}
	else
	{
		LOG_WARNING("No mediadescription for tSignalValue found!");
	}

	Register(m_SpeedReader, "speed", pTypePositionData);
	Register(m_PositionReader, "position", pTypePositionData);
	Register(m_PositionWriter, "position_injected", pTypePositionData);
}

tResult cSpeedInjector::Configure()
{
	RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
	RETURN_NOERROR;
}

tResult cSpeedInjector::Process(tTimeStamp tmTimeOfTrigger)
{
	object_ptr<const ISample> pReadSampleSpeed;

	tFloat32 speed_to_inject = 0;
	tBool inject = false;
	if (IS_OK(m_PositionReader.GetLastSample(pReadSampleSpeed)))
	{
		// read latest speed sample
		auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(*pReadSampleSpeed);
		RETURN_IF_FAILED(oDecoder.IsValid());
		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &speed_to_inject));

		inject = true;
	}

	object_ptr<const ISample> pReadSamplePos;

	while (IS_OK(m_PositionReader.GetNextSample(pReadSamplePos)))
	{
		// read position
		::tPosition position;
		auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(*pReadSamplePos);
		RETURN_IF_FAILED(oDecoder.IsValid());

		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.f32x, &position.f32x));
		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.f32y, &position.f32y));
		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.f32heading, &position.f32heading));
		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.f32radius, &position.f32radius));
		RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.f32speed, &position.f32speed));

		// overwrite speed if available
		if (inject)
		{
			position.f32speed = speed_to_inject;
		}

		// write position
		object_ptr<ISample> pWriteSample;
		RETURN_IF_FAILED(alloc_sample(pWriteSample, m_pClock->GetStreamTime()));
		{
			auto oCodec = m_PositionSampleFactory.MakeCodecFor(pWriteSample);

			RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.f32x, position.f32x));
			RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.f32y, position.f32y));
			RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.f32heading, position.f32heading));
			RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.f32radius, position.f32radius));
			RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.f32speed, position.f32speed));
		}

		m_PositionWriter << pWriteSample << flush << trigger;
	}

	RETURN_NOERROR;
}