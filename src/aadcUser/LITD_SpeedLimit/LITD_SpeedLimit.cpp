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
#include "LITD_SpeedLimit.h"
#include <ADTF3_helper.h>


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER,
    "LITD Speed Limit",
    cSpeedLimit,
    adtf::filter::pin_trigger({"input"}));


cSpeedLimit::cSpeedLimit()
{
    RegisterPropertyVariable("MAX speed allowed", maxspeed);
    RegisterPropertyVariable("Gain factor", gainfactor);
    RegisterPropertyVariable("Activity Signal Threshold", active_property);

    //DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description
    object_ptr<IStreamType> pTypeTemplateData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeTemplateData, m_templateDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_templateDataSampleFactory, cString("f32Value"), m_ddlSignalValueId.value);
    }
    else
    {
        LOG_WARNING("No mediadescription for tTemplateData found!");
    }

    object_ptr<IStreamType> pTypeBoolSignalValue;
	if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", pTypeBoolSignalValue, m_BoolSignalValueSampleFactory))
	{
		access_element::find_index(m_BoolSignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlBoolSignalValueId.ui32ArduinoTimestamp);
		access_element::find_index(m_BoolSignalValueSampleFactory, cString("bValue"), m_ddlBoolSignalValueId.bValue);
	}
	else
	{
		LOG_INFO("No mediadescription for tBoolSignalValue found!");
	}

    Register(m_oReader, "input" , pTypeTemplateData);
    Register(m_oWriter, "output", pTypeTemplateData);

    
	// update signal
	Register(m_active_signal, "update", pTypeBoolSignalValue);

}


//implement the Configure function to read ALL Properties
tResult cSpeedLimit::Configure()
{
    speed_config = maxspeed;
    RETURN_NOERROR;
}

tResult cSpeedLimit::Process(tTimeStamp tmTimeOfTrigger)
{

    object_ptr<const ISample> pReadSample;

    tFloat32 inputData;

    if (IS_OK(m_oReader.GetLastSample(pReadSample)))
    {
        auto oDecoder = m_templateDataSampleFactory.MakeDecoderFor(*pReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &inputData));

    }

    // Do the Processing
    tFloat32 outputData = inputData * gainfactor;
    if (outputData > speed_config)
     {
         outputData = speed_config;
         LOG_INFO("SPEED LIMIT IS %f",speed_config);
     }
      else if(outputData < -speed_config)
      {
         outputData = -speed_config;
         LOG_INFO("SPEED LIMIT REVERSE IS %f",-speed_config);
      }

    object_ptr<ISample> pWriteSample;

    if (IS_OK(alloc_sample(pWriteSample)))
    {

        auto oCodec = m_templateDataSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, outputData));

    }
    m_oWriter << pWriteSample << flush << trigger;


    // activity signal
    tBool boolVal = fabs(outputData) >= active_property;
    transmitBoolSignalValue(m_active_signal, tmTimeOfTrigger, m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, boolVal);

    RETURN_NOERROR;
}
