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


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER,
    "LITD_SpeedLimit_cf",
    cLITD_SpeedLimit,
    adtf::filter::pin_trigger({"input"}));


cLITD_SpeedLimit::cLITD_SpeedLimit()
{    //Get Media Descriptions
    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_signalDataSampleFactory))
    {
        //(adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp));
        (adtf_ddl::access_element::find_index(m_signalDataSampleFactory, cString("f32Value"), m_ddlSignalValueId.value));
        LOG_INFO("constructor reachedd");
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }
    LOG_INFO("registering pin reached");
    //Register(m_oInputMeasWheelsteer, "input steer", pTypeSignalValue);
    //Register(m_oInputMeasWheelSpeed, "input speed", pTypeSignalValue);
    //Register(m_oOutputWheelSpeed, "output speed", pTypeSignalValue);
    //Register(m_oOutputWheelsteer, "output steer", pTypeSignalValue);
    LOG_INFO("registering pin done");
}


//implement the Configure function to read ALL Properties
tResult cLITD_SpeedLimit::Configure()
{
    LOG_INFO("Configuration reached");
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    LOG_INFO("Configuration done");
    RETURN_NOERROR;

}

tResult cLITD_SpeedLimit::Process(tTimeStamp tmTimeOfTrigger)
{

    LOG_INFO("process reached");


    RETURN_NOERROR;
}
