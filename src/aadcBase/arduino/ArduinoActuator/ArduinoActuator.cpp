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
#include <adtf_systemsdk.h>


using namespace adtf::util;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::system;

#include "ArduinoActuator.h"

#define CONSOLE_LOG(_text, _log_level) if (m_propEnableConsoleOutput) { LOG_ADD_ENTRY(_log_level, _text); }    //!< enables log function if console output is activated
#define CONSOLE_LOG_INFO(_text)      CONSOLE_LOG(_text, A_UTILS_NS::log::tLogLevel::Info)                        //!< log info messages

ADTF_PLUGIN("Arduino Actuator",
    cArduinoActuator);

cArduinoActuator::cArduinoActuator() : cSampleStreamingSink(),
m_serialDevicePrefix(SERIAL_DEVICE_PREFIX),
m_arduinoID(ARDUINO_CENTER_ACTUATORS),
m_startupTime(10e6 /*us = 10s*/),
m_lightMask(0)
{
    RegisterPropertyVariable("Enable console log", m_propEnableConsoleOutput);

    m_mapLightMaskToReaderMap[ID_ARD_ACT_LIGHT_MASK_HEAD] = &m_readerHeadLight;
    m_mapLightMaskToReaderMap[ID_ARD_ACT_LIGHT_MASK_BRAKE] = &m_readerBreakLight;
    m_mapLightMaskToReaderMap[ID_ARD_ACT_LIGHT_MASK_TURNLEFT] = &m_readerTurnSignalLeft;
    m_mapLightMaskToReaderMap[ID_ARD_ACT_LIGHT_MASK_TURNRIGHT] = &m_readerTurnSignalRight;
    m_mapLightMaskToReaderMap[ID_ARD_ACT_LIGHT_MASK_HAZARD] = &m_readerHazardLight;
    m_mapLightMaskToReaderMap[ID_ARD_ACT_LIGHT_MASK_REVERSE] = &m_readerReverseLight;
}


tResult cArduinoActuator::Construct()
{
    RETURN_IF_FAILED(cSampleStreamingSink::Construct());


    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_inputSignalValueSampleFactory))
    {
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_inputSignalValueSampleFactory, "ui32ArduinoTimestamp", m_ddlSignalValueId.timeStamp));
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_inputSignalValueSampleFactory, "f32Value", m_ddlSignalValueId.value));
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }
    RETURN_IF_FAILED(create_pin(*this, m_readerSteering, "steering", pTypeSignalValue));
    RETURN_IF_FAILED(create_pin(*this, m_readerSpeed, "speed", pTypeSignalValue));


    object_ptr<IStreamType> pTypeBoolSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", pTypeBoolSignalValue, m_inputBoolSignalValueSampleFactory))
    {
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_inputBoolSignalValueSampleFactory, "ui32ArduinoTimestamp", m_ddlBoolSignalValueId.timeStamp));
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_inputBoolSignalValueSampleFactory, "bValue", m_ddlBoolSignalValueId.value));
    }
    else
    {
        LOG_INFO("No mediadescription for tBoolSignalValue found!");
    }
    RETURN_IF_FAILED(create_pin(*this, m_readerHeadLight, "head_light", pTypeBoolSignalValue));
    RETURN_IF_FAILED(create_pin(*this, m_readerTurnSignalLeft, "turn_signal_left", pTypeBoolSignalValue));
    RETURN_IF_FAILED(create_pin(*this, m_readerTurnSignalRight, "turn_signal_right", pTypeBoolSignalValue));
    RETURN_IF_FAILED(create_pin(*this, m_readerBreakLight, "brake_light", pTypeBoolSignalValue));
    RETURN_IF_FAILED(create_pin(*this, m_readerHazardLight, "hazard_light", pTypeBoolSignalValue));
    RETURN_IF_FAILED(create_pin(*this, m_readerReverseLight, "reverse_light", pTypeBoolSignalValue));

    RETURN_NOERROR;
}


tResult cArduinoActuator::Destruct()
{
    return cSampleStreamingSink::Destruct();
}


tResult cArduinoActuator::Init()
{
    RETURN_IF_FAILED(cSampleStreamingSink::Init());

    if (m_serialDevice.init(m_arduinoID, m_serialDevicePrefix, NUM_ARDUINO))
    {
        LOG_INFO(cString::Format("Connected to arduino on port: %d\t ID: %d\t Software version: %d",
            m_serialDevice.get_port_num(), m_serialDevice.get_id(), m_serialDevice.get_software_version()));
    }
    else
    {
        LOG_ERROR(cString::Format("Could not find an arduino with correct id: %d ", m_arduinoID));
        RETURN_ERROR_DESC(ERR_DEVICE_IO, cString::Format("Could not find an arduino with correct id: %d ", m_arduinoID));

    }
    RETURN_NOERROR;
}

tResult cArduinoActuator::Shutdown()
{
    m_serialDevice.stop_reading();
    m_serialDevice.end();
    return cSampleStreamingSink::Shutdown();
}

tResult cArduinoActuator::StartStreaming()
{
    RETURN_IF_FAILED(cSampleStreamingSink::StartStreaming());

    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    m_oTimer = kernel_timer(cString(get_named_graph_object_full_name(*this) + "::generation_timer"),
        m_nFrameDelay, 0, &cArduinoActuator::TimerFunc, this);

    if (!m_oTimer.Stoppable())
    {
        RETURN_ERROR_DESC(ERR_UNEXPECTED, "Unable to create kernel timer");
    }

    RETURN_NOERROR;
}

tResult cArduinoActuator::StopStreaming()
{
    m_oTimer.Stop();

    return cSampleStreamingSink::StopStreaming();
}

tResult cArduinoActuator::GetSignalValueFromReader(sample_reader<size_limited_sample_reader_queue<1>>& reader, tFloat32& signalValue) const
{
    object_ptr<const ISample> pSample;
    object_ptr_shared_locked<const ISampleBuffer> pBuffer;
    RETURN_IF_FAILED(reader.GetNextSample(pSample));
    RETURN_IF_FAILED(pSample->Lock(pBuffer));

    //create DECODER to get access to value of sample
    auto oDecoder = m_inputSignalValueSampleFactory.MakeDecoderFor(*pSample);
    RETURN_IF_FAILED(oDecoder.IsValid());

    RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &signalValue));
    RETURN_NOERROR;
}

tResult cArduinoActuator::GetBoolSignalValueFromLightReaders(LIGHT_MASK& mask, tBool& value) const
{
    object_ptr<const ISample> pSample;
    object_ptr_shared_locked<const ISampleBuffer> pBuffer;
    for (auto reader : m_mapLightMaskToReaderMap)
    {
        if IS_OK(reader.second->GetNextSample(pSample))
        {
            mask = reader.first;
            break;
        }

    }
    RETURN_IF_POINTER_NULL(pSample);
    RETURN_IF_FAILED(pSample->Lock(pBuffer));

    //create DECODER to get access to value of sample
    auto oDecoder = m_inputBoolSignalValueSampleFactory.MakeDecoderFor(*pSample);
    RETURN_IF_FAILED(oDecoder.IsValid());

    RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlBoolSignalValueId.value, &value));
    RETURN_NOERROR;
}

tResult cArduinoActuator::TimerFunc()
{
    //keep the watchdog alive
    m_serialDevice.send_watchdog_trigger();

    // read motor actuator (steering + speed)
    tFloat32 steeringValue = 0;
    if IS_OK(GetSignalValueFromReader(m_readerSteering, steeringValue))
    {
        m_serialDevice.send_steering(steeringValue);
        CONSOLE_LOG_INFO(cString::Format("Setting steering %f", steeringValue));
    }

    tFloat32 speedValue = 0;
    tResult result = ERR_NOERROR;
    if (m_pClock->GetStreamTime() > m_startupTime)
    { // we need to send zero to the drive controller after startup
        result = GetSignalValueFromReader(m_readerSpeed, speedValue);
    }
    else
    {
        CONSOLE_LOG_INFO(cString::Format("Init Speed %f", speedValue))
    }
    if IS_OK(result)
    {
        m_serialDevice.send_speed(speedValue);
        CONSOLE_LOG_INFO(cString::Format("Setting Speed %f", speedValue));
    }

    //light signals handling
    LIGHT_MASK lightID;
    tBool bValue = tFalse;
    if IS_OK(GetBoolSignalValueFromLightReaders(lightID, bValue))
    {
        //Special case: If turn signal switch from left to right. Disable other turn signal.
        if (lightID == ID_ARD_ACT_LIGHT_MASK_TURNLEFT)
        {
            m_lightMask &= ~ID_ARD_ACT_LIGHT_MASK_TURNRIGHT;
        }

        if (lightID == ID_ARD_ACT_LIGHT_MASK_TURNRIGHT)
        {
            m_lightMask &= ~ID_ARD_ACT_LIGHT_MASK_TURNLEFT;
        }

        // set enable or disable in frame
        if (bValue == tTrue)
        {
            m_lightMask |= lightID;
        }
        else
        {
            m_lightMask &= ~lightID;
        }
        m_serialDevice.send_light(m_lightMask);
        CONSOLE_LOG_INFO(cString::Format("Setting light %d, with mask %x", bValue, m_lightMask));
    }

    RETURN_NOERROR;
}
