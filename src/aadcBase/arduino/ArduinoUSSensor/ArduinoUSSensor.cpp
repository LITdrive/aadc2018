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
#include "ArduinoUSSensor.h"

#define CONSOLE_LOG(_text, _log_level) if (m_propEnableConsoleOutput) { LOG_ADD_ENTRY(_log_level, _text); }    //!< enables log function if console output is activated
#define CONSOLE_LOG_INFO(_text)      CONSOLE_LOG(_text, A_UTILS_NS::log::tLogLevel::Info)                        //!< log info messages

ADTF_PLUGIN(LABEL_ARD_US_SENSOR_STREAMING_SOURCE,
    cArduinoUSSensor);

cArduinoUSSensor::cArduinoUSSensor() : cArduinoBase(ARDUINO_REAR_US)
{
    m_mapUSSensor2String.insert(std::make_pair(ID_ARD_SENS_US_SIDE_RIGHT, "tSideRight"));
    m_mapUSSensor2String.insert(std::make_pair(ID_ARD_SENS_US_REAR_CENTER_RIGHT, "tRearRight"));
    m_mapUSSensor2String.insert(std::make_pair(ID_ARD_SENS_US_REAR_CENTER, "tRearCenter"));
    m_mapUSSensor2String.insert(std::make_pair(ID_ARD_SENS_US_REAR_CENTER_LEFT, "tRearLeft"));
    m_mapUSSensor2String.insert(std::make_pair(ID_ARD_SENS_US_SIDE_LEFT, "tSideLeft"));
    tSize index = 0;
    for (auto element : m_mapUSSensor2String)
    {
        m_mapSensorID2WriterIndex[element.first] = index;
        index++;
    }
}


tResult cArduinoUSSensor::Construct()
{
    RETURN_IF_FAILED(cArduinoBase::Construct());

    //get the media description for the ultrasonic struct
    object_ptr<IStreamType> pType;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tUltrasonicStruct", pType, m_USStructSampleFactory))
    {
        //// find the indexes of the element for faster access in the process method.
        LogNamedMessage("Found mediadescription for tUltrasonicStruct!");
        //get all the member indices
        for (auto element : m_mapUSSensor2String)
        {
            RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_USStructSampleFactory, cString(std::string(element.second + ".ui32ArduinoTimestamp").c_str()), m_USStructSignalValueIDs[element.first].timeStamp));
            RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_USStructSampleFactory, cString(std::string(element.second + ".f32Value").c_str()), m_USStructSignalValueIDs[element.first].value));
        }

    }
    else
    {
        LOG_INFO("No mediadescription for tUltrasonicStruct found!");
    }
    //create the pins for the ultrasonic struct
    RETURN_IF_FAILED(create_pin(*this, m_oOutputWriterUSStruct, "ultrasonic_struct", pType));

    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_signalValueSampleFactory))
    {
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_signalValueSampleFactory, "ui32ArduinoTimestamp", m_ddlSignalValueId.timeStamp));
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_signalValueSampleFactory, "f32Value", m_ddlSignalValueId.value));

    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }

    //create the pins for the ultrasonic values

    RETURN_IF_FAILED(create_pin(*this, m_outputWritersUS[0], "side_right", pTypeSignalValue));
    RETURN_IF_FAILED(create_pin(*this, m_outputWritersUS[1], "rear_right", pTypeSignalValue));
    RETURN_IF_FAILED(create_pin(*this, m_outputWritersUS[2], "rear_center", pTypeSignalValue));
    RETURN_IF_FAILED(create_pin(*this, m_outputWritersUS[3], "rear_left", pTypeSignalValue));
    RETURN_IF_FAILED(create_pin(*this, m_outputWritersUS[4], "side_left", pTypeSignalValue));

    RETURN_NOERROR;
}


tResult cArduinoUSSensor::Destruct()
{
    return cArduinoBase::Destruct();
}

tResult cArduinoUSSensor::TransmitUSStruct()
{
    object_ptr<ISample> pSample;

    RETURN_IF_FAILED(alloc_sample(pSample, m_pClock->GetStreamTime()))
    {
        auto oCodec = m_USStructSampleFactory.MakeCodecFor(pSample);

        for (auto element : m_USStructSignalValueIDs)
        {
            tSignalValue elementValue = m_USStructLastValues[element.first];
            RETURN_IF_FAILED(oCodec.SetElementValue(element.second.value, &elementValue.f32Value));
            RETURN_IF_FAILED(oCodec.SetElementValue(element.second.timeStamp, &elementValue.ui32ArduinoTimestamp));
        }
    }

    m_oOutputWriterUSStruct << pSample << flush << trigger;

    RETURN_NOERROR;
}

tResult cArduinoUSSensor::TransmitUSSignalValue(const SENSOR_ID currentSensor, const tSignalValue& outValue)
{
    object_ptr<ISample> pSample;
    RETURN_IF_FAILED(alloc_sample(pSample, m_pClock->GetStreamTime()))
    {
        auto oCodec = m_signalValueSampleFactory.MakeCodecFor(pSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.timeStamp, outValue.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, outValue.f32Value));
    }

    m_outputWritersUS[m_mapSensorID2WriterIndex[currentSensor]] << pSample << flush << trigger;

    RETURN_NOERROR;
}

tResult cArduinoUSSensor::Transmit(tUInt8 headerID, tUInt32 arduinoTimestamp, tDataUnion data)
{

    CONSOLE_LOG_INFO(cString::Format("Received US data, headerID: %d, timestamp: %ld, value: %d", headerID, arduinoTimestamp, data.us.i16Distance));

    // create temp data structs and enum
    SENSOR_ID currentSensor = static_cast<SENSOR_ID>(headerID);

    tSignalValue singleSignalValue;
    singleSignalValue.f32Value = data.us.i16Distance;
    singleSignalValue.ui32ArduinoTimestamp = arduinoTimestamp;

    //save signal value to map
    m_USStructLastValues[currentSensor] = singleSignalValue;

    //transmit single signal value
    RETURN_IF_FAILED(TransmitUSSignalValue(currentSensor, singleSignalValue));

    //trigger ultrasonic struct also on us rear center 
    if (currentSensor == ID_ARD_SENS_US_REAR_CENTER)
    {
        RETURN_IF_FAILED(TransmitUSStruct());
    }

    RETURN_NOERROR;
}

