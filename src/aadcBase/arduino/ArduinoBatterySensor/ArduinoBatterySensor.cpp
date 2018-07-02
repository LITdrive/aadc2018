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

#include "ArduinoBatterySensor.h"

#define CONSOLE_LOG(_text, _log_level) if (m_propEnableConsoleOutput) { LOG_ADD_ENTRY(_log_level, _text); }    //!< enables log function if console output is activated
#define CONSOLE_LOG_INFO(_text)      CONSOLE_LOG(_text, A_UTILS_NS::log::tLogLevel::Info)                        //!< log info messages

ADTF_PLUGIN(LABEL_ARD_BATTERY_SENSOR_STREAMING_SOURCE,
    ArduinoBatterySensor);

ArduinoBatterySensor::ArduinoBatterySensor() : cArduinoBase(ARDUINO_CENTER_MEASUREMENT)
{
    m_mapBatterySensor2String.insert(std::make_pair(ID_ARD_SENS_VOLT_ACTUATOR, "tActuatorVoltage"));
    m_mapBatterySensor2String.insert(std::make_pair(ID_ARD_SENS_VOLT_ACTUATOR_CELL1, "tActuatorCell1"));
    m_mapBatterySensor2String.insert(std::make_pair(ID_ARD_SENS_VOLT_ACTUATOR_CELL2, "tActuatorCell2"));
    m_mapBatterySensor2String.insert(std::make_pair(ID_ARD_SENS_VOLT_SENSORS, "tSensorVoltage"));
    m_mapBatterySensor2String.insert(std::make_pair(ID_ARD_SENS_VOLT_SENSORS_CELL1, "tSensorCell1"));
    m_mapBatterySensor2String.insert(std::make_pair(ID_ARD_SENS_VOLT_SENSORS_CELL2, "tSensorCell2"));
    m_mapBatterySensor2String.insert(std::make_pair(ID_ARD_SENS_VOLT_SENSORS_CELL3, "tSensorCell3"));
    m_mapBatterySensor2String.insert(std::make_pair(ID_ARD_SENS_VOLT_SENSORS_CELL4, "tSensorCell4"));
    m_mapBatterySensor2String.insert(std::make_pair(ID_ARD_SENS_VOLT_SENSORS_CELL5, "tSensorCell5"));
    m_mapBatterySensor2String.insert(std::make_pair(ID_ARD_SENS_VOLT_SENSORS_CELL6, "tSensorCell6"));

}

tResult ArduinoBatterySensor::Construct()
{
    RETURN_IF_FAILED(cArduinoBase::Construct());

    //get the media description for the ultrasonic struct
    object_ptr<IStreamType> pType;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tVoltageStruct", pType, m_voltageStructSampleFactory))
    {
        //// find the indexes of the element for faster access in the process method.
        LogNamedMessage("Found mediadescription for tVoltageStruct!");
        //get all the member indices
        for (auto element : m_mapBatterySensor2String)
        {
            RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_voltageStructSampleFactory, cString(std::string(element.second + ".ui32ArduinoTimestamp").c_str()), m_voltageStructSignalValueIDs[element.first].timeStamp));
            RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_voltageStructSampleFactory, cString(std::string(element.second + ".f32Value").c_str()), m_voltageStructSignalValueIDs[element.first].value));
        }

    }
    else
    {
        LOG_INFO("No mediadescription for tVoltageStruct found!");
    }
    //create the pins for the ultrasonic struct
    RETURN_IF_FAILED(create_pin(*this, m_oOutputWriterBatteryStruct, "voltage_struct", pType));

    RETURN_NOERROR;
}

tResult ArduinoBatterySensor::Destruct()
{
    return cArduinoBase::Destruct();
}

tResult ArduinoBatterySensor::Transmit(tUInt8 headerID, tUInt32 arduinoTimestamp, tDataUnion data)
{
    CONSOLE_LOG_INFO(cString::Format("Received Battery data, headerID: %d, timestamp: %ld, voltage: %d", headerID, arduinoTimestamp, data.voltage.ui16VoltageData));

    const SENSOR_ID currentSensor = static_cast<SENSOR_ID>(headerID);
    static SENSOR_ID firstSensor = currentSensor;
    tSignalValue singleSignalValue;
    singleSignalValue.f32Value = data.voltage.ui16VoltageData;
    singleSignalValue.ui32ArduinoTimestamp = arduinoTimestamp;
    //save signal value to map
    m_voltageStructLastValues[currentSensor] = singleSignalValue;

    if (currentSensor == firstSensor)
    { //trigger struct on first received data
        TransmitVoltageStruct();
    }

    RETURN_NOERROR;
}

tResult ArduinoBatterySensor::TransmitVoltageStruct()
{
    object_ptr<ISample> pSample;

    RETURN_IF_FAILED(alloc_sample(pSample, m_pClock->GetStreamTime()))
    {

        auto oCodec = m_voltageStructSampleFactory.MakeCodecFor(pSample);

        for (auto element : m_voltageStructSignalValueIDs)
        {
            tSignalValue elementValue = m_voltageStructLastValues[element.first];
            RETURN_IF_FAILED(oCodec.SetElementValue(element.second.value, &elementValue.f32Value));
            RETURN_IF_FAILED(oCodec.SetElementValue(element.second.timeStamp, &elementValue.ui32ArduinoTimestamp));
        }       

    }

    m_oOutputWriterBatteryStruct << pSample << flush << trigger;

    RETURN_NOERROR;
}