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

#include "ArduinoEgoMotionSensor.h"

#define CONSOLE_LOG(_text, _log_level) if (m_propEnableConsoleOutput) { LOG_ADD_ENTRY(_log_level, _text); }    //!< enables log function if console output is activated
#define CONSOLE_LOG_INFO(_text)      CONSOLE_LOG(_text, A_UTILS_NS::log::tLogLevel::Info)                        //!< log info messages

ADTF_PLUGIN(LABEL_ARD_EGOMO_SENSOR_STREAMING_SOURCE,
    ArduinoEgoMotionSensor);

ArduinoEgoMotionSensor::ArduinoEgoMotionSensor() : cArduinoBase(ARDUINO_REAR_IMU_WHEELENC)
{
}


tResult ArduinoEgoMotionSensor::Construct()
{
    RETURN_IF_FAILED(cArduinoBase::Construct());

    //the imu struct
    object_ptr<IStreamType> pTypeIMUData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tInerMeasUnitData", pTypeIMUData, m_IMUDataSampleFactory))
    {
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "ui32ArduinoTimestamp", m_ddlInerMeasUnitDataIndex.timeStamp));
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32A_x", m_ddlInerMeasUnitDataIndex.A_x));
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32A_y", m_ddlInerMeasUnitDataIndex.A_y));
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32A_z", m_ddlInerMeasUnitDataIndex.A_z));

        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32G_x", m_ddlInerMeasUnitDataIndex.G_x));
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32G_y", m_ddlInerMeasUnitDataIndex.G_y));
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32G_z", m_ddlInerMeasUnitDataIndex.G_z));

        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32M_x", m_ddlInerMeasUnitDataIndex.M_x));
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32M_y", m_ddlInerMeasUnitDataIndex.M_y));
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32M_z", m_ddlInerMeasUnitDataIndex.M_z));
    }
    else
    {
        LOG_INFO("No mediadescription for tInerMeasUnitData found!");
    }
    //create the pins
    RETURN_IF_FAILED(create_pin(*this, m_outputWriterIMU, "iner_meas_unit", pTypeIMUData));

    //the two wheel data pins
    object_ptr<IStreamType> pTypeWheelData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tWheelData", pTypeWheelData, m_wheelDataSampleFactory))
    {
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_wheelDataSampleFactory, "ui32ArduinoTimestamp", m_ddlWheelDataIndex.timeStamp));
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_wheelDataSampleFactory, "ui32WheelTach", m_ddlWheelDataIndex.tach));
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_wheelDataSampleFactory, "i8WheelDir", m_ddlWheelDataIndex.dir));
    }
    else
    {
        LOG_INFO("No mediadescription for tWheelData found!");
    }
    //create the pins
    RETURN_IF_FAILED(create_pin(*this, m_outputWriterWheelLeft, "wheel_left", pTypeWheelData));
    RETURN_IF_FAILED(create_pin(*this, m_outputWriterWheelRight, "wheel_right", pTypeWheelData));


    RETURN_NOERROR;
}


tResult ArduinoEgoMotionSensor::Destruct()
{
    return cArduinoBase::Destruct();
}

tResult ArduinoEgoMotionSensor::Transmit(tUInt8 headerID, tUInt32 arduinoTimestamp, tDataUnion data)
{
    CONSOLE_LOG_INFO(cString::Format("Received Ego Motion data, headerID: %d, timestamp: %ld, wheelTach: %d, wheeldir %d", headerID, arduinoTimestamp, data.wheel.ui32WheelTach, data.wheel.i8WheelDir));

    const SENSOR_ID currentSensor = static_cast<SENSOR_ID>(headerID);

    switch (currentSensor)
    {
    case ID_ARD_SENS_IMU:
        RETURN_IF_FAILED(TransmitInerMeasUnitData(arduinoTimestamp, data));
        break;
    case ID_ARD_SENS_WHEEL_LEFT:
        RETURN_IF_FAILED(TransmitWheelData(arduinoTimestamp, data, m_outputWriterWheelLeft));
        break;
    case ID_ARD_SENS_WHEEL_RIGHT:
        RETURN_IF_FAILED(TransmitWheelData(arduinoTimestamp, data, m_outputWriterWheelRight));
        break;
    default:
        break;
    }

    RETURN_NOERROR;
}

tResult ArduinoEgoMotionSensor::TransmitWheelData(tUInt32 timestamp, tDataUnion wheelData, cSampleWriter& writer)
{
    object_ptr<ISample> pSample;
    RETURN_IF_FAILED(alloc_sample(pSample, m_pClock->GetStreamTime()))
    {
        auto oCodec = m_wheelDataSampleFactory.MakeCodecFor(pSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlWheelDataIndex.timeStamp, timestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlWheelDataIndex.tach, wheelData.wheel.ui32WheelTach));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlWheelDataIndex.dir, wheelData.wheel.i8WheelDir));
    }
    writer << pSample << flush << trigger;

    RETURN_NOERROR;
}

tResult ArduinoEgoMotionSensor::TransmitInerMeasUnitData(tUInt32 timestamp, tDataUnion imuData)
{
    object_ptr<ISample> pSample;
    RETURN_IF_FAILED(alloc_sample(pSample, m_pClock->GetStreamTime()));
    {
        auto oCodec = m_IMUDataSampleFactory.MakeCodecFor(pSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlInerMeasUnitDataIndex.timeStamp, timestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlInerMeasUnitDataIndex.A_x, imuData.imu.f32ax));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlInerMeasUnitDataIndex.A_y, imuData.imu.f32ay));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlInerMeasUnitDataIndex.A_z, imuData.imu.f32az));

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlInerMeasUnitDataIndex.G_x, imuData.imu.f32gx));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlInerMeasUnitDataIndex.G_y, imuData.imu.f32gy));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlInerMeasUnitDataIndex.G_z, imuData.imu.f32gz));

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlInerMeasUnitDataIndex.M_x, imuData.imu.f32mx));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlInerMeasUnitDataIndex.M_y, imuData.imu.f32my));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlInerMeasUnitDataIndex.M_z, imuData.imu.f32mz));
    }
    // the sample buffer lock is released in the destructor of oCodec
    m_outputWriterIMU << pSample << flush << trigger;

    RETURN_NOERROR;
}