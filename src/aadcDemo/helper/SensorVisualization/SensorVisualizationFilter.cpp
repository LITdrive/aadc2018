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


#include "SensorVisualizationFilter.h"


ADTF_PLUGIN(LABEL_ADTF_SENSOR_VISUALIZATION, cSensorVisualizationFilter)


cSensorVisualizationFilter::cSensorVisualizationFilter() : m_pUiFileWidget(nullptr)
{
    //the imu struct
    object_ptr<IStreamType> pTypeIMUData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tInerMeasUnitData", pTypeIMUData, m_IMUDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "ui32ArduinoTimestamp", m_ddlInerMeasUnitDataIndex.timeStamp);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32A_x", m_ddlInerMeasUnitDataIndex.A_x);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32A_y", m_ddlInerMeasUnitDataIndex.A_y);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32A_z", m_ddlInerMeasUnitDataIndex.A_z);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32G_x", m_ddlInerMeasUnitDataIndex.G_x);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32G_y", m_ddlInerMeasUnitDataIndex.G_y);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32G_z", m_ddlInerMeasUnitDataIndex.G_z);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32M_x", m_ddlInerMeasUnitDataIndex.M_x);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32M_y", m_ddlInerMeasUnitDataIndex.M_y);
        adtf_ddl::access_element::find_index(m_IMUDataSampleFactory, "f32M_z", m_ddlInerMeasUnitDataIndex.M_z);
    }
    else
    {
        LOG_INFO("No mediadescription for tInerMeasUnitData found!");
    }

    create_pin(*this, m_oInputInerMeasUnit, "iner_meas_unit", pTypeIMUData);

    //the us struct
    object_ptr<IStreamType> pTypeUSData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tUltrasonicStruct", pTypeUSData, m_USDataSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tSideLeft") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.SideLeft.timeStamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tSideLeft") + cString(".f32Value"), m_ddlUltrasonicStructIndex.SideLeft.value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tSideRight") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.SideRight.timeStamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tSideRight") + cString(".f32Value"), m_ddlUltrasonicStructIndex.SideRight.value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearLeft") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.RearLeft.timeStamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearLeft") + cString(".f32Value"), m_ddlUltrasonicStructIndex.RearLeft.value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearCenter") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.RearCenter.timeStamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearCenter") + cString(".f32Value"), m_ddlUltrasonicStructIndex.RearCenter.value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearRight") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.RearRight.timeStamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearRight") + cString(".f32Value"), m_ddlUltrasonicStructIndex.RearRight.value));
    }
    else
    {
        LOG_INFO("No mediadescription for tUltrasonicStruct found!");
    }
    create_pin(*this, m_oInputUltrasonicUnit, "ultrasonic_struct", pTypeUSData);

    //the wheel data
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
    create_pin(*this, m_oInputlWheelLeft, "wheel_left", pTypeWheelData);
    create_pin(*this, m_oInputlWheelRight, "wheel_right", pTypeWheelData);

    //the voltage data
    object_ptr<IStreamType> pTypeVoltageStruct;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tVoltageStruct", pTypeVoltageStruct, m_VoltageStructSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_VoltageStructSampleFactory, cString("tActuatorVoltage") + cString(".ui32ArduinoTimestamp"), m_ddlVoltageStructIndex.ActuatorVoltage.timeStamp);
        adtf_ddl::access_element::find_index(m_VoltageStructSampleFactory, cString("tActuatorCell1") + cString(".ui32ArduinoTimestamp"), m_ddlVoltageStructIndex.ActuatorCell1.timeStamp);
        adtf_ddl::access_element::find_index(m_VoltageStructSampleFactory, cString("tActuatorCell2") + cString(".ui32ArduinoTimestamp"), m_ddlVoltageStructIndex.ActuatorCell2.timeStamp);
        adtf_ddl::access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorVoltage") + cString(".ui32ArduinoTimestamp"), m_ddlVoltageStructIndex.SensorVoltage.timeStamp);
        adtf_ddl::access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell1") + cString(".ui32ArduinoTimestamp"), m_ddlVoltageStructIndex.SensorCell1.timeStamp);
        adtf_ddl::access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell2") + cString(".ui32ArduinoTimestamp"), m_ddlVoltageStructIndex.SensorCell2.timeStamp);
        adtf_ddl::access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell3") + cString(".ui32ArduinoTimestamp"), m_ddlVoltageStructIndex.SensorCell3.timeStamp);
        adtf_ddl::access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell4") + cString(".ui32ArduinoTimestamp"), m_ddlVoltageStructIndex.SensorCell4.timeStamp);
        adtf_ddl::access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell5") + cString(".ui32ArduinoTimestamp"), m_ddlVoltageStructIndex.SensorCell5.timeStamp);
        adtf_ddl::access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell6") + cString(".ui32ArduinoTimestamp"), m_ddlVoltageStructIndex.SensorCell6.timeStamp);

        adtf_ddl::access_element::find_index(m_VoltageStructSampleFactory, cString("tActuatorVoltage") + cString(".f32Value"), m_ddlVoltageStructIndex.ActuatorVoltage.value);
        adtf_ddl::access_element::find_index(m_VoltageStructSampleFactory, cString("tActuatorCell1") + cString(".f32Value"), m_ddlVoltageStructIndex.ActuatorCell1.value);
        adtf_ddl::access_element::find_index(m_VoltageStructSampleFactory, cString("tActuatorCell2") + cString(".f32Value"), m_ddlVoltageStructIndex.ActuatorCell2.value);
        adtf_ddl::access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorVoltage") + cString(".f32Value"), m_ddlVoltageStructIndex.SensorVoltage.value);
        adtf_ddl::access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell1") + cString(".f32Value"), m_ddlVoltageStructIndex.SensorCell1.value);
        adtf_ddl::access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell2") + cString(".f32Value"), m_ddlVoltageStructIndex.SensorCell2.value);
        adtf_ddl::access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell3") + cString(".f32Value"), m_ddlVoltageStructIndex.SensorCell3.value);
        adtf_ddl::access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell4") + cString(".f32Value"), m_ddlVoltageStructIndex.SensorCell4.value);
        adtf_ddl::access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell5") + cString(".f32Value"), m_ddlVoltageStructIndex.SensorCell5.value);
        adtf_ddl::access_element::find_index(m_VoltageStructSampleFactory, cString("tSensorCell6") + cString(".f32Value"), m_ddlVoltageStructIndex.SensorCell6.value);
    }
    else
    {
        LOG_INFO("No mediadescription for tVoltageStruct found!");
    }
    create_pin(*this, m_oInputVoltageUnit, "voltage_struct", pTypeVoltageStruct);

    object_ptr<IStreamType> pTypeLSData;
    if (ERR_NOERROR == adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tLaserScannerData", pTypeLSData, m_LSStructSampleFactory))
    {
        //// find the indexes of the element for faster access in the process method.
        LogNamedMessage("Found mediadescription for tLaserScannerData!");
        //get all the member indices
        (adtf_ddl::access_element::find_index(m_LSStructSampleFactory, "ui32Size", m_ddlLSDataId.size));
        (adtf_ddl::access_element::find_array_index(m_LSStructSampleFactory, "tScanArray", m_ddlLSDataId.scanArray));
    }
    else
    {
        LOG_INFO("No mediadescription for tLaserScannerData found!");
    }
    create_pin(*this, m_oInputLaserScanner, "laser_scanner", pTypeLSData);
}

cSensorVisualizationFilter::~cSensorVisualizationFilter()
{

}


QWidget* cSensorVisualizationFilter::CreateView()
{
    // use single UI File in background
    m_pUiFileWidget = new cSensorVisualizationWidget(nullptr);

    qRegisterMetaType<aadc::laserscanner::tLaserScan >("aadc::laserscanner::tLaserScan");

    tBool isConnected = tTrue;

    isConnected &= tBool(connect(this, SIGNAL(setImu(tInerMeasUnitData)), m_pUiFileWidget, SLOT(setImu(tInerMeasUnitData))));
    isConnected &= tBool(connect(this, SIGNAL(setUs(tUltrasonicStruct)), m_pUiFileWidget, SLOT(setUs(tUltrasonicStruct))));
    isConnected &= tBool(connect(this, SIGNAL(setWheelLeft(int, int)), m_pUiFileWidget, SLOT(setWheelLeft(int, int))));
    isConnected &= tBool(connect(this, SIGNAL(setWheelRight(int, int)), m_pUiFileWidget, SLOT(setWheelRight(int, int))));
    isConnected &= tBool(connect(this, SIGNAL(setVoltage(tVoltageStruct)), m_pUiFileWidget, SLOT(setVoltage(tVoltageStruct))));
    isConnected &= tBool(connect(this, SIGNAL(setLaserScan(aadc::laserscanner::tLaserScan&)), m_pUiFileWidget, SLOT(setLaserScan(aadc::laserscanner::tLaserScan&))));

    if (!isConnected)
    {
        LOG_WARNING("SensorVisualization not all signal could be connected to a slot!");
    }
    return m_pUiFileWidget;
}

tVoid cSensorVisualizationFilter::ReleaseView()
{
    delete m_pUiFileWidget;
    m_pUiFileWidget = nullptr;
}

tResult cSensorVisualizationFilter::OnIdle()
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    RETURN_NOERROR;
}

tResult cSensorVisualizationFilter::OnTimer()
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    //this will empty the Reader queue and return the last sample received.
    //If no sample was during the time between the last execution of Process the "old sample" is return.
    object_ptr<const ISample> pSampleFromIMU;

    if (IS_OK(m_oInputInerMeasUnit.GetLastSample(pSampleFromIMU)))
    {
        auto oDecoderIMU = m_IMUDataSampleFactory.MakeDecoderFor(*pSampleFromIMU);

        RETURN_IF_FAILED(oDecoderIMU.IsValid());


        // retrieve the values (using convenience methods that return a variant)
            //IMU
        tInerMeasUnitData IMU_data;

        RETURN_IF_FAILED(oDecoderIMU.GetElementValue(m_ddlInerMeasUnitDataIndex.timeStamp, &IMU_data.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oDecoderIMU.GetElementValue(m_ddlInerMeasUnitDataIndex.A_x, &IMU_data.f32A_x));
        RETURN_IF_FAILED(oDecoderIMU.GetElementValue(m_ddlInerMeasUnitDataIndex.A_y, &IMU_data.f32A_y));
        RETURN_IF_FAILED(oDecoderIMU.GetElementValue(m_ddlInerMeasUnitDataIndex.A_z, &IMU_data.f32A_z));
        RETURN_IF_FAILED(oDecoderIMU.GetElementValue(m_ddlInerMeasUnitDataIndex.G_x, &IMU_data.f32G_x));
        RETURN_IF_FAILED(oDecoderIMU.GetElementValue(m_ddlInerMeasUnitDataIndex.G_y, &IMU_data.f32G_y));
        RETURN_IF_FAILED(oDecoderIMU.GetElementValue(m_ddlInerMeasUnitDataIndex.G_z, &IMU_data.f32G_z));
        RETURN_IF_FAILED(oDecoderIMU.GetElementValue(m_ddlInerMeasUnitDataIndex.M_x, &IMU_data.f32M_x));
        RETURN_IF_FAILED(oDecoderIMU.GetElementValue(m_ddlInerMeasUnitDataIndex.M_y, &IMU_data.f32M_y));
        RETURN_IF_FAILED(oDecoderIMU.GetElementValue(m_ddlInerMeasUnitDataIndex.M_z, &IMU_data.f32M_z));

        // Set UI
        emit(setImu(IMU_data));

    }

    object_ptr<const ISample> pSampleFromUS;

    if (IS_OK(m_oInputUltrasonicUnit.GetLastSample(pSampleFromUS)))
    {
        auto oDecoderUS = m_USDataSampleFactory.MakeDecoderFor(*pSampleFromUS);

        RETURN_IF_FAILED(oDecoderUS.IsValid());


        // retrieve the values (using convenience methods that return a variant)
        //IMU
        tUltrasonicStruct US_data;

        //we do not need the timestamps here
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.SideLeft.value, &US_data.tSideLeft.f32Value));
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.SideRight.value, &US_data.tSideRight.f32Value));
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.RearLeft.value, &US_data.tRearLeft.f32Value));
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.RearCenter.value, &US_data.tRearCenter.f32Value));
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.RearRight.value, &US_data.tRearRight.f32Value));

        // Set UI
        emit(setUs(US_data));

    }

    object_ptr<const ISample> pSampleFromWheelLeft;

    if (IS_OK(m_oInputlWheelLeft.GetLastSample(pSampleFromWheelLeft)))
    {
        auto oDecoderWheel = m_WheelDataSampleFactory.MakeDecoderFor(*pSampleFromWheelLeft);

        RETURN_IF_FAILED(oDecoderWheel.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        tWheelData wheelData;

        RETURN_IF_FAILED(oDecoderWheel.GetElementValue(m_ddlWheelDataIndex.ArduinoTimestamp, &wheelData.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oDecoderWheel.GetElementValue(m_ddlWheelDataIndex.WheelTach, &wheelData.ui32WheelTach));
        RETURN_IF_FAILED(oDecoderWheel.GetElementValue(m_ddlWheelDataIndex.WheelDir, &wheelData.i8WheelDir));

        emit(setWheelLeft(wheelData.ui32WheelTach, wheelData.i8WheelDir));

    }

    object_ptr<const ISample> pSampleFromWheelRight;

    if (IS_OK(m_oInputlWheelRight.GetLastSample(pSampleFromWheelRight)))
    {
        auto oDecoderWheel = m_WheelDataSampleFactory.MakeDecoderFor(*pSampleFromWheelRight);

        RETURN_IF_FAILED(oDecoderWheel.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        tWheelData wheelData;

        RETURN_IF_FAILED(oDecoderWheel.GetElementValue(m_ddlWheelDataIndex.ArduinoTimestamp, &wheelData.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oDecoderWheel.GetElementValue(m_ddlWheelDataIndex.WheelTach, &wheelData.ui32WheelTach));
        RETURN_IF_FAILED(oDecoderWheel.GetElementValue(m_ddlWheelDataIndex.WheelDir, &wheelData.i8WheelDir));

        emit(setWheelRight(wheelData.ui32WheelTach, wheelData.i8WheelDir));

    }

    object_ptr<const ISample> pSampleFromVoltage;

    if (IS_OK(m_oInputVoltageUnit.GetLastSample(pSampleFromVoltage)))
    {
        auto oDecoderVoltage = m_VoltageStructSampleFactory.MakeDecoderFor(*pSampleFromVoltage);

        RETURN_IF_FAILED(oDecoderVoltage.IsValid());


        // retrieve the values (using convenience methods that return a variant)
        //TODO use struct here as well

        tVoltageStruct voltData;

        RETURN_IF_FAILED(oDecoderVoltage.GetElementValue(m_ddlVoltageStructIndex.ActuatorVoltage.value, &voltData.tActuatorVoltage.f32Value));
        RETURN_IF_FAILED(oDecoderVoltage.GetElementValue(m_ddlVoltageStructIndex.ActuatorCell1.value, &voltData.tActuatorCell1.f32Value));
        RETURN_IF_FAILED(oDecoderVoltage.GetElementValue(m_ddlVoltageStructIndex.ActuatorCell2.value, &voltData.tActuatorCell2.f32Value));
        RETURN_IF_FAILED(oDecoderVoltage.GetElementValue(m_ddlVoltageStructIndex.SensorVoltage.value, &voltData.tSensorVoltage.f32Value));
        RETURN_IF_FAILED(oDecoderVoltage.GetElementValue(m_ddlVoltageStructIndex.SensorCell1.value, &voltData.tSensorCell1.f32Value));
        RETURN_IF_FAILED(oDecoderVoltage.GetElementValue(m_ddlVoltageStructIndex.SensorCell2.value, &voltData.tSensorCell2.f32Value));
        RETURN_IF_FAILED(oDecoderVoltage.GetElementValue(m_ddlVoltageStructIndex.SensorCell3.value, &voltData.tSensorCell3.f32Value));
        RETURN_IF_FAILED(oDecoderVoltage.GetElementValue(m_ddlVoltageStructIndex.SensorCell4.value, &voltData.tSensorCell4.f32Value));
        RETURN_IF_FAILED(oDecoderVoltage.GetElementValue(m_ddlVoltageStructIndex.SensorCell5.value, &voltData.tSensorCell5.f32Value));
        RETURN_IF_FAILED(oDecoderVoltage.GetElementValue(m_ddlVoltageStructIndex.SensorCell6.value, &voltData.tSensorCell6.f32Value));

        // Set UI
        emit(setVoltage(voltData));
    }


    object_ptr<const ISample> pSampleFromLS;

    if (IS_OK(m_oInputLaserScanner.GetLastSample(pSampleFromLS)))
    {
        auto oDecoder = m_LSStructSampleFactory.MakeDecoderFor(*pSampleFromLS);

        RETURN_IF_FAILED(oDecoder.IsValid());
        tSize numOfScanPoints = 0;
        tResult res = oDecoder.GetElementValue(m_ddlLSDataId.size, &numOfScanPoints);

        const tPolarCoordiante* pCoordinates = reinterpret_cast<const tPolarCoordiante*>(oDecoder.GetElementAddress(m_ddlLSDataId.scanArray));
        aadc::laserscanner::tLaserScan scan;
        tPolarCoordiante scanPoint;

        for (tSize i = 0; i < numOfScanPoints; ++i)
        {
            scanPoint.f32Radius = pCoordinates[i].f32Radius;
            scanPoint.f32Angle = pCoordinates[i].f32Angle;
            scan.push_back(scanPoint);
        }


        emit(setLaserScan(scan));
    }

    RETURN_NOERROR;
}

tResult cSensorVisualizationFilter::Init(tInitStage eStage)
{
    RETURN_IF_FAILED(adtf::ui::cQtUIFilter::Init(eStage));
    RETURN_NOERROR;
}






