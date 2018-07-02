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

#pragma once

#include "stdafx.h"
#include "aadc_structs.h"

#define CID_ADTF_SENSOR_VISUALIZATION  "sensor_visualization.filter.demo.aadc.cid"
#define LABEL_ADTF_SENSOR_VISUALIZATION  "Sensor Visualization"

class cSensorVisualizationWidget;


/*! A sensor visualization filter. */
class cSensorVisualizationFilter : public QObject, virtual public cQtUIFilter
{
    Q_OBJECT

signals:

    /*!
     * Sets the us.
     *
     * \param   usData  The data.
     */
    void setUs                  (tUltrasonicStruct usData);

    /*!
     * Sets an imu.
     *
     * \param   imuData Information describing the imu.
     */
    void setImu                 (tInerMeasUnitData imuData);

    /*!
     * Sets wheel left.
     *
     * \param   count       Number of.
     * \param   direction   The direction.
     */
    void setWheelLeft           (int count, int direction);

    /*!
     * Sets wheel right.
     *
     * \param   count       Number of.
     * \param   direction   The direction.
     */
    void setWheelRight          (int count, int direction);

    /*!
     * Sets a voltage.
     *
     * \param   voltData    Information describing the volt.
     */
    void setVoltage             (tVoltageStruct voltData);

    /*!
     * Sets laser scan.
     *
     * \param [in,out]  scan    The scan.
     */
    void setLaserScan           (aadc::laserscanner::tLaserScan& scan);

public:
    ADTF_CLASS_ID_NAME(cSensorVisualizationFilter, CID_ADTF_SENSOR_VISUALIZATION, LABEL_ADTF_SENSOR_VISUALIZATION);
    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::ui::ant::IQtXSystem),
                            REQUIRE_INTERFACE(adtf::services::IReferenceClock));

private:

    /*! The input iner meas unit */
    cPinReader       m_oInputInerMeasUnit;

    /*! The input ultrasonic unit */
    cPinReader       m_oInputUltrasonicUnit;

    /*! The inputl wheel left */
    cPinReader       m_oInputlWheelLeft;

    /*! The inputl wheel right */
    cPinReader       m_oInputlWheelRight;

    /*! The input voltage unit  */
    cPinReader       m_oInputVoltageUnit;

    /*! The input laser scanner */
    cPinReader       m_oInputLaserScanner;


    /*! A ddl iner meas unit data index. */
    struct
    {
        tSize timeStamp;
        tSize A_x;
        tSize A_y;
        tSize A_z;
        tSize G_x;
        tSize G_y;
        tSize G_z;
        tSize M_x;
        tSize M_y;
        tSize M_z;
    } m_ddlInerMeasUnitDataIndex;


    /*! The imu data sample factory */
    adtf::mediadescription::cSampleCodecFactory m_IMUDataSampleFactory;

    /*! A signal value identifier. */
    struct tSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;

    /*! A ddl ultrasonic structure index. */
    struct
    {
        tSignalValueId SideLeft;
        tSignalValueId SideRight;
        tSignalValueId RearLeft;
        tSignalValueId RearCenter;
        tSignalValueId RearRight;

    } m_ddlUltrasonicStructIndex;
    adtf::mediadescription::cSampleCodecFactory m_USDataSampleFactory;


    /*! A ddl wheel data index. */
    struct
    {
        tSize ArduinoTimestamp;
        tSize WheelTach;
        tSize WheelDir;
    } m_ddlWheelDataIndex;
    adtf::mediadescription::cSampleCodecFactory m_WheelDataSampleFactory;
    
    /*! A ddl voltage structure index. */
    struct
    {
        tSignalValueId ActuatorVoltage;
        tSignalValueId ActuatorCell1;
        tSignalValueId ActuatorCell2;
        tSignalValueId SensorVoltage;
        tSignalValueId SensorCell1;
        tSignalValueId SensorCell2;
        tSignalValueId SensorCell3;
        tSignalValueId SensorCell4;
        tSignalValueId SensorCell5;
        tSignalValueId SensorCell6;
    } m_ddlVoltageStructIndex;


    /*! The voltage structure sample factory */
    adtf::mediadescription::cSampleCodecFactory m_VoltageStructSampleFactory;

    /*! A ddl laser scanner data identifier. */
    struct ddlLaserScannerDataId
    {
        tSize size;
        tSize scanArray;
    } m_ddlLSDataId;


    /*! The ls structure sample factory */
    adtf::mediadescription::cSampleCodecFactory m_LSStructSampleFactory;


    /*! The user interface file widget */
    cSensorVisualizationWidget*     m_pUiFileWidget;


    /*! The mutex */
    std::mutex m_oMutex;

public:


    /*! Default constructor. */
    cSensorVisualizationFilter();


    /*! Destructor. */
    virtual ~cSensorVisualizationFilter();

protected: // Implement cBaseQtFilter
    QWidget * CreateView() override;
    tVoid    ReleaseView() override;
    tResult OnIdle() override;
    tResult  OnTimer() override;
    tResult  Init(tInitStage eStage) override;
};
