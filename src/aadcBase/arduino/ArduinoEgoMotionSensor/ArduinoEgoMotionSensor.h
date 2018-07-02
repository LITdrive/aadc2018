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

#ifndef _ARD_EGOMO_SENSORS_H_
#define _ARD_EGOMO_SENSORS_H_

#include "../ArduinoBase.h"

#define CID_ARD_EGOMO_SENSOR_STREAMING_SOURCE "arduino_egomotion_sensor.streaming_source.base.aadc.cid"
#define LABEL_ARD_EGOMO_SENSOR_STREAMING_SOURCE "Arduino EgoMotion Sensor"

/*! this is the main class for the filter which receives the ego motion sensors from the arduino */
class ArduinoEgoMotionSensor : public cArduinoBase
{
public:
    ADTF_CLASS_ID_NAME(ArduinoEgoMotionSensor,
        CID_ARD_EGOMO_SENSOR_STREAMING_SOURCE,
        LABEL_ARD_EGOMO_SENSOR_STREAMING_SOURCE);

    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::services::IReferenceClock));

private:

    /*! The output sample writer for imu */
    cSampleWriter m_outputWriterIMU;
    /*! The output sample writer for wheel left */
    cSampleWriter m_outputWriterWheelLeft;
    /*! The output sample writer for wheel right */
    cSampleWriter m_outputWriterWheelRight;

    /*! The ddl indices for a tInerMeasUnitData */
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

    /*! The ddl indices for a tInerMeasUnitData */
    struct
    {
        tSize timeStamp;
        tSize tach;
        tSize dir;
    } m_ddlWheelDataIndex;

    /*! The wheel data sample factory */
    adtf::mediadescription::cSampleCodecFactory m_wheelDataSampleFactory;

public:

    /*! Default constructor. */
    ArduinoEgoMotionSensor();

    tResult Construct() override;
    tResult Destruct() override;

    /*!
     * Transmits the arduino frames
     *
     * \param   headerID            Identifier for the header.
     * \param   arduinoTimestamp    The arduino timestamp.
     * \param   data                The data.
     *
     * \return  Standard Result Code.
     */
    virtual tResult Transmit(tUInt8 headerID, tUInt32 arduinoTimestamp, tDataUnion data) override;

    /*!
     * Transmit wheel data as media sample
     *
     * \param           timestamp   The timestamp.
     * \param           wheelData   Information describing the wheel.
     * \param [in,out]  writer      The writer.
     *
     * \return  Standard Result Code.
     */
    tResult TransmitWheelData(tUInt32 timestamp, tDataUnion wheelData, cSampleWriter& writer);

    /*!
     * Transmit iner meas unit data media samples
     *
     * \param   timestamp   The timestamp.
     * \param   imuData     Information describing the imu.
     *
     * \return  Standard Result Code.
     */
    tResult TransmitInerMeasUnitData(tUInt32 timestamp, tDataUnion imuData);

};


//*************************************************************************************************
#endif // _ARD_EGOMO_SENSORS_H_
