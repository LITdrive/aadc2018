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

#ifndef _ARD_BATTERY_SENSORS_H_
#define _ARD_BATTERY_SENSORS_H_

#include "../ArduinoBase.h"

#define CID_ARD_BATTERY_SENSOR_STREAMING_SOURCE "arduino_battery_sensor.streaming_source.base.aadc.cid"
#define LABEL_ARD_BATTERY_SENSOR_STREAMING_SOURCE "Arduino Battery Sensor"


/*! Main class for the arduino filter which reads the battery sensor values */
class ArduinoBatterySensor : public cArduinoBase
{
public:
    ADTF_CLASS_ID_NAME(ArduinoBatterySensor,
        CID_ARD_BATTERY_SENSOR_STREAMING_SOURCE,
        LABEL_ARD_BATTERY_SENSOR_STREAMING_SOURCE);

    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::services::IReferenceClock));

private:

    //the writer for the US struct
    cSampleWriter m_oOutputWriterBatteryStruct;

    /*! The map us sensor to string also corresponding to the ddl */
    std::map<SENSOR_ID, std::string> m_mapBatterySensor2String;


    /*! the ddl identifiers for signalvalue */
    struct ddlSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;

    /*! mapping of the us sensor id to ddl indices */
    std::map<SENSOR_ID, ddlSignalValueId> m_voltageStructSignalValueIDs;

    /*! last recevied values*/
    std::map<SENSOR_ID, tSignalValue> m_voltageStructLastValues;

    //the sample factory for the uss struct
    adtf::mediadescription::cSampleCodecFactory m_voltageStructSampleFactory;

public:

    /*! Default constructor. */
    ArduinoBatterySensor();

    tResult Construct() override;
    tResult Destruct() override;

    /*!
     * implementation of the arduino frames
     *
     * \param   headerID            Identifier for the header.
     * \param   arduinoTimestamp    The arduino timestamp.
     * \param   data                The data.
     *
     * \return  Standard Result Code.
     */
    virtual tResult Transmit(tUInt8 headerID, tUInt32 arduinoTimestamp, tDataUnion data) override;

    /*!
     * Transmit voltage structure as mediasample
     *
     * \return  Standard Result Code.
     */
    tResult TransmitVoltageStruct();


};


//*************************************************************************************************
#endif // _ARD_BATTERY_SENSORS_H_
