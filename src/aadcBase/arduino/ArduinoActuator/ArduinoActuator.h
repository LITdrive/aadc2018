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

#ifndef _ARD_ACTUATOR_H_
#define _ARD_ACTUATOR_H_

#include "../helper/arduino_com_client.h"
#include <adtfstreaming3/samplereader.h>
#include "ArduinoActuator.h"

#undef GetObject

#define CID_ARD_ACTUATOR_STREAMING_SINK "arduino_actuator.streaming_sink.base.aadc.cid"
#define LABEL_ARD_ACTUATOR_STREAMING_SINK "Arduino Actuator"

/*! this is the main class of the arduino actuator filter. It forwards the received samples to the arduino to control steering, speed and lights */
class cArduinoActuator : public adtf::streaming::ant::cSampleStreamingSink
{
public:
    ADTF_CLASS_ID_NAME(cArduinoActuator,
        CID_ARD_ACTUATOR_STREAMING_SINK,
        LABEL_ARD_ACTUATOR_STREAMING_SINK);

    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::services::IReferenceClock));

private:

    /*! property for the frame delay */
    property_variable<tInt64> m_nFrameDelay = 1000;

    /*! The property to enable console output */
    property_variable<tBool> m_propEnableConsoleOutput = tFalse;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    /*! ddl identifier for signal value */
    struct ddlSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;

    /*! ddl identifier for the bool signal value . */
    struct ddlBoolSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlBoolSignalValueId;

    /*! The input sample reader for steering */
    sample_reader<size_limited_sample_reader_queue<1>>  m_readerSteering;

    /*! The input sample reader for speed */
    sample_reader<size_limited_sample_reader_queue<1>>  m_readerSpeed;

    /*! The input sample reader for head light */
    sample_reader<size_limited_sample_reader_queue<1>>  m_readerHeadLight;
    /*! The input sample reader for turn signal left */
    sample_reader<size_limited_sample_reader_queue<1>>  m_readerTurnSignalLeft;
    /*! The input sample reader for turn signal right */
    sample_reader<size_limited_sample_reader_queue<1>>  m_readerTurnSignalRight;
    /*! The input sample reader for break light */
    sample_reader<size_limited_sample_reader_queue<1>>  m_readerBreakLight;
    /*! The input sample reader for hazard light */
    sample_reader<size_limited_sample_reader_queue<1>>  m_readerHazardLight;
    /*! The input sample reader for reverse light */
    sample_reader<size_limited_sample_reader_queue<1>>  m_readerReverseLight;

    /*! The map light mask to reader map */
    std::map<LIGHT_MASK, sample_reader<size_limited_sample_reader_queue<1>>*> m_mapLightMaskToReaderMap;

    /*! The timer for sending data */
    kernel_timer m_oTimer;

public:
    cArduinoActuator();

    tResult Construct() override;
    tResult Destruct() override;

    tResult Init() override;
    tResult Shutdown() override;

    tResult StartStreaming() override;
    tResult StopStreaming() override;

    /*!
     * Gets signal value from reader.
     *
     * \param [in,out]  reader      The reader.
     * \param [in,out]  signalValue The signal value.
     *
     * \return  Standard Result Code.
     */
    tResult GetSignalValueFromReader(sample_reader<size_limited_sample_reader_queue<1>>& reader, tFloat32& signalValue) const;

    /*!
     * Gets bool signal value from light readers.
     *
     * \param [in,out]  mask    The mask.
     * \param [in,out]  value   The value.
     *
     * \return  Standard Result Code.
     */
    tResult GetBoolSignalValueFromLightReaders(LIGHT_MASK& mask, tBool& value) const;

private:

    /*!
     * Timer function.
     *
     * \return  Standard Result Code.
     */
    tResult TimerFunc();


    /*! The input signalvalue sample factory */
    adtf::mediadescription::cSampleCodecFactory m_inputSignalValueSampleFactory;

    /*! The input bool signalvalue sample factory */
    adtf::mediadescription::cSampleCodecFactory m_inputBoolSignalValueSampleFactory;
   
    /*! The serial device */
    arduino_com_client m_serialDevice;

    // the prefix OS dependend
    const std::string m_serialDevicePrefix;
    
    /*! Identifier for the arduino */
    const tInt m_arduinoID;

    /*! The startup time the drive controller needs to receive zero speed */
    const tTimeStamp m_startupTime;

    /*! The light mask */
    tUInt8 m_lightMask;
};


//*************************************************************************************************
#endif // _DEMO_ARD_US_SENSORS_H_
