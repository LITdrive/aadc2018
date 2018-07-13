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

#ifndef _ARD_BASE_H_
#define _ARD_BASE_H_

#include <adtf_systemsdk.h>

using namespace adtf::util;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::system;

#include "helper/arduino_com_client.h"
#include "aadc_structs.h"

#undef GetObject


/*! This is the base class for all the arduino sensors and implements the serial communication to the arduinos. */
class cArduinoBase : public adtf::streaming::cSampleStreamingSource
{

protected:

    /*! The property enable console output */
    property_variable<tBool> m_propEnableConsoleOutput = tFalse;

    /*! The clock for streamtime of the mediasamples */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

private:

    /*! The property for the frame delay */
    property_variable<tInt64> m_nFrameDelay = 5000;

    /*! The timer for reading data*/
    kernel_timer m_oTimer;

public:

    /*!
     * Constructor.
     *
     * \param   id  The identifier of the arduino to be used.
     */
    cArduinoBase(ARDUINO_ID id);

    tResult Construct() override;
    tResult Destruct() override;
    tResult StartStreaming() override;
    tResult StopStreaming() override;

    tResult Init() override;
    tResult Shutdown() override;

    /*!
     * Transmitting of the arduino frames which have to be overwritten by the child classes
     *
     * \param   headerID            Identifier for the header.
     * \param   arduinoTimestamp    The arduino timestamp.
     * \param   data                The data.
     *
     * \return  Standard Result Code.
     */
    virtual tResult Transmit(tUInt8 headerID, tUInt32 arduinoTimestamp, tDataUnion data) = 0;

protected:

    /*! Identifier for the arduino */
    const tInt m_arduinoID;
private:
    tVoid TimerFunc();

    /*! The serial device */
    arduino_com_client m_serialDevice;

    // the prefix OS dependend
    const std::string m_serialDevicePrefix;

};


//*************************************************************************************************
#endif // _ARD_BASE_H_
