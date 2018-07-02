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

#define CID_WHEELSPEEDCONTROLLER_DATA_TRIGGERED_FILTER "wheel_speed_controller.filter.demo.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

/*! This is the main class for the controller for handling wheel speed. */
class cWheelSpeedController : public cTriggerFunction
{
private:

    //Pins
    /*! Input Pin for wheel struct*/
    cPinReader      m_oInputMeasWheelSpeed;
    /*! Input Pin for wheel struct*/
    cPinReader      m_oInputSetWheelSpeed;


    /*! output pin for the the speed of the wheels */
    cPinWriter      m_oOutputActuator;


    /*! A signal value identifier. */
    struct tSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;

    /*! The signal value sample factory */
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;

    /*! holds the last measuredValue */
    tFloat64 m_f64MeasuredVariable;
    /*! holds the last measured error */
    tFloat64 m_f64LastMeasuredError;
    /*! holds the last setpoint */
    tFloat64 m_f64SetPoint;
    /*! holds the last output */
    tFloat64 m_f64LastOutput;
    /*! holds the last sample time */
    tTimeStamp m_lastSampleTime;
    /*! holds the accumulatedVariable for the controller */
    tFloat64 m_f64accumulatedVariable;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    /*!
    * Gets the time.
    *
    * \return  The time streamtime in milliseconds
    */
    tTimeStamp GetTime();

    /*! calculates the manipulated value for the given values, it uses the setpoint in m_setPoint
    * \param i_f64MeasuredValue    the measuredValue
    * \return the controller output for wheel speed
    */
    tFloat64 getControllerValue(tFloat64 i_f64MeasuredValue);

    // PID-Controller values
    //
    /*! proportional factor for PID Controller */
    property_variable<tFloat64>    m_f64PIDKp = 10;
    /*! integral factor for PID Controller */
    property_variable<tFloat64>    m_f64PIDKi = 0.85;
    /*! differential factor for PID Controller */
    property_variable<tFloat64>    m_f64PIDKd = 0.01;
    /*! the sampletime for the pid controller */
    property_variable<tFloat64> m_f64PIDSampleTime = 0.025;
    /*! the minimum output value for the controller */
    property_variable<tFloat64> m_f64PIDMinimumOutput = -20;
    /*! the maximum output value for the controller */
    property_variable<tFloat64> m_f64PIDMaximumOutput = 20;
    /*! The property for show debug */
    property_variable<tBool>       m_bShowDebug = tFalse;

    /*! input factor for PT1 */
    property_variable<tFloat64> m_f64PT1OutputFactor = 1;
    /*! time constant for pt1 controller */
    property_variable<tFloat64> m_f64PT1TimeConstant = 1;
    /*! the set point is multiplied with this factor, otherwise the set point is not reached by the controller. */
    property_variable<tFloat64> m_f64PT1CorrectionFactor = 1.15;
    /*! gain factor for PT1 controller */
    property_variable<tFloat64>    m_f64PT1Gain = 14;
    /*! defines whether PID or PT1 is used */
    property_variable<tInt32> m_i32ControllerMode = 2;

    /*! holds the last speed value */
    tFloat64 m_f64LastSpeedValue = 0;


public:

    /*! Default constructor. */
    cWheelSpeedController();

    /*! Destructor. */
    virtual ~cWheelSpeedController() = default;

    tResult Configure() override;

    /*!
     * Process the given tmTimeOfTrigger.
     *
     * \param   tmTimeOfTrigger The time time of trigger.
     *
     * \return  Standard Result Code.
     */
    tResult Process(tTimeStamp tmTimeOfTrigger) override;
};


//*************************************************************************************************
