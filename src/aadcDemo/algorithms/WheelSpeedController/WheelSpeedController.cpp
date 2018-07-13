/**********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: hart#$  $Date:: 2017-05-19 08:12:10#$ $Rev:: 63515   $
**********************************************************************/
#include <adtf3.h>
#include <stdlib.h>
#include "WheelSpeedController.h"
#include "ADTF3_helper.h"


/// This defines a data triggered filter and exposes it via a plugin class factory.
/// The Triggerfunction cSimpleDataStatistics will be embedded to the Filter
/// and called repeatedly (last parameter of this macro)!
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_WHEELSPEEDCONTROLLER_DATA_TRIGGERED_FILTER,
                                    "WheelSpeedController",
                                    cWheelSpeedController,
                                    adtf::filter::pin_trigger({"measured_vehicle_speed"}));
//adtf::filter::thread_trigger(tTrue));

//CTOR of the TriggerFuntion
//This is to initialize the Trigger
cWheelSpeedController::cWheelSpeedController()
{
    //Register Properties
        //RegisterPropertyVariable("wheel circumference in meter", m_f32wheelCircumference);
    RegisterPropertyVariable("proportional factor for PID Controller ", m_f64PIDKp               );
    RegisterPropertyVariable("integral factor for PID Controller", m_f64PIDKi                    );
    RegisterPropertyVariable("differential factor for PID Controller", m_f64PIDKd                );
    RegisterPropertyVariable("sampletime for the pid controller [ms]", m_f64PIDSampleTime             );
    RegisterPropertyVariable("the minimum output value for the controller [m/sec2]", m_f64PIDMinimumOutput);
    RegisterPropertyVariable("the maximum output value for the controller [m/sec2]", m_f64PIDMaximumOutput);
    RegisterPropertyVariable("show debug output", m_bShowDebug                                   );
    RegisterPropertyVariable("input factor for PT1", m_f64PT1OutputFactor                        );
    RegisterPropertyVariable("time constant for pt1 controller", m_f64PT1TimeConstant            );
    RegisterPropertyVariable("set point is multiplied with this factor", m_f64PT1CorrectionFactor);
    RegisterPropertyVariable("gain factor for PT1 controller", m_f64PT1Gain                      );
    RegisterPropertyVariable("controller type", m_i32ControllerMode                              );

    
    

    //Get Media Descriptions
    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value));
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }
    Register(m_oInputMeasWheelSpeed, "measured_vehicle_speed", pTypeSignalValue); 
    Register(m_oInputSetWheelSpeed,  "desired_vehicle_speed", pTypeSignalValue);
    Register(m_oOutputActuator,      "actuator_output", pTypeSignalValue);
}


//implement the Configure function to read ALL Properties
tResult cWheelSpeedController::Configure()
{
    m_f64LastOutput = 0;
    m_f64LastMeasuredError = 0;
    m_f64SetPoint = 0;
    m_lastSampleTime = 0;
    m_f64LastSpeedValue = 0;
    m_f64accumulatedVariable = 0;

    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    RETURN_NOERROR;
}

///this funtion will be executed each time a trigger occured 
///Due to the change of the data receive events it can be possible that more than one sample was pushed to the 
/// Readers queue. So make sure the execution of this funtion will read ALL Samples of ALL Readers until the queues are empty.
tResult cWheelSpeedController::Process(tTimeStamp tmTimeOfTrigger)
{
    // Setpoint value speed
    object_ptr<const ISample> pSetWheelSpeedSample;

    if (IS_OK(m_oInputSetWheelSpeed.GetNextSample(pSetWheelSpeedSample)))
    {
        //write values with zero
        tFloat32 f32Value = 0;
        tUInt32  Ui32TimeStamp = 0;

        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pSetWheelSpeedSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &Ui32TimeStamp));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &f32Value));

        // write to member variable
        m_f64SetPoint = static_cast<tFloat64>(f32Value);
        if (m_i32ControllerMode == 4)
            m_f64SetPoint = m_f64SetPoint * m_f64PT1CorrectionFactor;

    }

    // Measured Wheel Speed
    object_ptr<const ISample> pWheelSpeedSample;

    if (IS_OK(m_oInputMeasWheelSpeed.GetNextSample(pWheelSpeedSample)))
    {
        //write values with zero
        tFloat32 f32Value = 0;
        tUInt32  Ui32TimeStamp = 0;

        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pWheelSpeedSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &Ui32TimeStamp));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &f32Value));

        // write to member variable
        m_f64MeasuredVariable = f32Value;

        //calculation
        // if speed = 0 is requested output is immediately set to zero
        if (m_f64SetPoint == 0)
        {
            m_f64LastOutput = 0;
            m_f64accumulatedVariable = 0;
            m_f64LastMeasuredError = 0;
        }
        else
        {
            m_f64LastOutput = getControllerValue(f32Value)*m_f64PT1OutputFactor;
        }

        tFloat32 outputValue = static_cast<tFloat32>(m_f64LastOutput);
        transmitSignalValue(m_oOutputActuator, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, outputValue);

    }
   

    //if (m_bShowDebug)
    //{
    //    SendSignalData();
    //}

    RETURN_NOERROR;
}
//tResult cWheelSpeedController::SendSignalData()
//{
//    __synchronized_kernel(m_oLock);
//    tTimeStamp tsStreamTime = _clock->GetStreamTime();
//
//    for (tActiveSignals::iterator oSignal = m_oActive.begin();
//         oSignal != m_oActive.end();
//         ++oSignal)
//    {
//        if (*oSignal == WSC_SIGREG_ID_WHEELSPEED_MEASVALUE)
//        {
//            tSignalValue sValue;
//            sValue.nRawValue = 0;
//            sValue.nTimeStamp = tsStreamTime;
//            sValue.strTextValue = 0;
//            sValue.f64Value = m_f64MeasuredVariable;
//            m_pISignalRegistry->UpdateSignal(this, *oSignal, sValue);
//        }
//        else if (*oSignal == WSC_SIGREG_ID_WHEELSPEED_SETVALUE)
//        {
//            tSignalValue sValue;
//            sValue.nRawValue = 0;
//            sValue.nTimeStamp = tsStreamTime;
//            sValue.strTextValue = 0;
//            sValue.f64Value = m_f64SetPoint;
//
//            m_pISignalRegistry->UpdateSignal(this, *oSignal, sValue);
//        }
//
//        else if (*oSignal == WSC_SIGREG_ID_WHEELSPEED_OUTPUTVALUE)
//        {
//            tSignalValue sValue;
//            sValue.nRawValue = 0;
//            sValue.nTimeStamp = tsStreamTime;
//            sValue.strTextValue = 0;
//            sValue.f64Value = m_f64LastOutput;
//
//            m_pISignalRegistry->UpdateSignal(this, *oSignal, sValue);
//        }
//    }
//    RETURN_NOERROR;
//}

tFloat64 cWheelSpeedController::getControllerValue(tFloat64 i_f64MeasuredValue)
{

    //i_f64MeasuredValue = (i_f64MeasuredValue +  m_f64LastSpeedValue) /2.0;

    //m_f64LastSpeedValue = i_f64MeasuredValue;

    tFloat f64Result = 0;

    //the three controller algorithms
    if (m_i32ControllerMode == 1)
    {
        //m_lastSampleTime = GetTime();

        //algorithm:
        //y = Kp * e
        //error:
        tFloat64 f64Error = (m_f64SetPoint - i_f64MeasuredValue);

        f64Result = m_f64PIDKp * f64Error;
    }
    else if (m_i32ControllerMode == 2) //PI- Regler
    {
        //m_lastSampleTime = GetTime();

        //algorithm:
        //esum = esum + e
        //y = Kp * e + Ki * Ta * esum
        //error:
        tFloat64 f64Error = (m_f64SetPoint - i_f64MeasuredValue);
        // accumulated error:

        m_f64accumulatedVariable += (f64Error*m_f64PIDSampleTime);

        f64Result = m_f64PIDKp * f64Error
            + (m_f64PIDKi*m_f64accumulatedVariable);

    }
    else if (m_i32ControllerMode == 3)
    {
        m_lastSampleTime = GetTime();
        tFloat64 f64SampleTime = m_f64PIDSampleTime;

        //algorithm:
        //esum = esum + e
        //y = Kp * e + Ki * Ta * esum + Kd * (e – ealt)/Ta
        //ealt = e

        //error:
        tFloat64 f64Error = (m_f64SetPoint - i_f64MeasuredValue);
        // accumulated error:
        m_f64accumulatedVariable += f64Error * m_f64PIDSampleTime;

        f64Result = m_f64PIDKp * f64Error
            + (m_f64PIDKi*m_f64accumulatedVariable)
            + m_f64PIDKd * (f64Error - m_f64LastMeasuredError) / f64SampleTime;

        m_f64LastMeasuredError = f64Error;
    }
    else if (m_i32ControllerMode == 4)
    {
        /*********************************
        * PT 1 discrete algorithm
        *
        *               Tau
        *       In +    ---  * LastOut
        *             Tsample
        * Out = ---------------------
        *               Tau
        *       1 +     ---
        *             Tsample
        *
        *                           Tau
        * here with     PT1Gain =   ---
        *                         Tsample
        *
        *                  T
        * y(k) = y(k-1) + --- ( v * e(k)  - y(k-1))
        *                  T1
        *
        * e(k):  input
        * y(k-1) : last output
        * v : gain
        * T/T1: time constant
        ********************************************/
        f64Result = m_f64LastOutput + m_f64PT1TimeConstant * (m_f64PT1Gain *
            (m_f64SetPoint - i_f64MeasuredValue) - m_f64LastOutput);

    }
    // checking for minimum and maximum limits
    if (f64Result > m_f64PIDMaximumOutput)
    {
        f64Result = m_f64PIDMaximumOutput;
    }
    else if (f64Result < m_f64PIDMinimumOutput)
    {
        f64Result = m_f64PIDMinimumOutput;
    }

    m_f64LastOutput = f64Result;

    return f64Result;
}

tTimeStamp cWheelSpeedController::GetTime()
{
    return adtf_util::cHighResTimer::GetTime();
}
