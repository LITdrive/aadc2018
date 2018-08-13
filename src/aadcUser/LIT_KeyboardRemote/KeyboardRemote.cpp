/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/
#include "stdafx.h"
#include "KeyboardRemote.h"


ADTF_FILTER_PLUGIN(FILTERNAME, OID, KeyboardRemote);




KeyboardRemote::KeyboardRemote(const tChar* __info) :
    QObject(),
    cBaseQtFilter(__info)
{

    m_szIdOutputSpeedSet = tFalse;
    m_szIdOutputSteeringSet = tFalse;

    keyborad_enabled_ = false;
    transmit_speed_ = SPEED_CONTROLLER_DEFAULT_VALUE;
    transmit_steering_ = STEERING_ANGEL_DEFAULT_VALUE;
    go_steering_enabled_left_ = false;
    go_steering_enabled_right_ = false;
    go_speed_enabled_front_ = false;
    go_speed_enabled_back_ = false;

    SetPropertyBool(KEY_CONTROL_PROPERTY, keyborad_enabled_);
    SetPropertyBool(KEY_CONTROL_PROPERTY NSSUBPROP_ISCHANGEABLE, tFalse);
    SetPropertyStr(KEY_CONTROL_PROPERTY NSSUBPROP_DESCRIPTION, "Key-Control");

    SetPropertyBool(KEY_CONTROL_PROPERTY, keyborad_enabled_);
    SetPropertyBool(KEY_CONTROL_PROPERTY NSSUBPROP_ISCHANGEABLE, tFalse);
    SetPropertyStr(KEY_CONTROL_PROPERTY NSSUBPROP_DESCRIPTION, "Key-Control");

    SetPropertyFloat(SIGNAL_SPEED_PROPERTY, SIGNAL_SPEED_DEFAULT);
    SetPropertyBool(SIGNAL_SPEED_PROPERTY NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(SIGNAL_SPEED_PROPERTY NSSUBPROP_DESCRIPTION, "Speed-Magnitude");

    SetPropertyFloat(SIGNAL_STEERING_PROPERTY, SIGNAL_STEERIG_DEFAULT);
    SetPropertyBool(SIGNAL_STEERING_PROPERTY NSSUBPROP_ISCHANGEABLE, tTrue);
    SetPropertyStr(SIGNAL_STEERING_PROPERTY NSSUBPROP_DESCRIPTION, "Steering-Magnitude");
}

KeyboardRemote::~KeyboardRemote()
{
}

tHandle KeyboardRemote::CreateView()
{
    // create the widget
    QWidget* pWidget = (QWidget*)m_pViewport->VP_GetWindow();
    m_pWidget = new DisplayWidget(pWidget);

    connect(m_pWidget->m_btSendValueFalse, SIGNAL(clicked()), this, SLOT(OnTransmitValueFalse()));
    connect(m_pWidget->m_btSendValueTrue, SIGNAL(clicked()), this, SLOT(OnTransmitValueTrue()));

    connect(m_pWidget, SIGNAL(keyReceived(int)), this, SLOT(keycmd(int)));

    // press
    connect(m_pWidget, SIGNAL(sendpressUp()), this, SLOT(KeyDiveGo()));
    connect(m_pWidget, SIGNAL(sendpressDown()), this, SLOT(KeyDiveBackGo()));
    connect(m_pWidget, SIGNAL(sendpressLeft()), this, SLOT(KeyLeftGo()));
    connect(m_pWidget, SIGNAL(sendpressRight()), this, SLOT(KeyRightGo()));

    // realase
    connect(m_pWidget, SIGNAL(sendreleaseUp()), this, SLOT(KeyDriveStop()));
    connect(m_pWidget, SIGNAL(sendreleaseDown()), this, SLOT(KeyBackStop()));
    connect(m_pWidget, SIGNAL(sendreleaseLeft()), this, SLOT(KeyLeftStop()));
    connect(m_pWidget, SIGNAL(sendreleaseRight()), this, SLOT(KeyRightStop()));

    return (tHandle)m_pWidget;
}

tResult KeyboardRemote::ReleaseView()
{
    // delete the widget if present
    if (m_pWidget != NULL)
    {
        delete m_pWidget;
        m_pWidget = NULL;
    }
    RETURN_NOERROR;
}

tResult KeyboardRemote::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Init(eStage, __exception_ptr));

    if (eStage == StageFirst)
    {
        LOG_INFO("KeyboardRemote: Init");

        // Create Type Description

        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

        // tSignalValue
        tChar const *strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
        RETURN_IF_POINTER_NULL(strDescSignalValue);
        cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue, IMediaDescription::MDF_DDL_DEFAULT_VERSION);



        // speed, description
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &m_pSpeedDescription));
        // speed, pin
        RETURN_IF_FAILED(m_oSpeedPin.Create("Speed", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oSpeedPin));

        // steering, description
        RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid **) &m_pSteeringDescription));

        // steering, pin
        RETURN_IF_FAILED(m_oSteeringPin.Create("Steering", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_oSteeringPin));




        RETURN_NOERROR;
    }
    else if(eStage == StageGraphReady)
    {
    }
    RETURN_NOERROR;
}

tResult KeyboardRemote::Start(__exception)
{
    RETURN_IF_FAILED(cBaseQtFilter::Start(__exception_ptr));

    LOG_INFO("KeyboardRemoteFilter: Start");

    LOG_INFO("KeyboardRemoteFilter: enable controlls in the UI");
    LOG_INFO("KeyboardRemoteFilter: arrow keys: controll the car");
    LOG_INFO("KeyboardRemoteFilter: f keys: increment speed");
    LOG_INFO("KeyboardRemoteFilter: d keys: decrement speed");
    LOG_INFO("KeyboardRemoteFilter: s keys: increment steering");
    LOG_INFO("KeyboardRemoteFilter: a keys: decrement steering");

    keyborad_enabled_ = false;

    RETURN_NOERROR;
}

tResult KeyboardRemote::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */)
{
    return cBaseQtFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}

tResult KeyboardRemote::Stop(__exception)
{
    return cBaseQtFilter::Stop(__exception_ptr);
}

tResult KeyboardRemote::Shutdown(tInitStage eStage, __exception)
{
    return cBaseQtFilter::Shutdown(eStage, __exception_ptr);
}

//___________________________________________________________________________________
void KeyboardRemote::keycmd(int key) {

    if (keyborad_enabled_) {
        LOG_INFO("KeyboardRemote: keycmd");

        if (key == KEY_INC_SPEED) {
            if (transmit_speed_ + SPEED_CONTROLLER_INCREMENT_VALUE <=
                SPEED_CONTROLLER_MAX_VALUE)
                transmit_speed_ += SPEED_CONTROLLER_INCREMENT_VALUE;
            LOG_INFO(A_UTILS_NS::cString::Format("speed: %f", transmit_speed_));
            OnTransmitValuesDiveGo();
            OnTransmitValuesDiveBackGo();

        } else if (key == KEY_DECREMENRT_SPEED) {
            if (transmit_speed_ - SPEED_CONTROLLER_INCREMENT_VALUE >=
                SPEED_CONTROLLER_MIN_VALUE)
                transmit_speed_ -= SPEED_CONTROLLER_INCREMENT_VALUE;
            LOG_INFO(A_UTILS_NS::cString::Format("speed: %f", transmit_speed_));
            OnTransmitValuesDiveGo();
            OnTransmitValuesDiveBackGo();

        } else if (key == KEY_INC_ANGEL) {
            if (transmit_steering_ + STEERING_ANGEL_INCREMENT_VALUE <=
                STEERING_ANGEL_MAX_VALUE)
                transmit_steering_ += STEERING_ANGEL_INCREMENT_VALUE;
            LOG_INFO(A_UTILS_NS::cString::Format("steering: %f",
                                                 transmit_steering_));
            OnTransmitValuesLeftGo();
            OnTransmitValuesRightGo();

        } else if (key == KEY_DECREMENRT_ANGEL) {
            if (transmit_steering_ - STEERING_ANGEL_INCREMENT_VALUE >=
                STEERING_ANGEL_MIN_VALUE)
                transmit_steering_ -= STEERING_ANGEL_INCREMENT_VALUE;
            LOG_INFO(A_UTILS_NS::cString::Format("steering: %f",
                                                 transmit_steering_));
            OnTransmitValuesLeftGo();
            OnTransmitValuesRightGo();
        }

    }
}

// forward
void KeyboardRemote::KeyDiveGo() {
    go_speed_enabled_front_ = true;
    OnTransmitValuesDiveGo();
}
void KeyboardRemote::KeyDriveStop() {
    go_speed_enabled_front_ = false;
    OnTransmitValuesDriveStop();
}

// backward
void KeyboardRemote::KeyDiveBackGo() {
    go_speed_enabled_back_ = true;
    OnTransmitValuesDiveBackGo();
}
void KeyboardRemote::KeyBackStop() {
    go_speed_enabled_back_ = false;
    OnTransmitValuesDriveBackStop();
}

void KeyboardRemote::KeyLeftGo() {
    go_steering_enabled_left_ = true;
    OnTransmitValuesLeftGo();
}
void KeyboardRemote::KeyLeftStop() {
    go_steering_enabled_left_ = false;
    OnTransmitValuesLeftStop();
}

void KeyboardRemote::KeyRightGo() {
    go_steering_enabled_right_ = true;
    OnTransmitValuesRightGo();
}

void KeyboardRemote::KeyRightStop() {
    go_steering_enabled_right_ = false;
    OnTransmitValuesRightStop();
}

//___________________________________________________________________________________
// Front
void KeyboardRemote::OnTransmitValuesDiveGo() {
    if (keyborad_enabled_ && go_speed_enabled_front_)
    {

        LOG_INFO("DEBUG GO");



        cObjectPtr<IMediaSample> pMediaSample;
        AllocMediaSample((tVoid**)&pMediaSample);

        cObjectPtr<IMediaSerializer> pSerializer;
        m_pSpeedDescription->GetMediaSampleSerializer(&pSerializer);
        pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());


        tFloat32 value = transmit_speed_;
        tUInt32 timestamp = 0;

        {
            __adtf_sample_write_lock_mediadescription(m_pSpeedDescription, pMediaSample, pCoderOutput);

            if(!m_szIdOutputSpeedSet)
            {
                pCoderOutput->GetID("f32Value", m_szIdOutputSpeedValue);
                pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIdOutputSpeedTs);
                m_szIdOutputSpeedSet = tTrue;
            }

            pCoderOutput->Set(m_szIdOutputSpeedValue, (tVoid*)&value);
            pCoderOutput->Set(m_szIdOutputSpeedTs, (tVoid*)&timestamp);
        }

        pMediaSample->SetTime(_clock->GetStreamTime());
        m_oSpeedPin.Transmit(pMediaSample);



    } else {
        // LOG_INFO("Keyborad controll disabled");
    }
}

void KeyboardRemote::OnTransmitValuesDriveStop() {
    if (keyborad_enabled_) {
        LOG_INFO("DEBUG STOP");

        cObjectPtr<IMediaSample> pMediaSample;
        AllocMediaSample((tVoid**)&pMediaSample);

        cObjectPtr<IMediaSerializer> pSerializer;
        m_pSpeedDescription->GetMediaSampleSerializer(&pSerializer);
        pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());


        tFloat32 value = 0.0;
        tUInt32 timestamp = 0;

        {
            __adtf_sample_write_lock_mediadescription(m_pSpeedDescription, pMediaSample, pCoderOutput);

            if(!m_szIdOutputSpeedSet)
            {
                pCoderOutput->GetID("f32Value", m_szIdOutputSpeedValue);
                pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIdOutputSpeedTs);
                m_szIdOutputSpeedSet = tTrue;
            }

            pCoderOutput->Set(m_szIdOutputSpeedValue, (tVoid*)&value);
            pCoderOutput->Set(m_szIdOutputSpeedTs, (tVoid*)&timestamp);
        }

        pMediaSample->SetTime(_clock->GetStreamTime());
        m_oSpeedPin.Transmit(pMediaSample);
    }
}

//Back
void KeyboardRemote::OnTransmitValuesDiveBackGo() {
    if (keyborad_enabled_ && go_speed_enabled_back_) {
        LOG_INFO("DEBUG BACK GO");

        cObjectPtr<IMediaSample> pMediaSample;
        AllocMediaSample((tVoid**)&pMediaSample);

        cObjectPtr<IMediaSerializer> pSerializer;
        m_pSpeedDescription->GetMediaSampleSerializer(&pSerializer);
        pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());


        tFloat32 value = -transmit_speed_;
        tUInt32 timestamp = 0;

        {
            __adtf_sample_write_lock_mediadescription(m_pSpeedDescription, pMediaSample, pCoderOutput);

            if(!m_szIdOutputSpeedSet)
            {
                pCoderOutput->GetID("f32Value", m_szIdOutputSpeedValue);
                pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIdOutputSpeedTs);
                m_szIdOutputSpeedSet = tTrue;
            }

            pCoderOutput->Set(m_szIdOutputSpeedValue, (tVoid*)&value);
            pCoderOutput->Set(m_szIdOutputSpeedTs, (tVoid*)&timestamp);
        }

        pMediaSample->SetTime(_clock->GetStreamTime());
        m_oSpeedPin.Transmit(pMediaSample);
    }
}

void KeyboardRemote::OnTransmitValuesDriveBackStop() {
    if (keyborad_enabled_) {
        LOG_INFO("DEBUG BACK STOP");

        cObjectPtr<IMediaSample> pMediaSample;
        AllocMediaSample((tVoid**)&pMediaSample);

        cObjectPtr<IMediaSerializer> pSerializer;
        m_pSpeedDescription->GetMediaSampleSerializer(&pSerializer);
        pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());


        tFloat32 value = 0.0;
        tUInt32 timestamp = 0;

        {
            __adtf_sample_write_lock_mediadescription(m_pSpeedDescription, pMediaSample, pCoderOutput);

            if(!m_szIdOutputSpeedSet)
            {
                pCoderOutput->GetID("f32Value", m_szIdOutputSpeedValue);
                pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIdOutputSpeedTs);
                m_szIdOutputSpeedSet = tTrue;
            }

            pCoderOutput->Set(m_szIdOutputSpeedValue, (tVoid*)&value);
            pCoderOutput->Set(m_szIdOutputSpeedTs, (tVoid*)&timestamp);
        }

        pMediaSample->SetTime(_clock->GetStreamTime());
        m_oSpeedPin.Transmit(pMediaSample);
    }
}

//Left
void KeyboardRemote::OnTransmitValuesLeftGo() {
    if (keyborad_enabled_ && go_steering_enabled_left_) {
        LOG_INFO("DEBUG BACK STOP");

        cObjectPtr<IMediaSample> pMediaSample;
        AllocMediaSample((tVoid**)&pMediaSample);

        cObjectPtr<IMediaSerializer> pSerializer;
        m_pSteeringDescription->GetMediaSampleSerializer(&pSerializer);
        pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());


        tFloat32 value = -transmit_steering_;
        tUInt32 timestamp = 0;

        {
            __adtf_sample_write_lock_mediadescription(m_pSteeringDescription, pMediaSample, pCoderOutput);

            if(!m_szIdOutputSteeringSet)
            {
                pCoderOutput->GetID("f32Value", m_szIdOutputSteeringValue);
                pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIdOutputSteeringTs);
                m_szIdOutputSteeringSet = tTrue;
            }

            pCoderOutput->Set(m_szIdOutputSteeringValue, (tVoid*)&value);
            pCoderOutput->Set(m_szIdOutputSteeringTs, (tVoid*)&timestamp);
        }

        pMediaSample->SetTime(_clock->GetStreamTime());
        m_oSteeringPin.Transmit(pMediaSample);
    }
}

void KeyboardRemote::OnTransmitValuesLeftStop() {
    if (keyborad_enabled_) {
        LOG_INFO("DEBUG BACK STOP");

        cObjectPtr<IMediaSample> pMediaSample;
        AllocMediaSample((tVoid**)&pMediaSample);

        cObjectPtr<IMediaSerializer> pSerializer;
        m_pSteeringDescription->GetMediaSampleSerializer(&pSerializer);
        pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());


        tFloat32 value = 0.0;
        tUInt32 timestamp = 0;

        {
            __adtf_sample_write_lock_mediadescription(m_pSteeringDescription, pMediaSample, pCoderOutput);

            if(!m_szIdOutputSteeringSet)
            {
                pCoderOutput->GetID("f32Value", m_szIdOutputSteeringValue);
                pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIdOutputSteeringTs);
                m_szIdOutputSteeringSet = tTrue;
            }

            pCoderOutput->Set(m_szIdOutputSteeringValue, (tVoid*)&value);
            pCoderOutput->Set(m_szIdOutputSteeringTs, (tVoid*)&timestamp);
        }

        pMediaSample->SetTime(_clock->GetStreamTime());
        m_oSteeringPin.Transmit(pMediaSample);
    }
}

//Right
void KeyboardRemote::OnTransmitValuesRightGo() {
    if (keyborad_enabled_ && go_steering_enabled_right_) {
        LOG_INFO("DEBUG BACK STOP");

        cObjectPtr<IMediaSample> pMediaSample;
        AllocMediaSample((tVoid**)&pMediaSample);

        cObjectPtr<IMediaSerializer> pSerializer;
        m_pSteeringDescription->GetMediaSampleSerializer(&pSerializer);
        pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());


        tFloat32 value = transmit_steering_;
        tUInt32 timestamp = 0;

        {
            __adtf_sample_write_lock_mediadescription(m_pSteeringDescription, pMediaSample, pCoderOutput);

            if(!m_szIdOutputSteeringSet)
            {
                pCoderOutput->GetID("f32Value", m_szIdOutputSteeringValue);
                pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIdOutputSteeringTs);
                m_szIdOutputSteeringSet = tTrue;
            }

            pCoderOutput->Set(m_szIdOutputSteeringValue, (tVoid*)&value);
            pCoderOutput->Set(m_szIdOutputSteeringTs, (tVoid*)&timestamp);
        }

        pMediaSample->SetTime(_clock->GetStreamTime());
        m_oSteeringPin.Transmit(pMediaSample);
    }
}

void KeyboardRemote::OnTransmitValuesRightStop() {
    if (keyborad_enabled_) {
        LOG_INFO("DEBUG BACK STOP");

        cObjectPtr<IMediaSample> pMediaSample;
        AllocMediaSample((tVoid**)&pMediaSample);

        cObjectPtr<IMediaSerializer> pSerializer;
        m_pSteeringDescription->GetMediaSampleSerializer(&pSerializer);
        pMediaSample->AllocBuffer(pSerializer->GetDeserializedSize());


        tFloat32 value = 0.0;
        tUInt32 timestamp = 0;

        {
            __adtf_sample_write_lock_mediadescription(m_pSteeringDescription, pMediaSample, pCoderOutput);

            if(!m_szIdOutputSteeringSet)
            {
                pCoderOutput->GetID("f32Value", m_szIdOutputSteeringValue);
                pCoderOutput->GetID("ui32ArduinoTimestamp", m_szIdOutputSteeringTs);
                m_szIdOutputSteeringSet = tTrue;
            }

            pCoderOutput->Set(m_szIdOutputSteeringValue, (tVoid*)&value);
            pCoderOutput->Set(m_szIdOutputSteeringTs, (tVoid*)&timestamp);
        }

        pMediaSample->SetTime(_clock->GetStreamTime());
        m_oSteeringPin.Transmit(pMediaSample);
    }
}

void KeyboardRemote::OnTransmitValueFalse()
{
     //LOG_FN_OUTPUT(( "KeyboardRemote: OnTransmitValueFalse" ), A_UTILS_NS::LOG_LVL_INFO);
    LOG_INFO("KeyboardRemote: OnTransmitValueFalse");

    keyborad_enabled_ = false;
    SetPropertyBool(KEY_CONTROL_PROPERTY, keyborad_enabled_);
}

void KeyboardRemote::OnTransmitValueTrue()
{
    LOG_INFO("KeyboardRemote: OnTransmitValueTrue");

    keyborad_enabled_ = true;
    SetPropertyBool(KEY_CONTROL_PROPERTY, keyborad_enabled_);
}
