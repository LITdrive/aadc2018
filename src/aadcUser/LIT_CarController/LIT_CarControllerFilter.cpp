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

#include "LIT_CarControllerFilter.h"
#include "ADTF3_helper.h"
//*************************************************************************************************


ADTF_PLUGIN("LIT Car Controller Plugin", cCarControllerFilter)


cCarControllerFilter::cCarControllerFilter() : m_pUiFileWidget(nullptr)
{
    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value));
    }
    else
    {
        LOG_INFO("No mediadescription for tUltrasonicStruct found!");
    }
    create_pin(*this, m_oOutputSteeringController, "steering", pTypeSignalValue);
    create_pin(*this, m_oOutputSpeedController, "speed", pTypeSignalValue);

    transmit_speed_ = SPEED_DEFAULT_VALUE;
    transmit_steering_ = STEERING_ANGEL_DEFAULT_VALUE;
}


cCarControllerFilter::~cCarControllerFilter()
{

}

QWidget* cCarControllerFilter::CreateView()
{
    // use single UI File in background
    m_pUiFileWidget = new cCarControllerWidget(nullptr);
    connect(m_pUiFileWidget                       , SIGNAL(steeringReceived(int)), this, SLOT(SendSteering(int)));
    connect(m_pUiFileWidget                       , SIGNAL(throttleReceived(int)), this, SLOT(SendThrottle(int)));

    connect(m_pUiFileWidget->getRCButtonGroup(), SIGNAL(buttonClicked(int))     , this, SLOT(ToggleLights(int)));
    connect(m_pUiFileWidget                       , SIGNAL(buttonClicked(int))     , this, SLOT(ToggleLights(int)));

    // keys: a,s,d,w
    connect(m_pUiFileWidget, SIGNAL(keyReceived(int)), this, SLOT(keycmd(int)));

    // press
    connect(m_pUiFileWidget, SIGNAL(sendpressUp()), this, SLOT(KeyDiveGo()));
    connect(m_pUiFileWidget, SIGNAL(sendpressDown()), this, SLOT(KeyDiveBackGo()));
    connect(m_pUiFileWidget, SIGNAL(sendpressLeft()), this, SLOT(KeyLeftGo()));
    connect(m_pUiFileWidget, SIGNAL(sendpressRight()), this, SLOT(KeyRightGo()));

    // realase
    connect(m_pUiFileWidget, SIGNAL(sendreleaseUp()), this, SLOT(KeyDriveStop()));
    connect(m_pUiFileWidget, SIGNAL(sendreleaseDown()), this, SLOT(KeyBackStop()));
    connect(m_pUiFileWidget, SIGNAL(sendreleaseLeft()), this, SLOT(KeyLeftStop()));
    connect(m_pUiFileWidget, SIGNAL(sendreleaseRight()), this, SLOT(KeyRightStop()));

    return m_pUiFileWidget;
}

tVoid cCarControllerFilter::ReleaseView()
{
    delete m_pUiFileWidget;
    m_pUiFileWidget = nullptr;
}

tResult cCarControllerFilter::OnIdle()
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    RETURN_NOERROR;
}

tResult cCarControllerFilter::OnTimer()
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);
    
    RETURN_NOERROR;
}

tResult cCarControllerFilter::Init(tInitStage eStage)
{
    RETURN_IF_FAILED(adtf::ui::cQtUIFilter::Init(eStage));
    
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}


void cCarControllerFilter::keycmd(int key) {

    //if (keyborad_enabled_) {
        LOG_INFO("KeyboardRemote: keycmd");


        if (key == KEY_INC_SPEED) {

            if (transmit_speed_ <= SPEED_MAX_VALUE - SPEED_INCREMENT_VALUE)
            {
                transmit_speed_ = transmit_speed_ + SPEED_INCREMENT_VALUE;
            }

            LOG_INFO("KeyboardRemote: Inc Speed");
            LOG_INFO("speed %f", transmit_speed_);


        } else if (key == KEY_DECREMENRT_SPEED) {
            if (transmit_speed_ >= SPEED_MIN_VALUE + SPEED_INCREMENT_VALUE)
            {
                transmit_speed_ = transmit_speed_ - SPEED_INCREMENT_VALUE;
            }

            LOG_INFO("KeyboardRemote: Dec Speed");
            LOG_INFO("speed %f", transmit_speed_);
        } else if (key == KEY_INC_ANGEL) {

            if (transmit_steering_ <= STEERING_ANGEL_MAX_VALUE - STEERING_ANGEL_INCREMENT_VALUE)
            {
                transmit_steering_ = transmit_steering_ + STEERING_ANGEL_INCREMENT_VALUE;
            }

            LOG_INFO("KeyboardRemote: Inc Angle");
            LOG_INFO("steering %f", transmit_steering_);
        } else if (key == KEY_DECREMENRT_ANGEL) {

            if (transmit_steering_ >= STEERING_ANGEL_MIN_VALUE + STEERING_ANGEL_INCREMENT_VALUE)
            {
                transmit_steering_ = transmit_steering_ - STEERING_ANGEL_INCREMENT_VALUE;
            }

            LOG_INFO("KeyboardRemote: Dec Angle");
            LOG_INFO("steering %f", transmit_steering_);
        }

    //}
}


// forward
void cCarControllerFilter::KeyDiveGo() {
    //go_speed_enabled_front_ = true;
    LOG_INFO("KeyboardRemote: Up Start");
    this->SendThrottle(transmit_speed_);
}
void cCarControllerFilter::KeyDriveStop() {
    //go_speed_enabled_front_ = false;
    LOG_INFO("KeyboardRemote: Up Stop");
    this->SendThrottle(0);
}

// backward
void cCarControllerFilter::KeyDiveBackGo() {
    //go_speed_enabled_back_ = true;
    LOG_INFO("KeyboardRemote: Down Start");
    this->SendThrottle(-transmit_speed_);
}
void cCarControllerFilter::KeyBackStop() {
    //go_speed_enabled_back_ = false;
    LOG_INFO("KeyboardRemote: Down Stop");
    this->SendThrottle(0);
}

void cCarControllerFilter::KeyLeftGo() {
    //go_steering_enabled_left_ = true;
    this->SendSteering(-transmit_steering_);
    LOG_INFO("KeyboardRemote: Left Start");
}
void cCarControllerFilter::KeyLeftStop() {
    //go_steering_enabled_left_ = false;
    this->SendSteering(0);
    LOG_INFO("KeyboardRemote: Left Stop");
}

void cCarControllerFilter::KeyRightGo() {
    //go_steering_enabled_right_ = true;
    this->SendSteering(transmit_steering_);
    LOG_INFO("KeyboardRemote: Right Start");
}

void cCarControllerFilter::KeyRightStop() {
    //go_steering_enabled_right_ = false;
    this->SendSteering(0);
    LOG_INFO("KeyboardRemote: Right Stop");
}

tResult cCarControllerFilter::SendSteering(tFloat32 value)
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    transmitSignalValue(m_oOutputSteeringController, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, value);

    RETURN_NOERROR;
}

tResult cCarControllerFilter::SendThrottle(tFloat32 value)
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    transmitSignalValue(m_oOutputSpeedController, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, value);

    RETURN_NOERROR;
}

tResult cCarControllerFilter::ToggleLights(int buttonId)
{
    switch (buttonId)
    {
        case 0: // Go
            this->SendThrottle(10);
            break;
        case 1: // Brake
            this->SendThrottle(0);
            break;
        default:
            break;
    }

    RETURN_NOERROR;
}
