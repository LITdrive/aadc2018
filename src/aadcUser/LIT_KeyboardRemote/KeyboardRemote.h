/**********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spie#$  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/



#ifndef _LIT_KEYBOARD_REMOTE_
#define _LIT_KEYBOARD_REMOTE_

// #define __guid "lit.keyboard_remote"

#include "stdafx.h"
#include <aadc_structs.h>
#include "displaywidget.h"

#define OID "LIT.keyboard_remote"
#define FILTERNAME "LIT Keyboard Remote"

#define KEY_CONTROL_PROPERTY "KeyControl"
#define SIGNAL_SPEED_PROPERTY "Speed"
#define SIGNAL_STEERING_PROPERTY "Steering"

#define SIGNAL_SPEED_DEFAULT 0
#define SIGNAL_STEERIG_DEFAULT 0


#define SPEED_CONTROLLER_DEFAULT_VALUE 0.5
#define SPEED_CONTROLLER_MAX_VALUE 2.
#define SPEED_CONTROLLER_MIN_VALUE 0.
#define SPEED_CONTROLLER_INCREMENT_VALUE 0.1

#define STEERING_ANGEL_DEFAULT_VALUE 85.
#define STEERING_ANGEL_MAX_VALUE 100.
#define STEERING_ANGEL_MIN_VALUE 0.
#define STEERING_ANGEL_INCREMENT_VALUE 5.

#define GOAL_GO_V_VALUE 3.
#define GOAL_GO_S_VALUE 5.

#define GOAL_STOP_V_VALUE 0.
#define GOAL_STOP_S_VALUE 0.5

// key for speed ang angle change
#define KEY_INC_SPEED 70         // f
#define KEY_DECREMENRT_SPEED 68  // d
#define KEY_INC_ANGEL 83         // s
#define KEY_DECREMENRT_ANGEL 65  // a
#define KEY_MAP_SAVE 80  // p
#define KEY_MAP_ADD_DEBUG_POINT 79  // o
#define KEY_LIGHT_CMD 76  // l


/*! This is the main class of the BoolValue Generator Plugin */
class KeyboardRemote : public QObject, public cBaseQtFilter
{
    /*! set the filter ID and version */
    ADTF_DECLARE_FILTER_VERSION(OID, FILTERNAME, OBJCAT_Auxiliary, "LIT Key Remote", 1, 0, 0, "");

    Q_OBJECT


public:
    KeyboardRemote(const tChar* __info);

    virtual ~KeyboardRemote();

    virtual tResult Init(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

    virtual tResult Start(ucom::IException** __exception_ptr = NULL);

    virtual tResult Stop(ucom::IException** __exception_ptr = NULL);

    virtual tResult Shutdown(tInitStage eStage, ucom::IException** __exception_ptr = NULL);

    tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr = NULL);


protected: // Implement cBaseQtFilter

    tHandle CreateView();

    tResult ReleaseView();

private:

    tResult CreateDescriptions(IException **__exception_ptr);


private:

    /*! The displayed widget*/
    DisplayWidget *m_pWidget;

    // speed
    cOutputPin        m_oSpeedPin;

    tBufferID         m_szIdOutputSpeedValue;
    tBufferID         m_szIdOutputSpeedTs;
    tBool             m_szIdOutputSpeedSet;

    // steering
    cOutputPin        m_oSteeringPin;

    tBufferID         m_szIdOutputSteeringValue;
    tBufferID         m_szIdOutputSteeringTs;
    tBool             m_szIdOutputSteeringSet;


    cObjectPtr<IMediaTypeDescription> m_pSpeedDescription;
    cObjectPtr<IMediaTypeDescription> m_pSteeringDescription;

protected:
    bool keyborad_enabled_;
    bool go_speed_enabled_front_;
    bool go_speed_enabled_back_;
    bool go_steering_enabled_left_;
    bool go_steering_enabled_right_;
    tFloat32 transmit_speed_;
    tFloat32 transmit_steering_;


public slots:
    /*! transmits a new mediasample with value false */
    void OnTransmitValueFalse();

    /*! transmits a new mediasample with value true */
    void OnTransmitValueTrue();

    void keycmd(int k);

    // binded to the car wigdet arrow keys
    void KeyDiveGo();
    void KeyDriveStop();
    void KeyDiveBackGo();
    void KeyBackStop();
    void KeyLeftGo();
    void KeyLeftStop();
    void KeyRightGo();
    void KeyRightStop();

private:
    // function to send speed
    void OnTransmitValuesDiveGo();
    void OnTransmitValuesDriveStop();
    void OnTransmitValuesDiveBackGo();
    void OnTransmitValuesDriveBackStop();
    void OnTransmitValuesLeftGo();
    void OnTransmitValuesLeftStop();
    void OnTransmitValuesRightGo();
    void OnTransmitValuesRightStop();


};

#endif /** @} */ // end of group
