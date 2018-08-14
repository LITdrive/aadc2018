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

#define CID_CAR_CONTROLLER  "lit_carcontroler.filter.user.aadc.cid"


#define SPEED_DEFAULT_VALUE 5.
#define SPEED_MAX_VALUE 15.
#define SPEED_MIN_VALUE 0.
#define SPEED_INCREMENT_VALUE .5

#define STEERING_ANGEL_DEFAULT_VALUE 50.
#define STEERING_ANGEL_MAX_VALUE 80.
#define STEERING_ANGEL_MIN_VALUE 0.
#define STEERING_ANGEL_INCREMENT_VALUE 2.

// key for speed ang angle change
#define KEY_INC_SPEED 87         // w 87
#define KEY_DECREMENRT_SPEED 83  // s 83
#define KEY_INC_ANGEL 68         // d 68
#define KEY_DECREMENRT_ANGEL 65  // a 65

#define KEY_MAP_SAVE 80  // p
#define KEY_MAP_ADD_DEBUG_POINT 79  // o
#define KEY_LIGHT_CMD 76  // l

/*! forward declaration */
class cCarControllerWidget;

/*! the main class for the car controller filter. */
class cCarControllerFilter : public QObject, virtual public cQtUIFilter
{
    Q_OBJECT

public slots:

    /*!
     * Sends a steering.
     *
     * \param   value   The value.
     *
     * \return  Standard Result Code.
     */
    tResult SendSteering(tFloat32 value);

    /*!
     * Sends a throttle.
     *
     * \param   value   The value.
     *
     * \return  Standard Result Code.
     */
    tResult SendThrottle(tFloat32 value);

    /*!
     * Toggle lights.
     *
     * \param   buttonId    Identifier for the button.
     *
     * \return  Standard Result Code.
     */
    tResult ToggleLights(int buttonId);

    // binded to the car wigdet arrow keys
    void keycmd(int key);

    void KeyDiveGo();
    void KeyDriveStop();
    void KeyDiveBackGo();
    void KeyBackStop();
    void KeyLeftGo();
    void KeyLeftStop();
    void KeyRightGo();
    void KeyRightStop();


public:
    ADTF_CLASS_ID_NAME(cCarControllerFilter, CID_CAR_CONTROLLER, "LIT Car Controller");
    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::ui::ant::IQtXSystem),
                            REQUIRE_INTERFACE(adtf::services::IReferenceClock));

private:

    /*! The output speed controller */
    cPinWriter     m_oOutputSpeedController;
    /*! The output steering controller */
    cPinWriter     m_oOutputSteeringController;

    /*! A signal value identifier. */
    struct tSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;

    tFloat32 transmit_speed_;
    tFloat32 transmit_steering_;


    /*! The signal value sample factory */
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;

    
    /*! The signal value sample factory */
    adtf::mediadescription::cSampleCodecFactory m_BoolSignalValueSampleFactory;

    /*! The user interface file widget */
    cCarControllerWidget*     m_pUiFileWidget;

     /*! The mutex */
    std::mutex m_oMutex;
   
    /*! The reference clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;
public:

    /*! Default constructor. */
    cCarControllerFilter();

    /*! Destructor. */
    virtual ~cCarControllerFilter();



protected: // Implement cBaseQtFilter
    QWidget * CreateView() override;
    tVoid    ReleaseView() override;

    tResult OnIdle() override;

    tResult  OnTimer() override;

    tResult  Init(tInitStage eStage) override;
};
