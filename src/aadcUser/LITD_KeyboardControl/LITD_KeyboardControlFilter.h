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

#define CID_KEYBOARD_CONTROL  "litd_keyboard_control.filter.user.aadc.cid"

/*! forward declaration */
class cKeyboardControlWidget;

/*! the main class for the car controller filter. */
class cKeyboardControlFilter : public QObject, virtual public cQtUIFilter
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
	tResult SendSpeed(tFloat32 value);

public:
	ADTF_CLASS_ID_NAME(cKeyboardControlFilter, CID_KEYBOARD_CONTROL, "LITD Keyboard Control");

	ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::ui::ant::IQtXSystem),
		REQUIRE_INTERFACE(adtf::services::IReferenceClock));

private:

	/*! The output speed controller */
	cPinWriter m_oOutputSpeedController;
	/*! The output steering controller */
	cPinWriter m_oOutputSteeringController;

	/*! A signal value identifier. */
	struct tSignalValueId
	{
		tSize timeStamp;
		tSize value;
	} m_ddlSignalValueId;

	/*! The signal value sample factory */
	cSampleCodecFactory m_SignalValueSampleFactory;

	/*! The user interface file widget */
	cKeyboardControlWidget* m_pUiFileWidget = nullptr;

	/*! The mutex */
	std::mutex m_oMutex;

	/*! The reference clock */
	object_ptr<adtf::services::IReferenceClock> m_pClock;

	/*! The update interval of the QT GUI timer (to check for key presses) */
	property_variable<tFloat64> m_f64updateInterval = 50;

public:

	/*! Default constructor. */
	cKeyboardControlFilter();

	/*! Destructor. */
	virtual ~cKeyboardControlFilter() = default;

protected: // Implement cBaseQtFilter
	QWidget* CreateView() override;
	tVoid ReleaseView() override;

	tResult OnIdle() override;

	tResult OnTimer() override;

	tResult Init(tInitStage eStage) override;
};
