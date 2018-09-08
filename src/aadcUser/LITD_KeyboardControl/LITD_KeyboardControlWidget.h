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
#include "ui_litd_keyboard_control.h"

// keys for speed angle change
#define KEY_SPEED_INC						Qt::Key_W
#define KEY_SPEED_DEC						Qt::Key_S
#define KEY_ANGLE_INC						Qt::Key_D
#define KEY_ANGLE_DEC						Qt::Key_A

#define KEY_PRESS_OVERRIDE_TIMESTAMP std::numeric_limits<int64_t>::max()

// key-specific indexes in arrays
#define UP 0
#define DOWN 1
#define RIGHT 2
#define LEFT 3

/*! forward declaration */
class cKeyboardControlFilter;

class cKeyboardControlWidget : public QWidget, public Ui_KeyboardControlUi
{
Q_OBJECT

signals :
	void sendSpeed(tFloat32 value);
	void sendSteering(tFloat32 value);

public slots:
	void updateSignals();

public:
	void displaySpeed(tFloat32 value) const;
	void displaySteering(tFloat32 value) const;

private:

	enum eThrottleType
	{
		eStop = 0,
		eForward = 1,
		eBackward = 2
	};

	enum eSteeringType
	{
		eStraight = 0,
		eLeft = 1,
		eRight = 2
	};

	eThrottleType m_throttleType = eStop;
	eSteeringType m_steeringType = eStraight;

	tFloat64 m_currentSpeed;
	tFloat64 m_currentSteering;

	// debounce-fix for no machine remote desktop
	tTimeStamp m_tmLastKeyPressTimestamp[4] = {};
	tTimeStamp m_tmLastKeyReleaseTimestamp[4] = {};

	Ui_KeyboardControlUi* m_ui;
	cKeyboardControlFilter* m_filter;
	QTimer m_timer;

protected:
	void keyPressEvent(QKeyEvent* event);
	void keyReleaseEvent(QKeyEvent* event);
	void mousePressEvent(QMouseEvent* event);
	void focusOutEvent(QFocusEvent* event);


public:
	cKeyboardControlWidget(QWidget* parent = 0, cKeyboardControlFilter* filter = 0);

	~cKeyboardControlWidget();
};
