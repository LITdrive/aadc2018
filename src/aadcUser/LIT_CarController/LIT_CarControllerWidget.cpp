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

#include "LIT_CarControllerWidget.h"

cCarControllerWidget::cCarControllerWidget(QWidget* parent, tInt32 updateInterval) : QWidget(parent),
                                                                                     m_ui(new Ui_CarControllerUi)
{
	m_ui->setupUi(this);

	// check the key state rapidly
	m_timer.setInterval(updateInterval);
	connect(&m_timer, SIGNAL(timeout()), this, SLOT(updateSignals()));
	m_timer.start();
}

cCarControllerWidget::~cCarControllerWidget()
{
	delete m_ui;
}

void cCarControllerWidget::displaySpeed(tFloat32 value)
{
	m_ui->lcdNumber_throttle->display(value);
}

void cCarControllerWidget::displaySteering(tFloat32 value)
{
	m_ui->lcdNumber_steering->display(value);
}

// forward declaration
tTimeStamp GetTime();

void cCarControllerWidget::keyPressEvent(QKeyEvent* event)
{
	// only consider initial key presses
	if (!event->isAutoRepeat())
	{
		const int key = event->key();

		// W, A, S, D keys for changing the speed
		if (key == KEY_SPEED_INC)
		{
			m_currentSpeed = std::min(m_currentSpeed + SPEED_INCREMENT_VALUE,
			                          SPEED_MAX_VALUE);
		}
		else if (key == KEY_SPEED_DEC)
		{
			m_currentSpeed = std::max(-SPEED_MAX_VALUE,
			                          m_currentSpeed - SPEED_INCREMENT_VALUE);
		}
		else if (key == KEY_ANGLE_INC)
		{
			m_currentSteering = std::min(m_currentSteering + STEERING_OFFSET_INCREMENT_VALUE,
			                             STEERING_OFFSET_MAX_VALUE);
		}
		else if (key == KEY_ANGLE_DEC)
		{
			m_currentSteering = std::min(-STEERING_OFFSET_MAX_VALUE,
			                             m_currentSteering + STEERING_OFFSET_INCREMENT_VALUE);
		}

		// drive keys
		if (key == Qt::Key_Up)
		{
			m_throttleType = eForward;
		}
		else if (key == Qt::Key_Left)
		{
			m_steeringType = eLeft;
		}
		else if (key == Qt::Key_Right)
		{
			m_steeringType = eRight;
		}
		else if (key == Qt::Key_Down)
		{
			m_throttleType = eBackward;
		}

		// update gui
		if (m_throttleType != eStop)
		{
			displaySpeed(m_currentSpeed);
		}
		if (m_steeringType != eStraight)
		{
			displaySteering(m_currentSteering);
		}
	}
}

void cCarControllerWidget::keyReleaseEvent(QKeyEvent* event)
{
	// only consider initial key presses
	if (!event->isAutoRepeat())
	{
		// emit arrow-key release signals
		const int key = event->key();
		if (key == Qt::Key_Up)
		{
			m_throttleType = eStop;
			displaySpeed(0);
		}
		else if (key == Qt::Key_Left)
		{
			m_steeringType = eStraight;
			displaySteering(0);
		}
		else if (key == Qt::Key_Right)
		{
			m_steeringType = eStraight;
			displaySteering(0);
		}
		else if (key == Qt::Key_Down)
		{
			m_throttleType = eStop;
			displaySpeed(0);
		}
	}
}

void cCarControllerWidget::updateSignals()
{
	// ctrl is our "dead men key" because it can be polled easily
	if (Qt::ControlModifier == qApp->queryKeyboardModifiers())
	{
		switch (m_throttleType)
		{
		case eForward:
			emit sendSpeed(m_currentSpeed);
			break;
		case eBackward:
			emit sendSpeed(-m_currentSpeed);
			break;
		case eStop:
		default:
			emit sendSpeed(0);
			LOG_INFO("throttle default case");
		}

		switch (m_steeringType)
		{
		case eLeft:
			emit sendSteering(-m_currentSteering);
			break;
		case eRight:
			emit sendSteering(m_currentSteering);
			break;
		case eStraight:
		default:
			emit sendSteering(0);
			LOG_INFO("steering default case");
		}
	}
	else
	{
		// stop if the "dead men key" is not pressed
		emit sendSpeed(0);
		emit sendSteering(0);
	}
}

void cCarControllerWidget::mousePressEvent(QMouseEvent* event)
{
	// a simple click will not set the focus correctly ...
	setFocus();
}

void cCarControllerWidget::focusOutEvent(QFocusEvent* event)
{
	// force speed to zero
	m_throttleType = eStop;
	m_steeringType = eStraight;
}

tTimeStamp GetTime()
{
	return adtf_util::cHighResTimer::GetTime();
}
