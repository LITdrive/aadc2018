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

#include "LITD_KeyboardControlWidget.h"
#include "LITD_KeyboardControlFilter.h"

cKeyboardControlWidget::cKeyboardControlWidget(QWidget* parent, cKeyboardControlFilter* filter) :
	QWidget(parent),
	m_currentSpeed(filter->m_speed_default_value),
	m_currentSteering(filter->m_steering_offset_default_value),
	m_ui(new Ui_KeyboardControlUi),
	m_filter(filter)
{
	m_ui->setupUi(this);

	// check the key state rapidly
	m_timer.setInterval(m_filter->m_update_interval);
	connect(&m_timer, SIGNAL(timeout()), this, SLOT(updateSignals()));
	m_timer.start();
}

cKeyboardControlWidget::~cKeyboardControlWidget()
{
	delete m_ui;
}

void cKeyboardControlWidget::displaySpeed(tFloat32 value) const
{
	m_ui->lcdNumber_throttle->display(value);
}

void cKeyboardControlWidget::displaySteering(tFloat32 value) const
{
	m_ui->lcdNumber_steering->display(value);
}

// forward declaration
tTimeStamp GetTime();

void cKeyboardControlWidget::keyPressEvent(QKeyEvent* event)
{
	// only consider initial key presses
	if (!event->isAutoRepeat())
	{
		const int key = event->key();

		// W, A, S, D keys for changing the speed
		if (key == KEY_SPEED_INC)
		{
			m_currentSpeed = std::min(m_currentSpeed + m_filter->m_speed_increment_value,
			                          static_cast<double>(m_filter->m_speed_max_value));
		}
		else if (key == KEY_SPEED_DEC)
		{
			m_currentSpeed = std::max(-m_filter->m_speed_max_value,
			                          m_currentSpeed - m_filter->m_speed_increment_value);
		}
		else if (key == KEY_ANGLE_INC)
		{
			m_currentSteering = std::min(m_currentSteering + m_filter->m_steering_offset_increment_value,
			                             static_cast<double>(m_filter->m_steering_offset_max_value));
		}
		else if (key == KEY_ANGLE_DEC)
		{
			m_currentSteering = std::min(-m_filter->m_steering_offset_max_value,
			                             m_currentSteering + m_filter->m_steering_offset_increment_value);
		}

		// drive keys
		// register every key-press directly and override the release timestamp
		if (key == Qt::Key_Up)
		{
			m_throttleType = eForward;
			m_tmLastKeyReleaseTimestamp[UP] = KEY_PRESS_OVERRIDE_TIMESTAMP;
		}
		else if (key == Qt::Key_Left)
		{
			m_steeringType = eLeft;
			m_tmLastKeyReleaseTimestamp[LEFT] = KEY_PRESS_OVERRIDE_TIMESTAMP;
		}
		else if (key == Qt::Key_Right)
		{
			m_steeringType = eRight;
			m_tmLastKeyReleaseTimestamp[RIGHT] = KEY_PRESS_OVERRIDE_TIMESTAMP;
		}
		else if (key == Qt::Key_Down)
		{
			m_throttleType = eBackward;
			m_tmLastKeyReleaseTimestamp[DOWN] = KEY_PRESS_OVERRIDE_TIMESTAMP;
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

void cKeyboardControlWidget::keyReleaseEvent(QKeyEvent* event)
{
	// only consider initial key presses
	if (!event->isAutoRepeat())
	{
		// emit arrow-key release signals
		const int key = event->key();

		// save the timestamp of a key release and check in the updateSignals()
		// method, if the key was long enough released
		if (key == Qt::Key_Up)
		{
			m_tmLastKeyReleaseTimestamp[UP] = GetTime();
		}
		else if (key == Qt::Key_Left)
		{
			m_tmLastKeyReleaseTimestamp[LEFT] = GetTime();
		}
		else if (key == Qt::Key_Right)
		{
			m_tmLastKeyReleaseTimestamp[RIGHT] = GetTime();
		}
		else if (key == Qt::Key_Down)
		{
			m_tmLastKeyReleaseTimestamp[DOWN] = GetTime();
		}
	}
}

void cKeyboardControlWidget::updateSignals()
{
	// ctrl is our "dead men key" because it can be polled easily
	if (Qt::ControlModifier == qApp->queryKeyboardModifiers())
	{
		// only reset a key (stop throttle or steering) if the key release event happened at least MIN_DEBOUNCE_TIMEOUT [ms]
		// away and there was no key press in between (this fixes an issue with the no machine remote session)
		tTimeStamp now = GetTime();
		if ((now - m_tmLastKeyReleaseTimestamp[UP] > m_filter->m_min_debounce_timeout && m_throttleType == eForward) ||
			(now - m_tmLastKeyReleaseTimestamp[DOWN] > m_filter->m_min_debounce_timeout && m_throttleType == eBackward))
		{
			m_throttleType = eStop;
			displaySpeed(0);
		}

		if ((now - m_tmLastKeyReleaseTimestamp[LEFT] > m_filter->m_min_debounce_timeout && m_steeringType == eLeft) ||
			(now - m_tmLastKeyReleaseTimestamp[RIGHT] > m_filter->m_min_debounce_timeout && m_steeringType == eRight))
		{
			m_steeringType = eStraight;
			displaySteering(0);
		}

		// continuously send throttle
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
		}

		// continuously send steering
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
		}
	}
	else
	{
		// stop if the "dead men key" is not pressed
		emit sendSpeed(0);
		emit sendSteering(0);
	}
}

void cKeyboardControlWidget::mousePressEvent(QMouseEvent* event)
{
	// a simple click will not set the focus correctly ...
	setFocus();
}

void cKeyboardControlWidget::focusOutEvent(QFocusEvent* event)
{
	// force speed to zero
	m_throttleType = eStop;
	m_steeringType = eStraight;
}

tTimeStamp GetTime()
{
	return cHighResTimer::GetTime();
}
